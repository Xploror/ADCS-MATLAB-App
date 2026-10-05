function res = extractSimulinkResults(simOut, SC, RG, AG, BG, ctrl_id)
% Convert a Simulink.SimulationOutput into the engine-independent result struct.
%
% Inputs:
%   simOut  - Simulink.SimulationOutput, output of one harness run         
%   SC      - struct, sim-level parameters used for the run                [mixed SI]
%   RG      - struct, RobustGains used for the run                         [mixed SI]
%   AG      - struct, AdaptiveGains used for the run                       [mixed SI]
%   BG      - struct, BaselineGains used for the run                       [mixed SI]
%   ctrl_id - double [1], controller id of the run (1, 2 or 3)             
% Outputs:
%   res     - struct [1x1], fields of spec section 6 in this order:
%             t [Nx1] s, q [Nx4], w [Nx3] rad/s, q_r [Nx4], w_r [Nx3] rad/s,
%             q_e [Nx4], w_e [Nx3] rad/s, att_err_deg [Nx1] deg, s [Nx3] rad/s,
%             tau_cmd [Nx3] N*m, tau_rw [Nx3] N*m, h_w [Nx3] N*m*s,
%             theta_hat [Nx6] kg*m^2, tau_d/tau_gg/tau_aero/tau_srp/tau_mag
%             [Nx3] N*m, qnorm [Nx1] , sat_flags [Nx6] , then metadata
%             ctrl_id, ctrl_name, engine ('simulink'), SC, RG, AG, BG,
%             wallclock_s [s] (NaN if the run metadata has no timing).    [mixed]

%% ===== Get the logged dataset =====
logsout = [];
try
    logsout = simOut.logsout;
catch
    try
        logsout = simOut.get('logsout');
    catch
        % handled below
    end
end
if isempty(logsout)
    error('extractSimulinkResults:noLogsout', ...
        'No logsout in the simulation output (is SignalLogging on in the model?).');
end

%% ===== Signal list (name, expected width) =====
sigs = { 'q', 4;  'w', 3;  'q_r', 4;  'w_r', 3;  'q_e', 4;  'w_e', 3;
         'att_err_deg', 1;  's', 3;  'tau_cmd', 3;  'tau_rw', 3;  'h_w', 3;
         'theta_hat', 6;  'tau_d', 3;  'tau_gg', 3;  'tau_aero', 3;
         'tau_srp', 3;  'tau_mag', 3;  'qnorm', 1;  'sat_flags', 6 };

%% ===== Time base from 'q' =====
tsq = getLoggedTimeseries(logsout, 'q');
t   = double(tsq.Time(:));                               % [s] common time vector
N   = numel(t);

%% ===== Extract all signals =====
res = struct();
res.t = t;
for i = 1:size(sigs, 1)
    name = sigs{i,1};
    ts   = getLoggedTimeseries(logsout, name);
    X    = toRowMajor(ts, name);
    tt   = double(ts.Time(:));
    if numel(tt) ~= N || max(abs(tt - t)) > 1e-9 * max(1, max(abs(t)))
        % Different time base (should not happen with a single rate): resample.
        warning('extractSimulinkResults:resample', ...
            'Signal %s has a different time base; resampling onto the time of q.', name);
        [tt, iu] = unique(tt);
        X = interp1(tt, X(iu, :), t, 'linear', 'extrap');
    end
    if size(X, 2) ~= sigs{i,2}
        error('extractSimulinkResults:width', 'Signal %s has width %d, expected %d.', ...
            name, size(X, 2), sigs{i,2});
    end
    res.(name) = X;
end

%% ===== Metadata =====
names = controllerNames();
res.ctrl_id   = ctrl_id;
res.ctrl_name = names{ctrl_id};
res.engine    = 'simulink';
res.SC = SC;
res.RG = RG;
res.AG = AG;
res.BG = BG;
res.wallclock_s = NaN;                                   % [s] filled below if available
try
    res.wallclock_s = simOut.SimulationMetadata.TimingInfo.TotalElapsedWallTime;
catch
    % timing metadata unavailable - caller may fill it in
end
end

%% ===================================================================
%% ===== Local helper functions =====
%% ===================================================================

function ts = getLoggedTimeseries(logsout, name)
% Return the timeseries of a named element of a logsout Dataset.
%
% Inputs:
%   logsout - Simulink.SimulationData.Dataset, logged signals             
%   name    - char, signal (element) name                                 
% Outputs:
%   ts      - timeseries, logged values of that signal                    

%% ===== Look up the element by name =====
el = [];
try
    el = logsout.getElement(name);
catch
    % handled below
end

%% ===== Report a missing signal =====
if isempty(el)
    try
        availNames = logsout.getElementNames();
        avail = strjoin(availNames(:)', ', ');
    catch
        avail = '(could not list)';
    end
    error('extractSimulinkResults:missingSignal', ...
        'Logged signal ''%s'' not found. Available: %s', name, avail);
end

%% ===== Return the values =====
if isa(el, 'Simulink.SimulationData.Dataset')
    % Several elements share the name - use the first one.
    el = el.getElement(1);
end
ts = el.Values;
end

function X = toRowMajor(ts, name)
% Convert timeseries data ([N x n], [n x 1 x N] or [1 x n x N]) to [N x n].
%
% Inputs:
%   ts   - timeseries, logged signal                                      
%   name - char, signal name (for error messages)                         
% Outputs:
%   X    - double [N x n], one row per time sample                        

%% ===== Raw data and sample count =====
d = double(ts.Data);
N = numel(ts.Time);

%% ===== Reshape to [N x n] by layout =====
if ndims(d) == 3
    % Time along the 3rd dimension: [a x b x N] -> [N x a*b].
    if size(d, 3) ~= N
        error('extractSimulinkResults:shape', 'Signal %s: 3-D data with %d pages, %d samples.', ...
            name, size(d, 3), N);
    end
    X = reshape(d, size(d, 1) * size(d, 2), N).';
elseif size(d, 1) == N
    X = d;                                               % already [N x n] (time first)
elseif size(d, 2) == N
    % [n x N]: a 3-D array with a single page collapses to 2-D, or data was
    % stored time-last as a matrix.
    X = d.';
else
    error('extractSimulinkResults:shape', 'Signal %s: data of size %s does not match %d samples.', ...
        name, mat2str(size(d)), N);
end
end
