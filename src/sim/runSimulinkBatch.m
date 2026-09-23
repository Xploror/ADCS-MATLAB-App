function results = runSimulinkBatch(SC, RG, AG, BG, ctrl_ids)
%RUNSIMULINKBATCH Run the Simulink harness once per controller and return result structs.
%
% Inputs:
%   SC       - struct, sim-level spacecraft/scenario parameters (spec 4.2) [mixed SI]
%   RG       - struct, robust SMC gains (RobustGains)                      [mixed SI]
%   AG       - struct, adaptive SMC gains (AdaptiveGains)                  [mixed SI]
%   BG       - struct, PD benchmark gains (BaselineGains)                  [mixed SI]
%   ctrl_ids - double [1xK], controller ids to run (1, 2 and/or 3)         [-]
% Outputs:
%   results  - struct [1xK], result structs (spec section 6), one per
%              element of ctrl_ids, engine = 'simulink'                     [mixed]
%
% One Simulink.SimulationInput is built per controller. Variables SC,
% RobustGains, AdaptiveGains, BaselineGains and CONTROLLER_SELECT override
% the data-dictionary values, and StopTime / FixedStep are set numerically.
% parsim is used only when there is more than one run, the Parallel
% Computing Toolbox is licensed AND a parallel pool is already open
% (gcp('nocreate') non-empty); this function never starts a pool. It also
% honours the instance limit (PROJECT_RULES R8): no parsim when the MATLAB
% desktop (GUI) is running, and headless at most 2 pool workers (3 MATLAB
% instances in total). Otherwise,
% or if parsim throws or any individual parsim run reports an ErrorMessage,
% the batch runs sequentially with sim.

%% ===== Checks and paths =====
if nargin < 5 || isempty(ctrl_ids)
    ctrl_ids = [1 2 3];                                  % [-] default: all three controllers
end
ctrl_ids = double(ctrl_ids(:)');
if any(~ismember(ctrl_ids, [1 2 3]))
    error('runSimulinkBatch:badCtrl', 'ctrl_ids must contain only 1, 2 or 3.');
end
mdl     = ensureModelBuilt();                            % builds (if needed) and loads the model
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath')))); % [-] project root
K       = numel(ctrl_ids);                               % [-] number of runs

%% ===== Build the SimulationInput array =====
for k = 1:K
    ink = Simulink.SimulationInput(mdl);
    ink = ink.setVariable('SC', SC);
    ink = ink.setVariable('RobustGains', RG);
    ink = ink.setVariable('AdaptiveGains', AG);
    ink = ink.setVariable('BaselineGains', BG);
    ink = ink.setVariable('CONTROLLER_SELECT', ctrl_ids(k));
    ink = ink.setModelParameter('StopTime', sprintf('%.17g', SC.t_final), ...   % [s] full double precision
                                'FixedStep', sprintf('%.17g', SC.dt));          % [s] full double precision
    if k == 1
        in = ink;
    else
        in(k) = ink; %#ok<AGROW>
    end
end

%% ===== Decide on parsim (only with an already open pool) =====
usePar = false;                                          % [-] true -> try parsim first
if K > 1
    try
        hasPCT = logical(license('test', 'Distrib_Computing_Toolbox')) && ~isempty(ver('parallel'));
        pool   = gcp('nocreate');                        % [-] existing pool only; never start one here
        usePar = hasPCT && ~isempty(pool);
        %% ===== Instance-limit guard (PROJECT_RULES R8) =====
        % A GUI MATLAB session may be the only MATLAB instance, so no pool
        % workers are used. Headless, the client plus its workers must stay
        % at or below 3 instances, i.e. at most 2 workers.
        maxWorkersHeadless = 2;                          % [-] 3 instances total minus the client
        if usePar && (usejava('desktop') || pool.NumWorkers > maxWorkersHeadless)
            usePar = false;
        end
    catch
        usePar = false;
    end
end

%% ===== Run with parsim (falls back to sim on any failure) =====
tStart = tic;
out = [];
if usePar
    try
        % Workers need src/, config/ and models/ (dictionary) on their path.
        setupFcn = @() addpath(genpath(rootDir));
        out = parsim(in, 'ShowProgress', 'off', ...
                         'TransferBaseWorkspaceVariables', 'off', ...
                         'SetupFcn', setupFcn);
        errMsgs = {out.ErrorMessage};                    % per-run error messages ('' on success)
        if any(~cellfun(@isempty, errMsgs))
            warning('runSimulinkBatch:parsimRunError', ...
                'At least one parsim run failed (%s); re-running the batch sequentially with sim.', ...
                errMsgs{find(~cellfun(@isempty, errMsgs), 1)});
            out = [];
            tStart = tic;                                % restart timing for the sequential run
        end
    catch err
        warning('runSimulinkBatch:parsimFailed', ...
            'parsim failed (%s); running the batch sequentially with sim.', collectCauses(err, 0));
        out = [];
        tStart = tic;                                    % restart timing for the sequential run
    end
end

%% ===== Sequential run with sim =====
if isempty(out)
    try
        out = sim(in);
    catch err
        % Keep the original exception (and its stack) as the cause (critic F10).
        ME = MException('runSimulinkBatch:simFailed', 'Simulink run of %s failed: %s', mdl, ...
                        collectCauses(err, 0));
        ME = addCause(ME, err);
        throw(ME);
    end
end
wall_total = toc(tStart);                                % [s] wall-clock time of the whole batch

%% ===== Check errors and extract =====
cells = cell(1, K);
for k = 1:K
    msg = '';
    try
        msg = out(k).ErrorMessage;
    catch
        % no ErrorMessage property - treat as success
    end
    if ~isempty(msg)
        % Array runs report errors in the output, not by throwing: expand the
        % nested diagnostic causes when available (critic F6).
        try
            msg = collectCauses(out(k).SimulationMetadata.ExecutionInfo.ErrorDiagnostic.Diagnostic, 0);
        catch
            % keep the flat ErrorMessage
        end
        error('runSimulinkBatch:runError', 'Simulation for controller %d failed: %s', ...
            ctrl_ids(k), msg);
    end
    res = extractSimulinkResults(out(k), SC, RG, AG, BG, ctrl_ids(k));
    if isnan(res.wallclock_s)
        res.wallclock_s = wall_total / K;                % [s] fallback: share of batch time
    end
    cells{k} = res;
end
results = [cells{:}];
end

function txt = collectCauses(err, depth)
%COLLECTCAUSES Flatten an MException and all its nested causes into one message.
%
% Inputs:
%   err   - MException / MSLException / MSLDiagnostic, the error         [-]
%   depth - double scalar, nesting level (0 for the top-level error)     [-]
% Outputs:
%   txt   - char, the message and every nested cause, indented by level  [-]

%% ===== This level =====
pad = repmat('  ', 1, depth);                            % [-] indentation for this level
if isempty(err.identifier)
    txt = sprintf('%s%s', pad, err.message);             % no empty "[]" prefix (critic F9)
else
    txt = sprintf('%s[%s] %s', pad, err.identifier, err.message);
end

%% ===== Nested causes (Simulink "multiple causes" errors) =====
causes = {};
try
    causes = err.cause;                                  % cell array of MExceptions
catch
end
for k = 1:numel(causes)
    txt = sprintf('%s\n%s', txt, collectCauses(causes{k}, depth + 1));
end
end
