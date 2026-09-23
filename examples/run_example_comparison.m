%RUN_EXAMPLE_COMPARISON Compare the three attitude controllers on the nominal scenario.
%
% Inputs:
%   (none; script)
% Outputs:
%   (none; prints a metrics table and opens a 2x3 figure)
%
% Uses engine 'auto': Simulink when available, otherwise the RK4 reference
% engine (MATLAB or GNU Octave).

%% ===== Paths =====
if exist('runSimulation', 'file') ~= 2
    run(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'startup_ADCS.m'));
end

%% ===== Configuration =====
cfg = scn_nominal();                 % [-] nominal docking-approach scenario
ctrl_ids = [1 2 3];                  % [-] robust SMC, adaptive SMC, PD benchmark
engine = 'auto';                     % [-] 'auto' | 'simulink' | 'reference'

%% ===== Simulation =====
results = runSimulation(cfg, ctrl_ids, engine);
fprintf('Engine used: %s\n', results(1).engine);

%% ===== Metrics =====
K = numel(results);
passFlags = false(1, K);
for k = 1:K
    M = computeMetrics(results(k), cfg);
    if k == 1
        metrics = repmat(M, 1, K);
    end
    metrics(k) = M;
    passFlags(k) = evaluatePassFail(M, cfg.pass);
end
[header, rows] = metricsTable(results, metrics, passFlags);
showCols = [1 3 4 7 10 15 17];       % [-] controller, settle, ss, peak torque, peak h, theta err, pass
for c = showCols
    fprintf('%-18s', header{c});
end
fprintf('\n');
for k = 1:K
    for c = showCols
        v = rows{k, c};
        if ischar(v)
            fprintf('%-18s', v);
        else
            fprintf('%-18.4g', v);
        end
    end
    fprintf('\n');
end

%% ===== Plots =====
quantities = {'att_err', 'w_e', 'tau_rw', 'h_w', 'qnorm', 'theta_hat'};   % [-] one panel each
fig = figure('Name', ['ADCS comparison: ' cfg.name]);
for p = 1:numel(quantities)
    ax = subplot(2, 3, p, 'Parent', fig);
    plotResultsOnAxes(ax, results, quantities{p}, cfg);
end
