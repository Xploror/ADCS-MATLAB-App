function tbl = run_all_scenarios_smoketest(quick)
%RUN_ALL_SCENARIOS_SMOKETEST Run every scenario with every controller on the reference engine.
%
% Inputs:
%   quick - logical/double [1], optional; true scales every t_final by 0.25
%           for a fast check (pass verdicts then are indicative only)   [-]
% Outputs:
%   tbl   - struct [1x(K*3)], one element per scenario/controller (K = size(listScenarios(),1), currently 7) with fields
%           scenario (char), controller (char), settle_time_s [s], ss_err_deg [deg],
%           peak_torque_Nm [N*m], peak_h_Nms [N*m*s], theta_err_final_pct [%],
%           qnorm_max_dev [-], all_finite (logical), pass (logical),
%           reasons (cell of char), M (full metrics struct)
%
% Throws an error (after printing the table) if any run produced non-finite
% values or violated the quaternion-norm health tolerance. Pass/fail against
% the scenario ss-error limit is reported but does not throw.

%% ===== Setup =====
if nargin < 1 || isempty(quick)
    quick = false;                                       % [-] default: full-length runs
end
scen = listScenarios();
ctrl_ids = [1 2 3];                            % [-] all controllers
tbl = struct('scenario', {}, 'controller', {}, 'settle_time_s', {}, 'ss_err_deg', {}, ...
             'peak_torque_Nm', {}, 'peak_h_Nms', {}, 'theta_err_final_pct', {}, ...
             'qnorm_max_dev', {}, 'all_finite', {}, 'pass', {}, 'reasons', {}, 'M', {});
bad = {};

%% ===== Run all scenarios =====
for i = 1:size(scen, 1)
    cfg = feval(scen{i,1});
    if quick
        cfg.scenario.t_final_s = 0.25*cfg.scenario.t_final_s;
    end
    fprintf('Running %-32s (t_final %6.0f s) ...', scen{i,1}, cfg.scenario.t_final_s);
    tic;
    results = runSimulation(cfg, ctrl_ids, 'reference');
    fprintf(' %.1f s\n', toc);
    for k = 1:numel(results)
        M = computeMetrics(results(k), cfg);
        [ok, why] = evaluatePassFail(M, cfg.pass);
        e = struct();
        e.scenario = scen{i,1};
        e.controller = results(k).ctrl_name;
        e.settle_time_s = M.settle_time_s;
        e.ss_err_deg = M.ss_err_deg;
        e.peak_torque_Nm = M.peak_torque_Nm;
        e.peak_h_Nms = M.peak_h_Nms;
        e.theta_err_final_pct = M.theta_err_final_pct;
        e.qnorm_max_dev = M.qnorm_max_dev;
        e.all_finite = M.all_finite;
        e.pass = ok;
        e.reasons = why;
        e.M = M;
        tbl(end+1) = e; %#ok<AGROW>
        if ~M.all_finite || ~(M.qnorm_max_dev <= cfg.pass.qnorm_tol)
            bad{end+1} = sprintf('%s / %s', scen{i,1}, results(k).ctrl_name); %#ok<AGROW>
        end
    end
end

%% ===== Print table =====
fprintf('\n%-32s %-13s %9s %9s %8s %8s %8s %9s %s\n', 'Scenario', 'Controller', ...
        'Settle[s]', 'SS[deg]', 'PkTq[Nm]', 'PkH[Nms]', 'Th err%', 'qdev', 'Result');
for j = 1:numel(tbl)
    e = tbl(j);
    if e.pass
        verdict = 'PASS';
    else
        verdict = ['FAIL (' strjoin(e.reasons, '; ') ')'];
    end
    fprintf('%-32s %-13s %9.1f %9.4f %8.4f %8.3f %8.2f %9.1e %s\n', e.scenario, e.controller, ...
            e.settle_time_s, e.ss_err_deg, e.peak_torque_Nm, e.peak_h_Nms, ...
            e.theta_err_final_pct, e.qnorm_max_dev, verdict);
end
fprintf('\n%d of %d runs pass their scenario criteria.\n', sum([tbl.pass]), numel(tbl));

%% ===== Health assertions =====
if ~isempty(bad)
    error('run_all_scenarios_smoketest:health', 'Non-finite or qnorm-unhealthy runs: %s', strjoin(bad, ', '));
end
end
