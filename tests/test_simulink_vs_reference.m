function passed = test_simulink_vs_reference()
%TEST_SIMULINK_VS_REFERENCE Cross-check the Simulink harness against the MATLAB reference engine.
%
% Inputs:
%   (none)
% Outputs:
%   passed - logical [1], true if all checks pass or the test is skipped   [-]
%            (a failed check raises an error through assert)
%
% For scn_nominal with t_final reduced to 150 s, each controller (1, 2, 3)
% is simulated with runSimulation(cfg, k, 'simulink') and
% runSimulation(cfg, k, 'reference'). Pass criteria:
%   max |att_err_deg_simulink - att_err_deg_reference|   < 0.02 deg
%   |RMS(tau_rw_simulink) - RMS(tau_rw_reference)| / RMS(tau_rw_reference) < 1 %
% Both engines use fixed-step RK4 with the same dt, so the differences come
% only from the clamping implementation (limited integrator vs post-step clamp).
% Prints SKIPPED and returns true when Simulink is not available.

%% ===== Settings =====
t_final_test   = 150;                                    % [s] shortened run length
tol_att_deg    = 0.02;                                   % [deg] max attitude-error difference
tol_rms_rel    = 0.01;                                   % [-] max relative tau_rw RMS difference
passed = true;

%% ===== Skip if Simulink is unavailable =====
rootDir = fileparts(fileparts(mfilename('fullpath')));   % [-] project root
if exist('isSimulinkAvailable', 'file') ~= 2 || exist('scn_nominal', 'file') ~= 2
    if exist(fullfile(rootDir, 'startup_ADCS.m'), 'file') == 2
        run(fullfile(rootDir, 'startup_ADCS.m'));
    else
        addpath(genpath(fullfile(rootDir, 'src')));
        addpath(genpath(fullfile(rootDir, 'config')));
    end
end
if ~isSimulinkAvailable()
    fprintf('test_simulink_vs_reference: SKIPPED (Simulink not available)\n');
    return;
end

%% ===== Configuration =====
cfg = scn_nominal();
cfg.scenario.t_final_s = t_final_test;                   % [s]
names = controllerNames();

%% ===== Run and compare each controller =====
for k = 1:3
    rs = runSimulation(cfg, k, 'simulink');
    rr = runSimulation(cfg, k, 'reference');

    % --- attitude error: compare on the Simulink time base
    ea_s = rs.att_err_deg(:);
    ea_r = rr.att_err_deg(:);
    if numel(rs.t) ~= numel(rr.t) || max(abs(rs.t(:) - rr.t(:))) > 1e-9
        ea_r = interp1(rr.t(:), ea_r, rs.t(:), 'linear', 'extrap');
        tau_r = interp1(rr.t(:), rr.tau_rw, rs.t(:), 'linear', 'extrap');
    else
        tau_r = rr.tau_rw;
    end
    d_att = max(abs(ea_s - ea_r));                       % [deg]

    % --- wheel torque RMS over all axes and samples
    rms_s = sqrt(mean(rs.tau_rw(:).^2));                 % [N*m]
    rms_r = sqrt(mean(tau_r(:).^2));                     % [N*m]
    rel_rms = abs(rms_s - rms_r) / max(rms_r, eps);      % [-]
    rms_diff = sqrt(mean((rs.tau_rw(:) - tau_r(:)).^2)) / max(rms_r, eps); % [-] info only

    fprintf(['test_simulink_vs_reference: %-13s max|d att_err| = %.3g deg, ' ...
             'rel. RMS(tau_rw) diff = %.3g %%, RMS(d tau_rw)/RMS = %.3g %%\n'], ...
        names{k}, d_att, 100 * rel_rms, 100 * rms_diff);

    assert(d_att < tol_att_deg, ...
        'test_simulink_vs_reference: %s attitude-error mismatch %.4g deg >= %.4g deg', ...
        names{k}, d_att, tol_att_deg);
    assert(rel_rms < tol_rms_rel, ...
        'test_simulink_vs_reference: %s tau_rw RMS mismatch %.4g %% >= %.4g %%', ...
        names{k}, 100 * rel_rms, 100 * tol_rms_rel);
end
fprintf('test_simulink_vs_reference: PASSED\n');
end
