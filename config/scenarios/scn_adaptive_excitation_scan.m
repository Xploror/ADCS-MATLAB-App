function cfg = scn_adaptive_excitation_scan()
%SCN_ADAPTIVE_EXCITATION_SCAN Persistently exciting attitude scan with a +-25% inertia error (adaptation showcase).
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (see initDefaults)            [mixed]
%
% Rationale: in the docking-approach scenarios the reference is almost
% stationary in LVLH, so the regressor Y is not persistently exciting and
% the inertia estimate cannot converge (notes.md caveat L3). This scenario
% commands a 3-axis sinusoidal scan about the LVLH-hold attitude with
% incommensurate frequencies, which excites all six inertia parameters.
% A faster adaptation gain is used here (Gamma = 5e9) because excitation is
% present; with this gain and NO excitation the estimate drifts (see the
% technical report, section `Adaptation gain versus excitation`).

%% ===== Base configuration =====
cfg = initDefaults();
cfg.name = 'Adaptive showcase: PE attitude scan, inertia +-25%';
cfg.description = ['3-axis sinusoidal scan (3 deg, 0.015/0.020/0.025 rad/s) about LVLH hold; ', ...
                   'true inertia perturbed +-25% (seed 42); adaptive Gamma = 5e9.'];

%% ===== Scenario settings =====
cfg.scenario.target_mode      = 3;                        % [-]     attitude scan about LVLH hold
cfg.scenario.scan_amp_deg     = [3; 3; 3];                % [deg]   scan amplitudes
cfg.scenario.scan_freq_radps  = [0.015; 0.020; 0.025];    % [rad/s] incommensurate scan frequencies
cfg.scenario.J_unc_pct        = 25;                       % [%]     per-parameter uncertainty magnitude
cfg.scenario.J_unc_mode       = 1;                        % [-]     random signed per parameter
cfg.scenario.J_unc_seed       = 42;                       % [-]     RNG seed
cfg.scenario.t_final_s        = 1600;                     % [s]     duration (about 4 periods of the slowest axis)

%% ===== Gains specific to this scenario =====
cfg.gains.adaptive.Gamma_diag = 5e9*ones(6,1);            % [kg*m^2*s^2] fast adaptation (excitation present)

%% ===== Pass criteria =====
cfg.pass.ss_err_max_deg = 0.5;                            % [deg]   mean tracking error over the last 10 %
end
