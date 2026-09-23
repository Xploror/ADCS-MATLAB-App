function cfg = scn_inertia_uncertainty_pm25()
%SCN_INERTIA_UNCERTAINTY_PM25 Nominal case with a seeded +-25% per-parameter inertia error.
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (see initDefaults)            [mixed]

%% ===== Base configuration =====
cfg = initDefaults();
cfg.name = 'Inertia uncertainty +-25%';
cfg.description = 'Nominal ICs; true inertia = J_nom with each parameter perturbed by up to +-25% (seed 42, physically valid).';

%% ===== Scenario settings =====
cfg.scenario.J_unc_pct   = 25;                 % [%]     per-parameter uncertainty magnitude
cfg.scenario.J_unc_mode  = 1;                  % [-]     random signed per parameter
cfg.scenario.J_unc_seed  = 42;                 % [-]     RNG seed
cfg.scenario.t_final_s   = 400;                % [s]     duration

%% ===== Pass criteria =====
cfg.pass.ss_err_max_deg = 1.0;                 % [deg]   steady-state error limit
end
