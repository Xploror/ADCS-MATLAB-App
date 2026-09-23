function cfg = scn_disturbance_stress()
%SCN_DISTURBANCE_STRESS All four disturbance sources enabled and scaled by 4.
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (see initDefaults)            [mixed]

%% ===== Base configuration =====
cfg = initDefaults();
cfg.name = 'Disturbance stress';
cfg.description = 'Nominal approach with GG, aero, SRP and magnetic torques all enabled and scaled x4, 600 s.';

%% ===== Scenario settings =====
cfg.scenario.dist_enable = [1 1 1 1];          % [-]     GG, Aero, SRP, Mag enabled
cfg.scenario.dist_scale  = [4 4 4 4];          % [-]     magnitude scale factors
cfg.scenario.t_final_s   = 600;                % [s]     duration

%% ===== Pass criteria =====
cfg.pass.ss_err_max_deg = 2.0;                 % [deg]   steady-state error limit
end
