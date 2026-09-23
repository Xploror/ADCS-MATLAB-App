function cfg = scn_nominal()
%SCN_NOMINAL Nominal docking approach: 5 deg initial error, GG + aero, ideal sensors.
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (see initDefaults)            [mixed]

%% ===== Base configuration =====
cfg = initDefaults();
cfg.name = 'Nominal';
cfg.description = 'Docking approach (mode 2), 5 deg initial error about [1 1 1], GG + aero, ideal sensors, 400 s.';

%% ===== Scenario settings =====
cfg.scenario.att_err_angle_deg = 5;            % [deg]   initial attitude error
cfg.scenario.target_mode       = 2;            % [-]     docking approach
cfg.scenario.t_final_s         = 400;          % [s]     duration

%% ===== Pass criteria =====
cfg.pass.ss_err_max_deg = 0.5;                 % [deg]   steady-state error limit
end
