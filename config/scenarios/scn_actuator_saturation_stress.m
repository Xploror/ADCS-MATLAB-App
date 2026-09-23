function cfg = scn_actuator_saturation_stress()
%SCN_ACTUATOR_SATURATION_STRESS 120 deg slew with halved wheel torque and momentum capacity.
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (see initDefaults)            [mixed]

%% ===== Base configuration =====
cfg = initDefaults();
cfg.name = 'Actuator saturation stress';
cfg.description = '120 deg error about [1 2 3]/|.|, LVLH hold, wheels limited to 0.075 N*m and 7.5 N*m*s, 2000 s.';

%% ===== Scenario settings =====
cfg.scenario.att_err_axis      = [1; 2; 3]/sqrt(14);  % [-]     error axis (unit)
cfg.scenario.att_err_angle_deg = 120;                 % [deg]   initial error angle
cfg.scenario.target_mode       = 1;                   % [-]     LVLH hold
cfg.scenario.wheel_tau_max_Nm  = 0.075;               % [N*m]   per-wheel torque limit
cfg.scenario.wheel_h_max_Nms   = 7.5;                 % [N*m*s] per-wheel momentum limit
cfg.scenario.t_final_s         = 2000;                % [s]     duration

%% ===== Pass criteria =====
cfg.pass.ss_err_max_deg = 1.0;                        % [deg]   steady-state error limit
end
