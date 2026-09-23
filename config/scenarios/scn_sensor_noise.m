function cfg = scn_sensor_noise()
%SCN_SENSOR_NOISE Nominal case with star-tracker noise, gyro noise and gyro bias drift.
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (see initDefaults)            [mixed]

%% ===== Base configuration =====
cfg = initDefaults();
cfg.name = 'Sensor noise';
cfg.description = 'Nominal approach with noisy star tracker (0.005 deg), gyro noise and bias random walk.';

%% ===== Scenario settings =====
cfg.scenario.noise_enable = 1;                 % [-]     noisy sensors on
cfg.scenario.t_final_s    = 400;               % [s]     duration

%% ===== Pass criteria =====
cfg.pass.ss_err_max_deg = 1.0;                 % [deg]   steady-state error limit
end
