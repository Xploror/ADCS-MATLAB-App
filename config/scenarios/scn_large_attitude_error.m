function cfg = scn_large_attitude_error()
%SCN_LARGE_ATTITUDE_ERROR 150 deg initial error about [1 2 3] with LVLH hold.
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (see initDefaults)            [mixed]

%% ===== Base configuration =====
cfg = initDefaults();
cfg.name = 'Large attitude error';
cfg.description = '150 deg initial error about [1 2 3]/|.|, zero rate error, LVLH hold, 1500 s.';

%% ===== Scenario settings =====
cfg.scenario.att_err_axis      = [1; 2; 3]/sqrt(14);  % [-]     error axis (unit)
cfg.scenario.att_err_angle_deg = 150;                 % [deg]   initial error angle
cfg.scenario.rate_err0_degps   = [0; 0; 0];           % [deg/s] initial rate error
cfg.scenario.target_mode       = 1;                   % [-]     LVLH hold
cfg.scenario.t_final_s         = 1500;                % [s]     duration

%% ===== Pass criteria =====
cfg.pass.ss_err_max_deg = 0.5;                        % [deg]   steady-state error limit
end
