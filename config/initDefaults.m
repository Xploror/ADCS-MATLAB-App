function cfg = initDefaults()
%INITDEFAULTS Default user-level configuration (human units) of the comparison tool.
%
% Inputs:
%   (none)
% Outputs:
%   cfg - struct, user-level configuration (docs/DESIGN_SPEC.md 4.1) with
%         sub-structs scenario, gains, metrics, pass                     [mixed]

%% ===== Identification =====
cfg = struct();
cfg.format_version = 1;                        % [-] config file format version
cfg.name = 'Default';                          % [-] scenario name
cfg.description = 'Default configuration: 5 deg error, docking approach, GG + aero.';  % [-]

%% ===== Initial conditions and target =====
sc = struct();
sc.att_err_axis      = [1; 1; 1]/sqrt(3);      % [-]     initial error rotation axis (B w.r.t. R)
sc.att_err_angle_deg = 5;                      % [deg]   initial error rotation angle
sc.rate_err0_degps   = [0; 0; 0];              % [deg/s] body rate error w.r.t. reference at t=0
sc.target_mode       = 2;                      % [-]     0 inertial hold, 1 LVLH hold, 2 docking approach, 3 attitude scan
sc.q_inertial_ref    = [1; 0; 0; 0];           % [-]     inertial-hold reference quaternion (mode 0)

%% ===== Orbit and approach profile =====
sc.orbit_alt_km      = 400;                    % [km]    circular orbit altitude
sc.orbit_incl_deg    = 51.64;                  % [deg]   orbit inclination (ISS)
sc.R0_m              = 500;                    % [m]     initial chaser-to-ISS range
sc.Rf_m              = 10;                     % [m]     asymptotic final range
sc.T_close_s         = 600;                    % [s]     closing time constant
sc.y_off0_m          = 0;                      % [m]     initial cross-track offset
sc.z_off0_m          = 100;                    % [m]     initial radial offset (O3 axis)

%% ===== Inertia =====
sc.J_nom = [1200  100 -200;
             100 2200  300;
            -200  300 3100];                   % [kg*m^2] nominal inertia (Wie, Weiss & Arapostathis 1989)
sc.J_unc_pct         = 0;                      % [%]     true-inertia uncertainty magnitude
sc.J_unc_mode        = 1;                      % [-]     1 random signed per parameter, 2 uniform scale
sc.J_unc_seed        = 42;                     % [-]     RNG seed of the inertia perturbation

%% ===== Disturbances =====
sc.dist_enable       = [1 1 0 0];              % [-]     enable GG, Aero, SRP, Mag
sc.dist_scale        = [1 1 1 1];              % [-]     magnitude scale GG, Aero, SRP, Mag
sc.rho_kgm3          = 3e-12;                  % [kg/m^3] density at 400 km, nominal (USSA-76: 2.8e-12; range ~5e-13..2e-11 over the solar cycle)
sc.Cd                = 2.2;                    % [-]     drag coefficient
sc.A_aero_m2         = 10;                     % [m^2]   drag reference area
sc.r_cp_aero_m       = [0.10; 0.05; 0.0];      % [m]     aero centre-of-pressure offset from CG, B axes
sc.P_srp_Nm2         = 4.54e-6;                % [N/m^2] solar radiation pressure at 1 AU (TSI 1361 W/m^2 / c)
sc.refl_q            = 0.6;                    % [-]     reflectance factor
sc.A_srp_m2          = 10;                     % [m^2]   SRP area
sc.r_cp_srp_m        = [0.05; 0.10; 0.0];      % [m]     SRP centre-of-pressure offset, B axes
sc.sun_dir_N         = [1; 0; 0];              % [-]     direction to the Sun in N
sc.B0_T              = 2.97e-5;                % [T]     equatorial surface dipole field strength (IGRF-14, epoch 2025)
sc.m_res_Am2         = [0.5; 0.5; 0.5];        % [A*m^2] residual magnetic dipole, B axes

%% ===== Reaction wheels =====
sc.wheel_tau_max_Nm  = 0.15;                   % [N*m]   per-wheel torque limit (HR12-25-class wheel, derated)
sc.wheel_h_max_Nms   = 15;                     % [N*m*s] per-wheel momentum limit (HR12-25-class wheel, derated)
sc.h_w0_Nms          = [0; 0; 0];              % [N*m*s] initial wheel momentum

%% ===== Sensors =====
sc.noise_enable           = 0;                 % [-]     1 = noisy star tracker and gyro
sc.att_noise_std_deg      = 0.005;             % [deg]   star-tracker noise std (per axis)
sc.gyro_noise_std_degps   = 0.001;             % [deg/s] gyro white-noise std
sc.gyro_bias_rw_degps_rts = 1e-5;              % [deg/s/sqrt(s)] gyro bias random walk
sc.noise_seed             = 7;                 % [-]     noise RNG seed

%% ===== Simulation settings =====
sc.t_final_s         = 400;                    % [s]     simulation duration
sc.dt_s              = 0.1;                    % [s]     fixed step (RK4 / ode4)
sc.shortest_path     = 1;                      % [-]     1 = enforce q_e0 >= 0
sc.gyro_comp         = 1;                      % [-]     1 = add +w x h_w compensation
sc.k_norm            = 1.0;                    % [1/s]   quaternion constraint-stabilisation gain
sc.ref_fd_h_s        = 0.5;                    % [s]     finite-difference step of the reference
sc.scan_amp_deg      = [3; 3; 3];              % [deg]   mode-3 scan amplitudes about LVLH-hold axes
sc.scan_freq_radps   = [0.015; 0.020; 0.025];  % [rad/s] mode-3 scan frequencies (incommensurate)
cfg.scenario = sc;

%% ===== Gains =====
cfg.gains = struct();
cfg.gains.robust   = robust_default(sc.J_nom);
cfg.gains.adaptive = adaptive_default(sc.J_nom);
cfg.gains.baseline = baseline_default(sc.J_nom);

%% ===== Metrics and pass criteria =====
cfg.metrics = struct();
cfg.metrics.settle_thresh_deg = 0.5;           % [deg]   settling threshold on the error angle
cfg.metrics.ss_window_frac    = 0.1;           % [-]     final fraction of the run for the ss error
cfg.pass = struct();
cfg.pass.ss_err_max_deg    = 0.5;              % [deg]   maximum steady-state error
cfg.pass.settle_time_max_s = Inf;              % [s]     maximum settling time (Inf = not checked)
cfg.pass.qnorm_tol         = 1e-3;             % [-]     maximum |qnorm - 1|
end
