function [SC, RG, AG, BG, info] = buildSimParams(cfg)
%BUILDSIMPARAMS Convert a user-level config into numeric SI simulation structs.
%
% Inputs:
%   cfg - struct, user-level configuration (docs/DESIGN_SPEC.md 4.1)     [mixed]
% Outputs:
%   SC   - struct, sim constants (numeric only, SI; spec 4.2)            [mixed SI]
%   RG   - struct, robust gains lambda_diag, K_diag, eta, phi            [SI]
%   AG   - struct, adaptive gains incl. Gamma_diag, theta_hat0,
%          theta_min, theta_max                                          [SI]
%   BG   - struct, baseline gains Kp_diag, Kd_diag                       [SI]
%   info - struct, J_true_physical (logical), q_r0 [4x1], w_r0 [3x1],
%          J_unc_tries [-], notes (char)                                 [mixed]

sc = cfg.scenario;                                          % [-] user-level scenario sub-struct
d2r = pi/180;                                               % [rad/deg] degree-to-radian factor

%% ===== Orbit, reference and simulation constants =====
SC = struct();
SC.dt       = double(sc.dt_s);                              % [s] fixed integration step
SC.t_final  = double(sc.t_final_s);                         % [s] simulation stop time
SC.mode     = double(sc.target_mode);                       % [-] reference mode: 0 inertial, 1 LVLH, 2 docking approach, 3 attitude scan
SC.q_inertial_ref = qNormalize(double(sc.q_inertial_ref(:))); % [-] inertial reference quaternion N->B (mode 0), scalar-first
SC.mu       = 3.986004418e14;                               % [m^3/s^2] Earth gravitational parameter (WGS-84)
SC.R_E      = 6378137;                                      % [m] Earth equatorial radius (WGS-84)
SC.R_orb    = SC.R_E + 1e3*double(sc.orbit_alt_km);         % [m] circular-orbit radius
SC.n        = sqrt(SC.mu/SC.R_orb^3);                       % [rad/s] orbital mean motion
SC.incl     = double(sc.orbit_incl_deg)*d2r;                % [rad] orbit inclination
SC.R0       = double(sc.R0_m);                              % [m] initial chaser-to-ISS range
SC.Rf       = double(sc.Rf_m);                              % [m] final (asymptotic) range
SC.T_close  = double(sc.T_close_s);                         % [s] closing time constant of the range profile
SC.y0       = double(sc.y_off0_m);                          % [m] initial cross-track offset (LVLH)
SC.z0       = double(sc.z_off0_m);                          % [m] initial radial offset (LVLH)
SC.ref_fd_h = double(sc.ref_fd_h_s);                        % [s] finite-difference step for reference rates
SC.scan_amp  = double(sc.scan_amp_deg(:))*pi/180;           % [rad] mode-3 scan amplitudes (rotation-vector components)
SC.scan_freq = double(sc.scan_freq_radps(:));               % [rad/s] mode-3 scan angular frequencies

%% ===== Inertia (nominal and true) =====
J_nom = double(sc.J_nom);                                   % [kg*m^2] nominal inertia
J_nom = 0.5*(J_nom + J_nom.');                              % [kg*m^2] enforce symmetry
[J_true, phys, tries] = localTrueInertia(J_nom, double(sc.J_unc_pct), double(sc.J_unc_mode), double(sc.J_unc_seed));
SC.J_nom      = J_nom;                                      % [kg*m^2] nominal inertia (controller model)
SC.J_true     = J_true;                                     % [kg*m^2] true inertia (plant)
SC.theta_nom  = inertiaToTheta(J_nom);                      % [kg*m^2] nominal inertia parameter vector [6x1]
SC.theta_true = inertiaToTheta(J_true);                     % [kg*m^2] true inertia parameter vector [6x1]

%% ===== Placeholders for the initial state (field order per spec) =====
SC.q0   = [1; 0; 0; 0];                                     % [-] placeholder, set below from the reference
SC.w0   = zeros(3,1);                                       % [rad/s] placeholder, set below from the reference
SC.h_w0 = double(sc.h_w0_Nms(:));                           % [N*m*s] initial wheel momentum, body axes

%% ===== Actuators, disturbances, sensors, options =====
SC.tau_max     = double(sc.wheel_tau_max_Nm);               % [N*m] per-wheel torque limit
SC.h_max       = double(sc.wheel_h_max_Nms);                % [N*m*s] per-wheel momentum limit
SC.dist_enable = reshape(double(sc.dist_enable), 1, 4);     % [-] enable flags: GG, aero, SRP, magnetic
SC.dist_scale  = reshape(double(sc.dist_scale), 1, 4);      % [-] magnitude scale: GG, aero, SRP, magnetic
SC.rho         = double(sc.rho_kgm3);                       % [kg/m^3] atmospheric density
SC.Cd          = double(sc.Cd);                             % [-] drag coefficient
SC.A_aero      = double(sc.A_aero_m2);                      % [m^2] aerodynamic reference area
SC.r_cp_aero   = double(sc.r_cp_aero_m(:));                 % [m] aero centre-of-pressure offset, body axes
SC.P_srp       = double(sc.P_srp_Nm2);                      % [N/m^2] solar radiation pressure
SC.refl_q      = double(sc.refl_q);                         % [-] reflectance factor
SC.A_srp       = double(sc.A_srp_m2);                       % [m^2] SRP reference area
SC.r_cp_srp    = double(sc.r_cp_srp_m(:));                  % [m] SRP centre-of-pressure offset, body axes
sun = double(sc.sun_dir_N(:));                              % [-] Sun direction in N (unnormalised)
SC.sun_N       = sun/max(norm(sun), eps);                   % [-] unit Sun direction in N
SC.B0          = double(sc.B0_T);                           % [T] equatorial surface dipole field strength
SC.m_res       = double(sc.m_res_Am2(:));                   % [A*m^2] residual magnetic dipole, body axes
SC.noise_enable = double(sc.noise_enable);                  % [-] sensor-noise enable flag
SC.sig_att     = double(sc.att_noise_std_deg)*d2r;          % [rad] attitude-sensor noise std
SC.sig_gyro    = double(sc.gyro_noise_std_degps)*d2r;       % [rad/s] gyro white-noise std
SC.sig_bias_rw = double(sc.gyro_bias_rw_degps_rts)*d2r;     % [rad/s/sqrt(s)] gyro bias random-walk intensity
SC.noise_seed  = double(sc.noise_seed);                     % [-] RNG seed of the sensor noise
SC.shortest_path = double(sc.shortest_path);                % [-] 1 = enforce q_e0 >= 0 (shortest path)
SC.gyro_comp   = double(sc.gyro_comp);                      % [-] 1 = add +w x h_w compensation
SC.k_norm      = double(sc.k_norm);                         % [1/s] quaternion-norm stabilisation gain

%% ===== Initial attitude and rate =====
[q_r0, w_r0] = referenceAttitude(0, SC);
dq = qFromAxisAngle(double(sc.att_err_axis(:)), double(sc.att_err_angle_deg)*d2r); % [-] initial attitude-error quaternion
SC.q0 = qNormalize(qMult(dq, q_r0));                        % [-] initial attitude N->B (error applied to the reference)
SC.w0 = qToDCM(dq)*w_r0 + double(sc.rate_err0_degps(:))*d2r; % [rad/s] initial body rate w.r.t. N

%% ===== Gain structs =====
g = cfg.gains;                                              % [-] user-level gain sub-struct
RG = struct();
RG.lambda_diag = double(g.robust.lambda_diag(:));           % [1/s] sliding-surface slope
RG.K_diag      = double(g.robust.K_diag(:));                % [N*m*s] linear feedback on s
RG.eta         = double(g.robust.eta(:));                   % [N*m] switching gain
RG.phi         = double(g.robust.phi(:));                   % [rad/s] boundary-layer thickness

AG = struct();
AG.lambda_diag = double(g.adaptive.lambda_diag(:));         % [1/s] sliding-surface slope
AG.K_diag      = double(g.adaptive.K_diag(:));              % [N*m*s] linear feedback on s
AG.eta         = double(g.adaptive.eta(:));                 % [N*m] switching gain
AG.phi         = double(g.adaptive.phi(:));                 % [rad/s] boundary-layer thickness
AG.Gamma_diag  = double(g.adaptive.Gamma_diag(:));          % [kg*m^2*s^2] adaptation gain Gamma (diagonal)
if double(g.adaptive.theta_hat0_from_Jnom) > 0.5
    AG.theta_hat0 = SC.theta_nom;                           % [kg*m^2] start the estimate at the nominal inertia
else
    AG.theta_hat0 = double(g.adaptive.theta_hat0(:));       % [kg*m^2] user-given initial estimate
end
f = double(g.adaptive.proj_frac);                           % [-] projection-box half-width fraction
th = SC.theta_nom;                                          % [kg*m^2] projection-box centre
mdiag = mean(th(1:3));                                      % [kg*m^2] mean principal moment (scale for products)
AG.theta_min = [(1 - f)*th(1:3); th(4:6) - (abs(th(4:6)) + f*mdiag)]; % [kg*m^2] projection lower bound
AG.theta_max = [(1 + f)*th(1:3); th(4:6) + (abs(th(4:6)) + f*mdiag)]; % [kg*m^2] projection upper bound
AG.theta_hat0 = min(max(AG.theta_hat0, AG.theta_min), AG.theta_max); % [kg*m^2] clip: the initial estimate must lie inside the projection box
if isfield(g.adaptive, 'freeze_on_sat')
    AG.freeze_on_sat = double(g.adaptive.freeze_on_sat);   % [-] adaptation anti-windup flag
else
    AG.freeze_on_sat = 1;                                    % [-] default: freeze while saturated
end

BG = struct();
BG.Kp_diag = double(g.baseline.Kp_diag(:));                 % [N*m] PD proportional gain
BG.Kd_diag = double(g.baseline.Kd_diag(:));                 % [N*m*s] PD derivative gain

%% ===== Info =====
info = struct();
info.J_true_physical = phys;
info.q_r0 = q_r0;
info.w_r0 = w_r0;
info.J_unc_tries = tries;
if phys
    info.notes = sprintf('J_true physically valid (%d draw(s)).', tries);
else
    info.notes = 'WARNING: no physically valid J_true found; J_true = J_nom used.';
end
end

%% ===== Local functions =====
function [J_true, phys, tries] = localTrueInertia(J_nom, pct, mode, seed)
%LOCALTRUEINERTIA Draw the true inertia from the uncertainty settings.
%
% Inputs:
%   J_nom - double [3x3], nominal inertia                                [kg*m^2]
%   pct   - double [1], uncertainty magnitude                            [%]
%   mode  - double [1], 1 random signed per parameter, 2 uniform scale   [-]
%   seed  - double [1], RNG seed                                          [-]
% Outputs:
%   J_true - double [3x3], true inertia                                  [kg*m^2]
%   phys   - logical [1], true if J_true is physically valid             [-]
%   tries  - double [1], number of random draws used                     [-]

%% ===== No uncertainty / uniform scale =====
tries = 0;
if pct == 0
    J_true = J_nom;
    phys = localIsPhysical(J_true);
    return;
end
if mode > 1.5
    J_true = (1 + pct/100)*J_nom;
    phys = localIsPhysical(J_true);
    return;
end

%% ===== Seeded per-parameter perturbation (redraw until physical) =====
seedRNG(seed);
th = inertiaToTheta(J_nom);
phys = false;
J_true = J_nom;
while ~phys && tries < 1000
    tries = tries + 1;
    u = 2*rand(6,1) - 1;
    Jc = thetaToInertia(th.*(1 + u*pct/100));
    if localIsPhysical(Jc)
        J_true = Jc;
        phys = true;
    end
end
end

function tf = localIsPhysical(J)
%LOCALISPHYSICAL Positive definiteness and triangle inequalities of the principal moments.
%
% Inputs:
%   J  - double [3x3], symmetric inertia matrix                          [kg*m^2]
% Outputs:
%   tf - logical [1], true if J is a physically realisable inertia       [-]

e = sort(eig(0.5*(J + J.')));
tf = all(e > 0) && (e(1) + e(2) >= e(3));
end
