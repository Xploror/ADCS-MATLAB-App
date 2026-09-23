function [tau_d, tau_gg, tau_aero, tau_srp, tau_mag] = disturbanceTorques(t, q, SC)
%DISTURBANCETORQUES Enabled and scaled environmental torques and their sum, body axes.
%
% Inputs:
%   t  - double [1], simulation time                                      [s]
%   q  - double [4x1], true attitude quaternion B w.r.t. N               [-]
%   SC - struct, uses SC.dist_enable [1x4] and SC.dist_scale [1x4]
%        (order GG, Aero, SRP, Mag) [-] plus each model's fields
% Outputs:
%   tau_d    - double [3x1], total disturbance torque                    [N*m]
%   tau_gg   - double [3x1], gravity-gradient torque (enabled*scaled)    [N*m]
%   tau_aero - double [3x1], aerodynamic torque (enabled*scaled)         [N*m]
%   tau_srp  - double [3x1], SRP torque (enabled*scaled)                 [N*m]
%   tau_mag  - double [3x1], magnetic torque (enabled*scaled)            [N*m]

%% ===== Orbit geometry =====
[r_hat_N, v_hat_N] = orbitGeometry(t, SC);

%% ===== Individual torques =====
tau_gg   = zeros(3,1);
tau_aero = zeros(3,1);
tau_srp  = zeros(3,1);
tau_mag  = zeros(3,1);
g = SC.dist_enable.*SC.dist_scale;                   % [1x4] effective gains
if g(1) ~= 0
    tau_gg = g(1)*gravityGradientTorque(q, r_hat_N, SC);
end
if g(2) ~= 0
    tau_aero = g(2)*aeroTorque(q, v_hat_N, SC);
end
if g(3) ~= 0
    tau_srp = g(3)*srpTorque(q, SC);
end
if g(4) ~= 0
    tau_mag = g(4)*magneticTorque(q, r_hat_N, SC);
end

%% ===== Sum =====
tau_d = tau_gg + tau_aero + tau_srp + tau_mag;
end
