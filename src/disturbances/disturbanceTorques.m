function [tau_d, tau_gg, tau_aero, tau_srp, tau_mag] = disturbanceTorques(t, q, SC)
% Enabled and scaled environmental torques and their sum, body axes.
%
% Inputs:
%   t  - double [1], simulation time in secs
%   q  - double [4x1], true attitude quaternion B w.r.t. N
%   SC - struct, NOTE SC.dist_scale [1x4] (order GG, Aero, SRP, Mag)
% Outputs:
%   tau_d    - double [3x1], total disturbance torque in N*m
%   tau_gg   - double [3x1], gravity-gradient torque (enabled*scaled) in N*m
%   tau_aero - double [3x1], aerodynamic torque (enabled*scaled) in N*m
%   tau_srp  - double [3x1], SRP torque (enabled*scaled) in N*m
%   tau_mag  - double [3x1], magnetic torque (enabled*scaled) in N*m

%% ===== Orbit geometry =====
[r_hat_N, v_hat_N] = orbitGeometry(t, SC);

%% ===== Individual torques =====
g = SC.dist_enable.*SC.dist_scale; % effective disturbance gains
tau_gg = g(1)*gravityGradientTorque(q, r_hat_N, SC);
tau_aero = g(2)*aeroTorque(q, v_hat_N, SC);
tau_srp = g(3)*srpTorque(q, SC);
tau_mag = g(4)*magneticTorque(q, r_hat_N, SC);
%% ===== Sum =====
tau_d = tau_gg + tau_aero + tau_srp + tau_mag;
end
