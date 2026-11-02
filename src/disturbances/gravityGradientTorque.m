function tau = gravityGradientTorque(q, r_hat_N, SC)
% Gravity-gradient torque on the true inertia, body axes.
%
% Inputs:
%   q       - double [4x1], attitude quaternion B w.r.t. N
%   r_hat_N - double [3x1], unit radial (zenith) direction in N
%   SC      - struct
% Outputs:
%   tau     - double [3x1], unscaled GG torque 3 n^2 r_B x (J r_B) in N*m

z_N = [0 sin(SC.incl) cos(SC.incl)]'; % Earth rot axis in 

r_B = qToDCM(q)*r_hat_N;
z_B = qToDCM(q)*z_N;
s = dot(r_B, z_B);

tauJ2 = -SC.mu*SC.J2*SC.R_E^2/(r_B^5)*(0.5*(105*s^2 - 15)*cross(r_B, SC.J_true*r_B) - 15*s*(cross(r_B, SC.J_true*z_B) + cross(z_B, SC.J_true*r_B)) + (3*cross(z_B, SC.J_true*z_B)));
tauJ4 = -SC.mu*SC.J4*SC.R_E^2/(r_B^7)*(0.125*(3465*s^4-1890*s^2+105)*cross(r_B, SC.J_true*r_B) + 0.5*(105*s-315*s^3)*(cross(r_B, SC.J_true*z_B) + cross(z_B, SC.J_true*r_B)) + (0.5*(105*s^2-15)*cross(z_B, SC.J_true*z_B)));
tau = 3*SC.n^2*cross(r_B, SC.J_true*r_B) + tauJ2 + tauJ4;
end
