function [Kp_diag, Kd_diag] = pdGainsFromBandwidth(J_nom, wn, zeta)
% Diagonal quaternion-PD gains from a bandwidth and damping ratio.
%
% Inputs:
%   J_nom - double [3x3], nominal inertia matrix in kg*m^2
%   wn    - double [1], closed-loop natural frequency in rad/s
%   zeta  - double [1], damping ratio
% Outputs:
%   Kp_diag - double [3x1], 2*diag(J)*wn^2 (q_ev ~ angle/2) in N*m
%   Kd_diag - double [3x1], 2*zeta*wn*diag(J) in N*m*s

d = diag(J_nom);
Kp_diag = 2*d*wn^2;
Kd_diag = 2*zeta*wn*d;
end
