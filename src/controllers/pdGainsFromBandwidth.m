function [Kp_diag, Kd_diag] = pdGainsFromBandwidth(J_nom, wn, zeta)
%PDGAINSFROMBANDWIDTH Diagonal quaternion-PD gains from a bandwidth and damping ratio.
%
% Inputs:
%   J_nom - double [3x3], nominal inertia matrix                         [kg*m^2]
%   wn    - double [1], closed-loop natural frequency                    [rad/s]
%   zeta  - double [1], damping ratio                                    [-]
% Outputs:
%   Kp_diag - double [3x1], 2*diag(J)*wn^2 (q_ev ~ angle/2)              [N*m]
%   Kd_diag - double [3x1], 2*zeta*wn*diag(J)                            [N*m*s]

d = [J_nom(1,1); J_nom(2,2); J_nom(3,3)];
Kp_diag = 2*d*wn*wn;
Kd_diag = 2*zeta*wn*d;
end
