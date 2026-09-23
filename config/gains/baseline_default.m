function g = baseline_default(J_nom)
%BASELINE_DEFAULT Default gains of the quaternion PD benchmark (controller 3).
%
% Inputs:
%   J_nom - double [3x3], nominal inertia matrix                         [kg*m^2]
% Outputs:
%   g     - struct, gain set with fields Kp_diag [3x1] [N*m], Kd_diag [3x1] [N*m*s]

wn   = 0.03;   % [rad/s] closed-loop natural frequency
zeta = 0.9;    % [-]     damping ratio
[Kp, Kd] = pdGainsFromBandwidth(J_nom, wn, zeta);
g = struct();
g.Kp_diag = Kp;   % [N*m]    proportional gain on q_ev (diagonal)
g.Kd_diag = Kd;   % [N*m*s]  derivative gain on w_e (diagonal)
end
