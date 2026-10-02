function [tau_c, s] = robustControlLaw(q, w, q_r, w_r, wdot_r, SC, RG)
% Boundary-layer sliding-mode control.
%
% Inputs:
%   q      - double [4x1], measured attitude quaternion B w.r.t. N
%   w      - double [3x1], measured body rate, B axes in rad/s
%   q_r    - double [4x1], reference quaternion
%   w_r    - double [3x1], reference rate, R axes in rad/s
%   wdot_r - double [3x1], reference rate derivative, R axes in rad/s^2
%   SC     - struct
%   RG     - struct, Robust Gain properties
% Outputs:
%   tau_c  - double [3x1], control torque before wheel-gyro compensation
%            and clamping in N*m
%   s      - double [3x1], sliding variable in rad/s

[~, ~, ~, ~, w_rv, w_rv_dot, s] = attitudeErrors(q, w, q_r, w_r, wdot_r, RG.lambda_diag, SC.shortest_path);
Y = inertiaRegressor(w, w_rv, w_rv_dot);
tau_c = Y*SC.theta_nom - RG.K_diag.*s - RG.eta.*satVec(s./RG.phi);
end
