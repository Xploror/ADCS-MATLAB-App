function [tau_c, s] = robustControlLaw(q, w, q_r, w_r, wdot_r, SC, RG)
%ROBUSTCONTROLLAW Boundary-layer sliding-mode control with fixed nominal inertia (controller 1).
%
% Inputs:
%   q      - double [4x1], measured attitude quaternion B w.r.t. N       [-]
%   w      - double [3x1], measured body rate, B axes                    [rad/s]
%   q_r    - double [4x1], reference quaternion                          [-]
%   w_r    - double [3x1], reference rate, R axes                        [rad/s]
%   wdot_r - double [3x1], reference rate derivative, R axes             [rad/s^2]
%   SC     - struct, uses SC.theta_nom [6x1] [kg*m^2], SC.shortest_path [-]
%   RG     - struct, lambda_diag [1/s], K_diag [N*m*s], eta [N*m], phi [rad/s] (3x1 each)
% Outputs:
%   tau_c  - double [3x1], control torque before wheel-gyro compensation
%            and clamping: Y*theta_nom - K s - eta.*sat(s./phi)          [N*m]
%   s      - double [3x1], sliding variable                              [rad/s]

[~, ~, ~, ~, w_rv, w_rv_dot, s] = attitudeErrors(q, w, q_r, w_r, wdot_r, RG.lambda_diag, SC.shortest_path);
Y = inertiaRegressor(w, w_rv, w_rv_dot);
tau_c = Y*SC.theta_nom - RG.K_diag.*s - RG.eta.*satVec(s./RG.phi);
end
