function [tau_c, theta_hat_dot, s] = adaptiveControlLaw(q, w, q_r, w_r, wdot_r, theta_hat, SC, AG)
%ADAPTIVECONTROLLAW Parametric (inertia-estimating) adaptive sliding-mode control (controller 2).
%
% Inputs:
%   q         - double [4x1], measured attitude quaternion B w.r.t. N    [-]
%   w         - double [3x1], measured body rate, B axes                 [rad/s]
%   q_r       - double [4x1], reference quaternion                       [-]
%   w_r       - double [3x1], reference rate, R axes                     [rad/s]
%   wdot_r    - double [3x1], reference rate derivative, R axes          [rad/s^2]
%   theta_hat - double [6x1], current inertia-parameter estimate         [kg*m^2]
%   SC        - struct, uses SC.shortest_path [-]
%   AG        - struct, lambda_diag [1/s], K_diag [N*m*s], eta [N*m], phi [rad/s],
%               Gamma_diag [6x1] [kg*m^2*s^2], theta_min/theta_max [6x1] [kg*m^2]
% Outputs:
%   tau_c         - double [3x1], Y*theta_hat - K s - eta.*sat(s./phi)   [N*m]
%   theta_hat_dot - double [6x1], projected update of -Gamma*Y'*s        [kg*m^2/s]
%   s             - double [3x1], sliding variable                       [rad/s]

[~, ~, ~, ~, w_rv, w_rv_dot, s] = attitudeErrors(q, w, q_r, w_r, wdot_r, AG.lambda_diag, SC.shortest_path);
Y = inertiaRegressor(w, w_rv, w_rv_dot);
tau_c = Y*theta_hat - AG.K_diag.*s - AG.eta.*satVec(s./AG.phi);
thdot_raw = -AG.Gamma_diag.*(Y.'*s);
theta_hat_dot = paramProjection(theta_hat, thdot_raw, AG.theta_min, AG.theta_max);
end
