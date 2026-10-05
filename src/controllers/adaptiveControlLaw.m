function [tau_c, theta_hat_dot, s] = adaptiveControlLaw(q, w, q_r, w_r, wdot_r, theta_hat, SC, AG)
% Parametric adaptive sliding-mode control.
%
% Inputs:
%   q         - double [4x1], measured attitude quaternion B w.r.t. N
%   w         - double [3x1], measured body rate, B axes in rad/s
%   q_r       - double [4x1], reference quaternion
%   w_r       - double [3x1], reference rate, R axes in rad/s
%   wdot_r    - double [3x1], reference rate derivative, R axes in rad/s^2
%   theta_hat - double [6x1], current inertia-parameter estimate in kg*m^2
%   SC        - struct
%   AG        - struct, Adaptive Gain properties
% Outputs:
%   tau_c         - double [3x1], control torque before wheel-gyro compensation
%            and clamping in N*m
%   theta_hat_dot - double [6x1], projected update of -Gamma*Y'*s in kg*m^2/s
%   s             - double [3x1], sliding variable in rad/s

[~, ~, ~, ~, w_rv, w_rv_dot, s] = attitudeErrors(q, w, q_r, w_r, wdot_r, AG.lambda_diag, SC.shortest_path);
Y = inertiaRegressor(w, w_rv, w_rv_dot);
tau_c = Y*theta_hat - AG.K_diag.*s - AG.eta.*satVec(s./AG.phi);
thdot_raw = -AG.Gamma_diag.*(Y'*s);
theta_hat_dot = paramProjection(theta_hat, thdot_raw, AG.theta_min, AG.theta_max);
end
