function Y = inertiaRegressor(w, w_rv, w_rv_dot)
%INERTIAREGRESSOR Regressor with Y*theta = J*w_rv_dot + w_rv x (J*w).
%
% Inputs:
%   w        - double [3x1], body rate w.r.t. N, B axes                   [rad/s]
%   w_rv     - double [3x1], virtual reference rate, B axes               [rad/s]
%   w_rv_dot - double [3x1], derivative of w_rv, B axes                   [rad/s^2]
% Outputs:
%   Y        - double [3x6], regressor matrix                             [rad/s^2]

Y = inertiaLinearMap(w_rv_dot) + skew3(w_rv)*inertiaLinearMap(w);
end
