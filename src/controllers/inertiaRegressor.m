function Y = inertiaRegressor(w, w_rv, w_rv_dot)
% Inertia Regressor for SMC (Slotine-Li regressor)
%
% Inputs:
%   w        - double [3x1], body rate w.r.t. N, B axes in rad/s
%   w_rv     - double [3x1], virtual reference rate, B axes in rad/s
%   w_rv_dot - double [3x1], derivative of w_rv, B axes in rad/s^2
% Outputs:
%   Y        - double [3x6], inertia regressor matrix

Y = inertiaLinearMap(w_rv_dot) + skew3(w_rv)*inertiaLinearMap(w);
end
