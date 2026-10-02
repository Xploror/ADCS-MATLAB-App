function thdot = paramProjection(theta_hat, thdot, th_min, th_max)
% Projection of the parameter update. Currently only supports box
% projection, ellipsoidal projection (ongoing)
%
% Inputs:
%   theta_hat - double [6x1], current estimate in kg*m^2
%   thdot - double [6x1], unprojected update in kg*m^2/s
%   th_min    - double [6x1], lower bounds in kg*m^2
%   th_max    - double [6x1], upper bounds in kg*m^2
% Outputs:
%   thdot     - double [6x1], update with outward components at an
%               active bound set to zero

for i = 1:numel(thdot)
    if (theta_hat(i) >= th_max(i) && thdot_raw(i) > 0) || (theta_hat(i) <= th_min(i) && thdot_raw(i) < 0)
        thdot(i) = 0;
    end
end
end
