function thdot = paramProjection(theta_hat, thdot_raw, th_min, th_max)
%PARAMPROJECTION Box projection of the parameter update (stop at the bounds).
%
% Inputs:
%   theta_hat - double [6x1], current estimate                           [kg*m^2]
%   thdot_raw - double [6x1], unprojected update                         [kg*m^2/s]
%   th_min    - double [6x1], lower bounds                               [kg*m^2]
%   th_max    - double [6x1], upper bounds                               [kg*m^2]
% Outputs:
%   thdot     - double [6x1], update with outward components at an
%               active bound set to zero                                 [kg*m^2/s]

thdot = thdot_raw;
for i = 1:numel(thdot_raw)
    if (theta_hat(i) >= th_max(i) && thdot_raw(i) > 0) || (theta_hat(i) <= th_min(i) && thdot_raw(i) < 0)
        thdot(i) = 0;
    end
end
end
