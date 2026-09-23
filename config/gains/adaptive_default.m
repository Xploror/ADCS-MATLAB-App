function g = adaptive_default(J_nom)
%ADAPTIVE_DEFAULT Default gains of the parametric adaptive SMC (controller 2).
%
% Inputs:
%   J_nom - double [3x3], nominal inertia matrix                         [kg*m^2]
% Outputs:
%   g     - struct, gain set with fields:
%           lambda_diag [3x1] [1/s], K_diag [3x1] [N*m*s], eta [3x1] [N*m],
%           phi [3x1] [rad/s] (shared with robust_default),
%           Gamma_diag [6x1] [kg*m^2*s^2], theta_hat0_from_Jnom [1] [-],
%           theta_hat0 [6x1] [kg*m^2], proj_frac [1] [-], freeze_on_sat [1] [-]

r = robust_default(J_nom);
g = struct();
g.lambda_diag = r.lambda_diag;            % [1/s]         sliding-surface slope (same as robust)
g.K_diag      = r.K_diag;                 % [N*m*s]       linear feedback on s (same as robust)
g.eta         = r.eta;                    % [N*m]         switching gain (same as robust)
g.phi         = r.phi;                    % [rad/s]       boundary layer (same as robust)
g.Gamma_diag  = 2e7*ones(6,1);            % [kg*m^2*s^2]  adaptation gain Gamma (diagonal)
g.theta_hat0_from_Jnom = 1;               % [-]           1 = initial estimate taken from J_nom
g.theta_hat0  = inertiaToTheta(J_nom);    % [kg*m^2]      initial estimate if flag == 0
g.proj_frac   = 0.6;                      % [-]           projection-box half-width fraction
g.freeze_on_sat = 1;                      % [-]           1 = freeze adaptation while the actuator is saturated
end
