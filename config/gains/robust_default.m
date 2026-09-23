function g = robust_default(J_nom)
%ROBUST_DEFAULT Default gains of the robust boundary-layer SMC (controller 1).
%
% Inputs:
%   J_nom - double [3x3], nominal inertia matrix                         [kg*m^2]
% Outputs:
%   g     - struct, gain set with fields:
%           lambda_diag [3x1] [1/s], K_diag [3x1] [N*m*s],
%           eta [3x1] [N*m], phi [3x1] [rad/s]
%
% K = diag(J)/20 gives an s-dynamics time constant of ~20 s (K/J = 0.05 1/s),
% matched to the sliding-surface slope Lambda = 0.05 1/s (q_ev decays at Lambda/2).

d = [J_nom(1,1); J_nom(2,2); J_nom(3,3)];
g = struct();
g.lambda_diag = 0.05*[1; 1; 1];   % [1/s]    sliding-surface slope Lambda (diagonal)
g.K_diag      = d/20;             % [N*m*s]  linear feedback on s (diagonal)
g.eta         = 2e-3*[1; 1; 1];   % [N*m]    switching (robustness) gain
g.phi         = 1e-3*[1; 1; 1];   % [rad/s]  boundary-layer thickness
end
