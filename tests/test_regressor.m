function test_regressor()
%TEST_REGRESSOR Checks of the inertia regressor and parameter maps (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

seedRNG(3);

%% ===== Regressor identity and linear map =====
for k = 1:50
    theta = [1000 + 2000*rand(3,1); 300*randn(3,1)];
    J = thetaToInertia(theta);
    w = 0.1*randn(3,1); w_rv = 0.1*randn(3,1); w_rv_dot = 0.01*randn(3,1);
    Y = inertiaRegressor(w, w_rv, w_rv_dot);
    ref = J*w_rv_dot + cross(w_rv, J*w);
    assert(norm(Y*theta - ref) < 1e-10*max(1, norm(ref)), 'Y*theta ~= J*w_rv_dot + w_rv x (J w)');
    v = randn(3,1);
    assert(norm(inertiaLinearMap(v)*theta - J*v) < 1e-10*norm(J*v), 'L(v)*theta ~= J*v');
end

%% ===== theta <-> J round trip =====
cfg = initDefaults();
J = cfg.scenario.J_nom;
assert(isequal(thetaToInertia(inertiaToTheta(J)), J), 'thetaToInertia(inertiaToTheta(J)) ~= J');
theta = randn(6,1);
assert(isequal(inertiaToTheta(thetaToInertia(theta)), theta), 'inertiaToTheta(thetaToInertia(theta)) ~= theta');

%% ===== Projection =====
th = [1; 1; 1; 0; 0; 0];                                 % [kg*m^2] parameter vector to project
out = paramProjection(th, [1; -1; 1; -1; 1; -1], [0; 1; 0; -1; -1; -1], [1; 2; 2; 1; 1; 1]);
assert(isequal(out, [0; 0; 1; -1; 1; -1]), 'paramProjection wrong');
end
