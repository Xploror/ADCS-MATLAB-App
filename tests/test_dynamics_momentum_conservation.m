function test_dynamics_momentum_conservation()
%TEST_DYNAMICS_MOMENTUM_CONSERVATION Torque-free invariants of the rigid-body model (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)
%
% Torque-free motion with constant wheel momentum h_w (zero wheel torque):
% inertial angular momentum C_BN'(J w + h_w) is conserved; with h_w = 0 the
% rotational kinetic energy is conserved as well. RK4, dt = 0.01 s, 200 s.

cfg = initDefaults();
SC = buildSimParams(cfg);
dt = 0.01;          % [s] RK4 step
T  = 200;           % [s] duration
nS = round(T/dt);

for caseId = 1:2
    %% ===== Case setup =====
    if caseId == 1
        h_w = [2; -1; 3];                      % [N*m*s] constant wheel momentum
    else
        h_w = zeros(3,1);
    end
    q = qFromAxisAngle([1; -2; 0.5], 0.8);
    w = [0.02; -0.03; 0.015];                  % [rad/s] initial body rate
    J = SC.J_true;
    H0 = qToDCM(q).'*(J*w + h_w);
    E0 = 0.5*(w.'*J*w);
    x = [q; w];
    f = @(x) localF(x, h_w, SC);

    %% ===== Integrate =====
    maxH = 0; maxE = 0;
    for k = 1:nS
        k1 = f(x); k2 = f(x + 0.5*dt*k1); k3 = f(x + 0.5*dt*k2); k4 = f(x + dt*k3);
        x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        if mod(k, 100) == 0
            qn = qNormalize(x(1:4));
            H = qToDCM(qn).'*(J*x(5:7) + h_w);
            maxH = max(maxH, norm(H - H0)/norm(H0));
            maxE = max(maxE, abs(0.5*(x(5:7).'*J*x(5:7)) - E0)/E0);
        end
    end

    %% ===== Checks =====
    assert(maxH < 1e-8, sprintf('case %d: inertial momentum drift %.3g', caseId, maxH));
    if caseId == 2
        assert(maxE < 1e-8, sprintf('kinetic energy drift %.3g', maxE));
    end
end
end

function xdot = localF(x, h_w, SC)
%LOCALF Torque-free state derivative [q_raw; w] with constant wheel momentum.
%
% Inputs:
%   x    - double [7x1], [q_raw; w]                                      [-, rad/s]
%   h_w  - double [3x1], wheel momentum                                  [N*m*s]
%   SC   - struct, sim constants                                         [mixed SI]
% Outputs:
%   xdot - double [7x1], derivative                                      [1/s, rad/s^2]

[qd, wd] = rigidBodyDerivatives(x(1:4), x(5:7), zeros(3,1), zeros(3,1), h_w, SC);
xdot = [qd; wd];
end
