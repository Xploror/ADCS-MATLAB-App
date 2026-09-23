function test_quaternion_utils()
%TEST_QUATERNION_UTILS Unit tests of the quaternion/DCM utilities (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

seedRNG(1);
tol = 1e-12;                                             % [-] numerical comparison tolerance

%% ===== qMult composition and conjugate =====
for k = 1:50
    qa = qNormalize(randn(4,1));
    qb = qNormalize(randn(4,1));
    Cab = qToDCM(qMult(qa, qb));
    assert(max(max(abs(Cab - qToDCM(qa)*qToDCM(qb)))) < tol, 'qMult does not compose DCMs');
    assert(max(max(abs(qToDCM(qConj(qa)) - qToDCM(qa).'))) < tol, 'qConj is not the transpose');
    C = qToDCM(qa);
    assert(max(max(abs(C*C.' - eye(3)))) < tol && abs(det(C) - 1) < tol, 'qToDCM is not a rotation');
end

%% ===== dcmToQ round trip (random and near-180 deg) =====
for k = 1:50
    q = qNormalize(randn(4,1));
    if q(1) < 0, q = -q; end
    q2 = dcmToQ(qToDCM(q));
    assert(q2(1) >= 0, 'dcmToQ must return q0 >= 0');
    assert(norm(q2 - q) < 1e-12, 'dcmToQ round trip failed');
end
angles = [pi, pi - 1e-9, pi - 1e-6, pi - 1e-3, 1e-9, 0]; % [rad] edge-case rotation angles near pi and 0
for a = angles
    for k = 1:10
        e = randn(3,1); e = e/norm(e);
        q = qFromAxisAngle(e, a);
        C = qToDCM(q);
        q2 = dcmToQ(C);
        assert(max(max(abs(qToDCM(q2) - C))) < 1e-12, 'dcmToQ DCM mismatch near 0/180 deg');
        assert(abs(abs(q2.'*q) - 1) < 1e-12, 'dcmToQ quaternion mismatch near 0/180 deg');
    end
end

%% ===== Axis-angle formula =====
for k = 1:20
    e = randn(3,1); phi = 2*pi*rand - pi;
    q = qFromAxisAngle(3*e, phi);           % non-unit axis must be normalised
    eh = e/norm(e);
    Cf = cos(phi)*eye(3) + (1 - cos(phi))*(eh*eh.') - sin(phi)*skew3(eh);
    assert(max(max(abs(qToDCM(q) - Cf))) < tol, 'axis-angle DCM formula mismatch');
    qr = qFromRotVec(phi*eh);
    assert(max(max(abs(qToDCM(qr) - Cf))) < tol, 'qFromRotVec mismatch');
end
assert(isequal(qFromAxisAngle([0;0;0], 1), [1;0;0;0]), 'zero axis must give identity');
assert(abs(qErrorAngleDeg(qFromAxisAngle([0;0;1], 0.3)) - 0.3*180/pi) < 1e-9, 'qErrorAngleDeg wrong');
assert(abs(qErrorAngleDeg(-qFromAxisAngle([0;0;1], 0.3)) - 0.3*180/pi) < 1e-9, 'qErrorAngleDeg sign dependence');

%% ===== skew3 and kinematics =====
a = randn(3,1); b = randn(3,1);
assert(norm(skew3(a)*b - cross(a, b)) < tol, 'skew3 wrong');
for k = 1:10
    q = qNormalize(randn(4,1));
    w = 0.2*randn(3,1);
    h = 1e-6;                                            % [s] central-difference step
    qp = qNormalize(q + 0.5*h*qKinMatrix(q)*w);
    qm = qNormalize(q - 0.5*h*qKinMatrix(q)*w);
    Cdot = (qToDCM(qp) - qToDCM(qm))/(2*h);
    Cref = -skew3(w)*qToDCM(q);
    assert(max(max(abs(Cdot - Cref))) < 1e-7, 'qKinMatrix inconsistent with Cdot = -[w x] C');
end
assert(isequal(satVec([-3; 0.2; 5]), [-1; 0.2; 1]), 'satVec wrong');
end
