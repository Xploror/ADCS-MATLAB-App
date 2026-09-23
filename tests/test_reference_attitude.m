function test_reference_attitude()
%TEST_REFERENCE_ATTITUDE Checks of the reference-attitude generator (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

cfg = initDefaults();
[SC] = buildSimParams(cfg);

%% ===== Mode 1: LVLH hold rate =====
SC.mode = 1;                                             % [-] LVLH hold
for t = [0, 123.4, 2000]
    [q_r, w_r, wdot_r] = referenceAttitude(t, SC);
    assert(norm(w_r - [0; -SC.n; 0]) < 1e-6*SC.n, 'mode 1: w_r ~= [0; -n; 0]');
    assert(norm(wdot_r) < 1e-9, 'mode 1: wdot_r should vanish');
    assert(abs(norm(q_r) - 1) < 1e-12 && q_r(1) >= 0, 'mode 1: q_r not unit or q0 < 0');
    assert(max(max(abs(qToDCM(q_r) - referenceDCM(t, SC)))) < 1e-12, 'mode 1: q_r inconsistent with C_RN');
end

%% ===== Modes 1, 2 and 3: rate consistent with finite differences of C_RN =====
for mode = [1 2 3]
    SC.mode = mode;
    if mode == 3
        tolC = 1e-7;                                     % [1/s] mode 3: O(h^2 A w^3) truncation of the h = 0.5 s stencil
    else
        tolC = 1e-8;                                     % [1/s] modes 1-2: slowly varying reference
    end
    for t = [0, 50, 311.7, 900]
        [~, w_r, wdot_r] = referenceAttitude(t, SC);
        h = 1e-3;                                        % [s] central-difference step
        Cdot = (referenceDCM(t + h, SC) - referenceDCM(t - h, SC))/(2*h);
        Cpred = -skew3(w_r)*referenceDCM(t, SC);
        assert(max(max(abs(Cdot - Cpred))) < tolC, sprintf('mode %d: w_r inconsistent with C_RN', mode));
        [~, w_p] = referenceAttitude(t + 0.01, SC);
        [~, w_m] = referenceAttitude(t - 0.01, SC);
        assert(norm((w_p - w_m)/0.02 - wdot_r) < 1e-8, sprintf('mode %d: wdot_r inconsistent', mode));
    end
end

%% ===== Mode 2: starts pointing at the ISS =====
SC.mode = 2;                                             % [-] docking approach
C = referenceDCM(0, SC);
[~, ~, C_ON] = orbitGeometry(0, SC);
[R, y, z] = approachProfile(0, SC);
u_O = [R; -y; -z]/norm([R; -y; -z]);
assert(norm(C*C_ON.'*u_O - [1; 0; 0]) < 1e-12, 'mode 2: b1 is not the line of sight');

%% ===== Mode 3: scan stays within its amplitude about LVLH hold =====
for t = [0, 77.7, 420, 1111]
    SC.mode = 3;                                         % [-] attitude scan
    C3 = referenceDCM(t, SC);
    SC.mode = 1;                                         % [-] LVLH hold
    C1 = referenceDCM(t, SC);
    ang = qErrorAngleDeg(dcmToQ(C3*C1.'))*pi/180;        % [rad] scan rotation angle
    assert(ang <= norm(SC.scan_amp) + 1e-12, 'mode 3: scan exceeds its amplitude');
end

%% ===== Mode 0: constant =====
SC.mode = 0;                                             % [-] inertial hold
SC.q_inertial_ref = qFromAxisAngle([1; 2; 3], 0.7);
for t = [0, 100, 1000]
    [q_r, w_r, wdot_r] = referenceAttitude(t, SC);
    assert(norm(q_r - SC.q_inertial_ref) < 1e-15, 'mode 0: q_r not constant');
    assert(all(w_r == 0) && all(wdot_r == 0), 'mode 0: rates not zero');
end
end
