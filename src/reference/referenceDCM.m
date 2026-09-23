function C_RN = referenceDCM(t, SC)
%REFERENCEDCM Reference-frame DCM C_RN for the selected target mode.
%
% Inputs:
%   t  - double [1], simulation time                                      [s]
%   SC - struct, uses SC.mode (0 inertial, 1 LVLH hold, 2 docking approach,
%        3 LVLH-relative attitude scan) [-], SC.q_inertial_ref [4x1] [-],
%        orbit and approach fields (see approachProfile), and for mode 3
%        SC.scan_amp [3x1] [rad] and SC.scan_freq [3x1] [rad/s]
% Outputs:
%   C_RN - double [3x3], DCM from N to the reference frame R             [-]

%% ===== Inertial hold =====
if SC.mode < 0.5
    C_RN = qToDCM(qNormalize(SC.q_inertial_ref));
    return;
end

%% ===== Docking-axis direction in O =====
[~, ~, C_ON] = orbitGeometry(t, SC);
u = [1; 0; 0];                         % mode 1: +V-bar
if SC.mode > 1.5 && SC.mode < 2.5
    [R, y, z] = approachProfile(t, SC);
    u = [R; -y; -z];                   % line of sight chaser -> ISS
    u = u/sqrt(u.'*u);
end

%% ===== Build C_RO = [b1'; b2'; b3'] =====
b1 = u;
b2 = cross([0; 0; 1], b1);
nb2 = sqrt(b2.'*b2);
if nb2 > 1e-12
    b2 = b2/nb2;
else
    b2 = [0; 1; 0];                    % degenerate: LOS along O3
end
b3 = cross(b1, b2);
C_RO = [b1.'; b2.'; b3.'];
C_RN = C_RO*C_ON;

%% ===== Mode 3: sinusoidal scan about the LVLH-hold attitude =====
% Rotation vector phi_i(t) = A_i sin(w_i t) with incommensurate w_i gives a
% persistently exciting reference for the inertia-parameter estimator.
if SC.mode > 2.5
    phi = SC.scan_amp.*sin(SC.scan_freq*t);   % [rad] scan rotation vector
    C_RN = qToDCM(qFromRotVec(phi))*C_RN;
end
end
