function test_visual_geometry()
% Checks of buildVisualTrajectory on every preset scenario and all four target modes (errors on failure).
%
% Checks: orbit and relative positions, display-ECI
% rotation, orthonormal rotations, error magnification, pointing error
% recomputed independently, torque arrows and saturation flags.

%% ===== Cases: every scenario (short runs) plus an inertial-hold case =====
list = listScenarios();                               % Kx2 cell {function, display name}
K = size(list, 1);
cases = cell(1, K + 1);
for k = 1:K
    cases{k} = feval(list{k, 1});
end
cfg0 = scn_nominal();
cfg0.scenario.target_mode = 0;                        % inertial hold (no preset uses it)
cases{K + 1} = cfg0;

for c = 1:numel(cases)
    cfg = cases{c};
    cfg.scenario.t_final_s = 20;                      % [s] 20 sec short run
    ctrl = mod(c - 1, 3) + 1;                         % cycle through the three controllers
    [SC, RG, AG, BG] = buildSimParams(cfg);
    res = simulateADCS_ref(SC, RG, AG, BG, ctrl);
    V = buildVisualTrajectory(res);
    V10 = buildVisualTrajectory(res, struct('mag_k', 10));
    N = numel(V.t);
    tag = sprintf('case %d (mode %d)', c, V.mode);

    %% ===== T1: positions and display rotation =====
    assert(max(abs(sqrt(sum(V.r_I_N.^2, 2)) - SC.R_orb)) < 1e-12*SC.R_orb, [tag ': |r_I| ~= R_orb']);
    assert(max(abs(sqrt(sum((V.r_C_N - V.r_I_N).^2, 2)) - V.range_m)) < 1e-6, [tag ': |r_C - r_I| ~= |rho|']);
    for m = [1, round(N/2), N]
        if V.mode == 2
            [R, y, z] = approachProfile(V.t(m), SC);
            rho = [-R; y; z];
        else
            rho = [-SC.R0; 0; 0];
        end
        assert(norm(V.rho_O(m,:)' - rho) < 1e-9, [tag ': rho_O differs from the approach profile / hold point']);
        [~, ~, C_ON] = orbitGeometry(V.t(m), SC);
        assert(norm(V.r_C_N(m,:)' - V.r_I_N(m,:)' - C_ON'*rho) < 1e-6, [tag ': r_C ~= r_I + C_ON''*rho']);
    end
    zE = V.C_EN*[0; sin(SC.incl); cos(SC.incl)];
    assert(norm(zE - [0; 0; 1]) < 1e-12, [tag ': C_EN does not map the spin axis to z']);
    assert(V.hold_point == (V.mode ~= 2), [tag ': hold_point flag']);

    %% ===== T2: rotations are proper and orthonormal =====
    for m = [1, round(N/2), N]
        Ms = {V.M_BO(:,:,m), V.M_BO_disp(:,:,m), V.M_RO(:,:,m), V10.M_BO_disp(:,:,m), V.M_SO};
        for j = 1:numel(Ms)
            M = Ms{j};
            assert(norm(M'*M - eye(3)) < 1e-12 && abs(det(M) - 1) < 1e-12, [tag ': rotation not orthonormal']);
        end
        C_BN = qToDCM(res.q(m,:)');
        [~, ~, C_ON] = orbitGeometry(V.t(m), SC);
        assert(norm(V.M_BO(:,:,m) - C_ON*C_BN') < 1e-12, [tag ': M_BO ~= C_ON*C_BN''']);
    end

    %% ===== T3: magnification =====
    assert(max(abs(V.M_BO_disp(:) - V.M_BO(:))) < 1e-12, [tag ': k = 1 must give the true attitude']);
    for m = [1, round(N/2), N]
        phi1 = rotAngle(V.M_BO(:,:,m)'*V.M_RO(:,:,m));
        phi10 = rotAngle(V10.M_BO_disp(:,:,m)'*V10.M_RO(:,:,m));
        if phi1 < 18*pi/180
            assert(abs(phi10 - 10*phi1) < 1e-9 + 1e-9*phi1, [tag ': k = 10 does not scale the error angle']);
        else
            assert(phi10 <= pi + 1e-12, [tag ': magnified angle exceeds 180 deg']);
        end
    end

    %% ===== T4: pointing error recomputed independently =====
    for m = [1, round(N/2), N]
        u = -V.rho_O(m,:)'/norm(V.rho_O(m,:));
        [~, ~, C_ON] = orbitGeometry(V.t(m), SC);
        b1 = C_ON*qToDCM(res.q(m,:)')'*[1; 0; 0];
        e_ref = 2*asin(min(1, norm(u - b1)/2))*180/pi;  % [deg] chord form, well conditioned at small angles
        assert(abs(V.e_point_deg(m) - e_ref) < 1e-9, [tag ': pointing error mismatch']);
    end

    %% ===== T4b: line of sight equals the reference b1 (independent of the sign convention in V) =====
    if V.mode == 1 || V.mode == 2
        for m = [1, round(N/2), N]
            assert(norm(V.M_RO(:,1,m) - V.u_O(m,:)') < 1e-9, [tag ': line of sight ~= reference b1']);
        end
    end

    %% ===== T5: torque arrows and flags =====
    L = 3;                                            % [m] default full-scale arrow length
    for m = [1, round(N/2), N]
        A = V.M_BO_disp(:,:,m)*diag(L*res.tau_rw(m,:)/SC.tau_max);
        assert(norm(V.A_O(:,:,m) - A) < 1e-12, [tag ': torque arrows']);
    end
    assert(isequal(V.sat_tau, res.sat_flags(:,1:3) > 0.5) && isequal(V.sat_h, res.sat_flags(:,4:6) > 0.5), ...
           [tag ': saturation flags']);
    assert(all(isfinite(V10.M_BO_disp(:))), [tag ': NaN in magnified attitude']);
end

%% ===== Zero-error sample: magnification must not produce NaN =====
cfg = scn_nominal();
cfg.scenario.t_final_s = 2;                           % [s]
[SC, RG, AG, BG] = buildSimParams(cfg);
res = simulateADCS_ref(SC, RG, AG, BG, 1);
res.q(1,:) = res.q_r(1,:);                            % exact zero error at the first sample
V = buildVisualTrajectory(res, struct('mag_k', 100));
assert(all(isfinite(V.M_BO_disp(:))), 'zero error: NaN in magnified attitude');
assert(norm(V.M_BO_disp(:,:,1) - V.M_RO(:,:,1)) < 1e-12, 'zero error: displayed attitude ~= reference');

%% ===== Options: hold range and arrow length =====
cfg1 = scn_large_attitude_error();
cfg1.scenario.t_final_s = 2;                          % [s]
[SC, RG, AG, BG] = buildSimParams(cfg1);
res = simulateADCS_ref(SC, RG, AG, BG, 3);
V = buildVisualTrajectory(res, struct('hold_range_m', 250, 'arrow_len_m', 7));
assert(norm(V.rho_O(end,:) - [-250 0 0]) < 1e-12, 'hold_range_m option ignored');
A = V.M_BO_disp(:,:,end)*diag(7*res.tau_rw(end,:)/SC.tau_max);
assert(norm(V.A_O(:,:,end) - A) < 1e-12, 'arrow_len_m option ignored');

%% ===== Labels =====
cfg.scenario.target_mode = 3;                         % attitude scan
[SC, RG, AG, BG] = buildSimParams(cfg);
V = buildVisualTrajectory(simulateADCS_ref(SC, RG, AG, BG, 2));
assert(~isempty(strfind(V.point_note, 'scan')), 'mode 3: missing "includes the scan" note');
end

%% ===== Local functions =====
function phi = rotAngle(R)
% Rotation angle of a proper rotation matrix, well conditioned at small angles.
%
% Inputs:
%   R   - double [3x3], rotation matrix
% Outputs:
%   phi - double [1], rotation angle       [rad]

s = norm([R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)])/2;   % sin(phi)
c = (trace(R) - 1)/2;                                              % cos(phi)
phi = atan2(s, c);
end
