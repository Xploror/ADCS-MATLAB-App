function V = buildVisualTrajectory(res, opts)
% Geometry of the 3D playback (positions, rotations, line of sight, torque arrows) for one run.
%
% Inputs:
%   res  - struct, one result of runSimulation; uses t, q, q_r,
%          tau_rw, sat_flags, att_err_deg, ctrl_id, ctrl_name and res.SC        [mixed]
%   opts - struct (optional), display options; missing fields take defaults:
%          mag_k        - double [1], attitude-error display magnification (1 = true) [-]
%          hold_range_m - double [1], hold-point range for modes 0, 1, 3 ([] = SC.R0) [m]
%          C_SO         - double [3x3], ISS attitude w.r.t. LVLH (eye = +XVV/+ZLV) [-]
%          arrow_len_m  - double [1], display length L of a full-scale torque arrow [m]
% Outputs:
%   V    - visual struct, everything the renderer needs:
%          t [Nx1] [s], mode [-], ctrl_id [-], ctrl_name (char), mag_k [-],
%          R_E [m], R_orb [m], n [rad/s], incl [rad],
%          C_EN [3x3] display-ECI <- N rotation (RAAN = 0) [-],
%          r_I_N, r_C_N [Nx3] ISS and chaser positions in N [m],
%          r_I_E, r_C_E [Nx3] the same in display ECI [m],
%          orbit_E [361x3] full ISS orbit circle in display ECI [m],
%          sun_E [1x3] unit Sun direction in display ECI [-],
%          rho_O [Nx3] chaser position w.r.t. the ISS in LVLH (O) [m], range_m [Nx1] [m],
%          M_BO, M_BO_disp, M_RO [3x3xN] body / displayed body / reference axes in O (expressed in columns) [-],
%          M_SO [3x3] ISS body axes in O (expressed in columns) [-],
%          u_O [Nx3] unit line of sight chaser -> ISS in O [-],
%          e_point_deg [Nx1] angle between the true b1 and u_O [deg],
%          att_err_deg [Nx1] true attitude error from res [deg],
%          A_O [3x3xN] torque arrows in O, column j = arrow of wheel axis j [m],
%          tau_frac [Nx3] tau_rw / tau_max [-], sat_tau, sat_h [Nx3] logical flags [-],
%          hold_point (logical), pos_note, point_note (char) labels for the display.
%
% Conventions: q is scalar-first, qToDCM(q) = C_BN so that v_B = C_BN v_N;
% C_ON is the LVLH DCM of orbitGeometry; rho_O = [-R; y; z] in mode 2.
% NOTE: the model has no translational dynamics.

%% ===== Options =====
if nargin < 2 || isempty(opts)
    opts = struct();
end
mag_k = getOpt(opts, 'mag_k', 1);                    % [-]  display magnification of the attitude error
hold_range = getOpt(opts, 'hold_range_m', []);       % [m]  hold-point range (empty = SC.R0)
C_SO = getOpt(opts, 'C_SO', eye(3));                 % [-]  ISS attitude relative to LVLH
L = getOpt(opts, 'arrow_len_m', 3);                  % [m]  full-scale torque-arrow length

%% ===== Run data =====
SC = res.SC;
t = res.t(:);                                        % [s]  sample times
N = numel(t);
if isempty(hold_range)
    hold_range = SC.R0;
end

V = struct();
V.t = t;
V.mode = round(SC.mode);
V.ctrl_id = res.ctrl_id;
V.ctrl_name = res.ctrl_name;
V.mag_k = mag_k;
V.R_E = SC.R_E;
V.R_orb = SC.R_orb;
V.n = SC.n;
V.incl = SC.incl;

%% ===== Display ECI rotation =====
ci = cos(SC.incl);
si = sin(SC.incl);
V.C_EN = [1 0 0; 0 ci -si; 0 si ci];                % maps the Earth rotation axis [0; sin i; cos i] in N-frame to [0; 0; 1] in ECI-frame
th = linspace(0, 2*pi, 361)';                       % [rad] full orbit
V.orbit_E = (V.C_EN*(SC.R_orb*[cos(th), sin(th), zeros(361,1)])')';
V.sun_E = (V.C_EN*SC.sun_N(:))';

%% ===== Initialization =====
V.r_I_N = zeros(N,3);
V.r_C_N = zeros(N,3);
V.rho_O = zeros(N,3);
V.range_m = zeros(N,1);
V.M_BO = zeros(3,3,N);
V.M_BO_disp = zeros(3,3,N);
V.M_RO = zeros(3,3,N);
V.M_SO = C_SO.';
V.u_O = zeros(N,3);
V.e_point_deg = zeros(N,1);
V.A_O = zeros(3,3,N);
V.tau_frac = res.tau_rw/SC.tau_max;
V.sat_tau = res.sat_flags(:,1:3) > 0.5;
V.sat_h = res.sat_flags(:,4:6) > 0.5;
V.att_err_deg = res.att_err_deg(:);
V.hold_point = (V.mode ~= 2);

%% ===== Per-sample geometry =====
for m = 1:N
    [~, ~, C_ON] = orbitGeometry(t(m), SC);
    th = SC.n*t(m);
    r_I = SC.R_orb*[cos(th); sin(th); 0];           % [m] ISS position in N
    if V.mode == 2
        [R, y, z] = approachProfile(t(m), SC);
        rho = [-R; y; z];                            % [m] approach profile in O
    else
        rho = [-hold_range; 0; 0];                   % [m] illustrative hold point on -V-bar
    end
    V.r_I_N(m,:) = r_I';
    V.r_C_N(m,:) = (r_I + C_ON'*rho)';
    V.rho_O(m,:) = rho';
    V.range_m(m) = norm(rho);

    % attitudes (true, reference, displayed)
    C_BN = qToDCM(res.q(m,:)');
    C_RN = qToDCM(res.q_r(m,:)');
    V.M_BO(:,:,m) = C_ON*C_BN';
    V.M_RO(:,:,m) = C_ON*C_RN';
    V.M_BO_disp(:,:,m) = C_ON*magnifiedAttitude(res.q(m,:).', res.q_r(m,:).', C_BN, C_RN, mag_k).';

    % line of sight and pointing error (always the true attitude)
    u = -rho/norm(rho);
    b1 = V.M_BO(:,1,m);
    V.u_O(m,:) = u';
    V.e_point_deg(m) = atan2(norm(cross(u, b1)), u'*b1)*180/pi;

    % torque arrows on the displayed body axes
    V.A_O(:,:,m) = V.M_BO_disp(:,:,m)*diag(L*V.tau_frac(m,:));
end
V.r_I_E = (V.C_EN*V.r_I_N')';
V.r_C_E = (V.C_EN*V.r_C_N')';

%% ===== Labels =====
if V.mode == 2
    V.pos_note = 'chaser position: analytic approach profile';
else
    V.pos_note = sprintf('chaser position: illustrative hold point %.0f m behind the ISS', hold_range);
end
switch V.mode
    case 0
        V.point_note = 'inertial hold: line of sight has no meaning';
    case 3
        V.point_note = 'pointing error includes the scan';
    otherwise
        V.point_note = '';
end
end

%% ===== Local functions =====

function C = magnifiedAttitude(q, q_r, C_BN, C_RN, k)
% Displayed C_BN with the attitude error about the reference scaled by k.
%
% Inputs:
%   q, q_r - double [4x1], true and reference attitude quaternions (scalar-first) [-]
%   C_BN   - double [3x3], true attitude DCM                                       [-]
%   C_RN   - double [3x3], reference attitude DCM                                  [-]
%   k      - double [1], magnification factor                                      [-]
% Outputs:
%   C      - double [3x3], displayed attitude DCM C(q_e^k) C_RN (C_BN if k = 1)    [-]

if k == 1
    C = C_BN;
    return;
end
q_e = qMult(q, qConj(q_r));
if q_e(1) < 0
    q_e = -q_e;                                      % shortest path
end
nv = norm(q_e(2:4));
if nv < eps
    C = C_RN;                                        % zero error: nothing to magnify
    return;
end
phi = 2*atan2(nv, q_e(1));                           % [rad] error angle
e = q_e(2:4)/nv;                                     % [-]   error axis
phi_k = min(k*phi, pi);                              % [rad] clamped at 180 deg
q_k = [cos(phi_k/2); e*sin(phi_k/2)];
C = qToDCM(q_k)*C_RN;
end

function v = getOpt(s, name, default)
% Field of an options struct, or the default when it is missing or empty.
%
% Inputs:
%   s       - struct, options                                [-]
%   name    - char, field name                               [-]
%   default - any, value used when the field is absent        [-]
% Outputs:
%   v       - any, the option value                           [-]

if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
