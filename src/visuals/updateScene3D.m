function updateScene3D(H, V, m, opts)
% Move the 3D playback to stored sample m (transforms, arrows, lines, HUD, camera).
%
% Inputs:
%   H    - struct, handles from drawScene3D                                  [-]
%   V    - struct, output of buildVisualTrajectory                           [mixed]
%   m    - double [1], sample index into V.t                                 [-]
%   opts - struct (optional); missing fields take defaults:
%          camera - char, 'iss' | 'chaser' | 'docking' (proximity camera)      [-]
%          draw   - logical [1], call drawnow limitrate at the end           [-]
% Outputs:
%   Graphics objects H is just updated
%
% Only data and transform matrices change here; nothing is created, so a
% frame costs a few property sets. Growing lines use the decimated
% samples H.line_idx (plus the current one). The HUD always shows true values, even
% when the attitude error is magnified for display.

%% ===== Options =====
if nargin < 4 || isempty(opts)
    opts = struct();
end
cam = 'iss';
if isfield(opts, 'camera') && ~isempty(opts.camera)
    cam = opts.camera;
end
do_draw = ~isfield(opts, 'draw') || opts.draw;
km = 1e-3;                                           % [km/m]

%% ===== Chaser, reference ghost =====
rho = V.rho_O(m,:)';                                % [m] chaser position in O
M = V.M_BO_disp(:,:,m);                              % [-] displayed body axes in O
H.chsTf.Matrix = [M rho; 0 0 0 1];
H.refTf.Matrix = [V.M_RO(:,:,m) rho; 0 0 0 1];

%% ===== Wheel-torque arrows (body axes; the transform carries them into O) =====
L = H.arrow_len_m;
f = V.tau_frac(m,:);                                 % [-] tau_rw / tau_max
for j = 1:3
    d = zeros(3,1);
    d(j) = L*f(j);
    set(H.arrow(j), 'UData', d(1), 'VData', d(2), 'WData', d(3));
    if V.sat_tau(m,j)
        H.arrow(j).Color = [1 0 0];
    else
        H.arrow(j).Color = H.arrowColor(j,:);
    end
end
set(H.arrow(4), 'UData', L*f(1), 'VData', L*f(2), 'WData', L*f(3));

%% ===== Line of sight, docking axis, trace =====
set(H.los, 'XData', [rho(1) 0], 'YData', [rho(2) 0], 'ZData', [rho(3) 0]);
tip = rho + M*H.probe_B;
de = tip + M(:,1)*V.range_m(m);
set(H.dock, 'XData', [tip(1) de(1)], 'YData', [tip(2) de(2)], 'ZData', [tip(3) de(3)]);
k = [H.line_idx(H.line_idx < m), m];                % [-] decimated samples up to m
set(H.trace, 'XData', V.rho_O(k,1), 'YData', V.rho_O(k,2), 'ZData', V.rho_O(k,3)); % maintains trajectory drawing 

%% ===== Global view =====
set(H.markI, 'XData', V.r_I_E(m,1)*km, 'YData', V.r_I_E(m,2)*km, 'ZData', V.r_I_E(m,3)*km);
set(H.markC, 'XData', V.r_C_E(m,1)*km, 'YData', V.r_C_E(m,2)*km, 'ZData', V.r_C_E(m,3)*km);
set(H.arcI, 'XData', V.r_I_E(k,1)*km, 'YData', V.r_I_E(k,2)*km, 'ZData', V.r_I_E(k,3)*km);
set(H.arcC, 'XData', V.r_C_E(k,1)*km, 'YData', V.r_C_E(k,2)*km, 'ZData', V.r_C_E(k,3)*km);

%% ===== HUD =====
H.hud.String = hudText(V, m);

%% ===== Camera settings =====
ax = H.axP;
up = [0 0 -1];                                       % zenith (-o3) up
prev = '';
if isstruct(ax.UserData) && isfield(ax.UserData, 'camera')
    prev = ax.UserData.camera;
end
switch cam
    case 'chaser'
        d = 30*norm(H.probe_B)/2.4;                  % [m] follows the chaser display size
        set(ax, 'CameraTarget', rho', 'CameraPosition', rho' + d*[-0.55 -0.75 -0.37], ...
            'CameraUpVector', up, 'CameraViewAngle', 30);
    case 'docking'
        b1 = M(:,1)';
        set(ax, 'CameraTarget', rho' + 40*b1, 'CameraPosition', rho' - 18*b1 + 4*up, ...
            'CameraUpVector', up, 'CameraViewAngle', 35);
    otherwise
        if ~strcmp(prev, 'iss')
            r = max(V.range_m) + 80;
            set(ax, 'CameraTarget', [-0.4*max(V.range_m) 0 0], 'CameraPosition', r*[-0.9 -1.3 -0.8], ...
                'CameraUpVector', up, 'CameraViewAngleMode', 'auto');
        end
end
if ~strcmp(prev, cam)
    ax.UserData = struct('camera', cam);
end

%% ===== Draw =====
if do_draw
    drawnow limitrate;
end
end



%% ===== Local functions =====
function s = hudText(V, m)
% Heads-up text for sample m (true values only).
%
% Inputs:
%   V - struct, visual trajectory  [mixed]
%   m - double [1], sample index
% Outputs:
%   s - cell of char, HUD lines

flag = 'xyz';
st = '---';
sh = '---';
st(V.sat_tau(m,:)) = flag(V.sat_tau(m,:));
sh(V.sat_h(m,:)) = flag(V.sat_h(m,:));
s = {sprintf('%s   t = %7.1f / %.1f s', V.ctrl_name, V.t(m), V.t(end)), ...
     sprintf('range %7.1f m   att. error %8.4f deg   pointing error %8.4f deg', ...
             V.range_m(m), V.att_err_deg(m), V.e_point_deg(m)), ...
     sprintf('torque saturated: %s   momentum saturated: %s', st, sh)};
if V.mag_k ~= 1
    s{end+1} = sprintf('drawn attitude error x%g (display only)', V.mag_k);
end
s{end+1} = V.pos_note;
if ~isempty(V.point_note)
    s{end+1} = V.point_note;
end
end
