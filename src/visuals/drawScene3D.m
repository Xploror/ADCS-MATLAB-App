function H = drawScene3D(axG, axP, V, Mx, opts)
% Create every graphics object of the 3D playback once (global orbit view and LVLH proximity view).
%
% Inputs:
%   axG  - axes or uiaxes handle, global Earth-centered view (display ECI, km)
%   axP  - axes or uiaxes handle, proximity view centered on the ISS (LVLH O, m)
%   V    - struct, output of buildVisualTrajectory
%   Mx   - struct, output of spacecraftMeshes
%   opts - struct (optional); missing fields take defaults:
%          chaser_scale - double [1], display enlargement of the chaser (1 = true size)
%          arrow_len_m  - double [1], length L of a full-scale torque arrow (as in V)   [m]
%          texture_file - char, Earth image ('' = assets/textures/earth.jpg)
% Outputs:
%   H    - struct of graphics handles used by updateScene3D, plus
%          earth_source (char) and layer groups (hggroup handles) for visibility:
%          H.layer.orbits, .sun, .lvlh, .iss, .chaser, .reference, .los, .torque, .trace;
%          H.line_idx (at most about 500 samples drawn on the growing lines) and H.axP
%
% Units: the global view is drawn in km, the proximity view in metres.
% The proximity camera keeps the zenith (-o3) up.

%% ===== Options =====
if nargin < 5 || isempty(opts)
    opts = struct();
end
kc = optField(opts, 'chaser_scale', 1);              % [-] chaser enlargement
L = optField(opts, 'arrow_len_m', 3);                % [m] full-scale torque arrow
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
tex_file = optField(opts, 'texture_file', fullfile(root, 'assets', 'textures', 'earth.jpg'));
space = [0.03 0.03 0.08];                            % [-] background colour
fg = [0.85 0.85 0.85];                               % [-] text and axis colour
H = struct();

%% ===== Global view: Earth, orbits, markers, Sun =====
cla(axG);
hold(axG, 'on');
km = 1e-3;                                           % [km/m]
[X, Y, Z] = sphere(72);
R = V.R_E*km;
[img, H.earth_source] = earthTexture(tex_file);
if isempty(img)
    H.earth = surf(axG, R*X, R*Y, R*Z, 'FaceColor', [0.20 0.40 0.80], 'EdgeColor', 'none');
else
    H.earth = surf(axG, R*X, R*Y, R*Z, 'CData', img, 'FaceColor', 'texturemap', 'EdgeColor', 'none');
end
H.earth.FaceLighting = 'gouraud';
H.layer.orbits = hggroup('Parent', axG);
plot3(V.orbit_E(:,1)*km, V.orbit_E(:,2)*km, V.orbit_E(:,3)*km, '--', 'Color', [0.5 0.8 1], 'Parent', H.layer.orbits);
H.arcI = plot3(V.r_I_E(1,1)*km, V.r_I_E(1,2)*km, V.r_I_E(1,3)*km, '-', 'Color', [0.3 0.9 1], 'LineWidth', 2, 'Parent', H.layer.orbits);
H.arcC = plot3(V.r_C_E(1,1)*km, V.r_C_E(1,2)*km, V.r_C_E(1,3)*km, ':', 'Color', [1 0.6 0.2], 'LineWidth', 2, 'Parent', H.layer.orbits);
H.markI = plot3(axG, V.r_I_E(1,1)*km, V.r_I_E(1,2)*km, V.r_I_E(1,3)*km, 'o', 'MarkerSize', 9, ...
                'MarkerFaceColor', [0.3 0.9 1], 'MarkerEdgeColor', 'w');
H.markC = plot3(axG, V.r_C_E(1,1)*km, V.r_C_E(1,2)*km, V.r_C_E(1,3)*km, '^', 'MarkerSize', 7, ...
                'MarkerFaceColor', [1 0.6 0.2], 'MarkerEdgeColor', 'w');
H.layer.sun = hggroup('Parent', axG);
s = 1.6*R*V.sun_E;
line([0 s(1)], [0 s(2)], [0 s(3)], 'Color', [1 0.85 0.2], 'LineWidth', 1.5, 'Parent', H.layer.sun);
text(s(1), s(2), s(3), ' Sun', 'Color', [1 0.85 0.2], 'Parent', H.layer.sun);
light(axG, 'Position', V.sun_E, 'Style', 'infinite');
axis(axG, 'equal');
styleAxes(axG, space, fg);
xlabel(axG, 'x_{ECI} [km]');
ylabel(axG, 'y_{ECI} [km]');
zlabel(axG, 'z_{ECI} [km]');
view(axG, [-40 25]);
title(axG, sprintf('Orbit view (chaser orbit lies within %.1f km of the ISS orbit)', max(V.range_m)/1e3), ...
      'Color', fg, 'Interpreter', 'none');
hold(axG, 'off');

%% ===== Proximity view: LVLH axes definitions =====
cla(axP);
hold(axP, 'on');
H.layer.lvlh = hggroup('Parent', axP);
la = 40;                                             % [m] LVLH axis length
names = {'V-bar (+o_1)', 'H-bar (+o_2)', 'R-bar (+o_3, nadir)'};
for j = 1:3
    e = zeros(3,1);
    e(j) = la;
    line([0 e(1)], [0 e(2)], [0 e(3)], 'Color', [0.6 0.6 0.6], 'LineStyle', '-.', 'Parent', H.layer.lvlh);
    text(e(1), e(2), e(3), [' ' names{j}], 'Color', [0.7 0.7 0.7], 'FontSize', 8, 'Parent', H.layer.lvlh);
end

%% ===== Proximity view: ISS =====
H.layer.iss = hggroup('Parent', axP);
H.issTf = hgtransform('Parent', H.layer.iss, 'Matrix', [V.M_SO zeros(3,1); zeros(1,3) 1]);
drawParts(Mx.iss, H.issTf);
drawTriad(H.issTf, 30, {'x_S', 'y_S', 'z_S'}, '-');
pe = Mx.port_S + 25*Mx.port_axis_S;
line([Mx.port_S(1) pe(1)], [Mx.port_S(2) pe(2)], [Mx.port_S(3) pe(3)], 'Color', [1 0.3 1], ...
     'LineStyle', '--', 'LineWidth', 1.5, 'Parent', H.issTf);
text(pe(1), pe(2), pe(3), ' target port (illustrative)', 'Color', [1 0.5 1], 'FontSize', 8, 'Parent', H.issTf);

%% ===== Proximity view: chaser, reference ghost, torque arrows =====
H.layer.chaser = hggroup('Parent', axP);
H.chsTf = hgtransform('Parent', H.layer.chaser);
H.chsScale = hgtransform('Parent', H.chsTf, 'Matrix', makehgtform('scale', kc));
drawParts(Mx.chaser, H.chsScale);
drawTriad(H.chsScale, 6, {'b_1', 'b_2', 'b_3'}, '-');
H.layer.reference = hggroup('Parent', axP);
H.refTf = hgtransform('Parent', H.layer.reference);
H.refScale = hgtransform('Parent', H.refTf, 'Matrix', makehgtform('scale', kc));
drawTriad(H.refScale, 7, {'', '', ''}, ':');
H.layer.torque = hggroup('Parent', H.chsScale);
H.arrowColor = [0.95 0.35 0.35; 0.35 0.95 0.35; 0.45 0.55 1.00];
H.arrow = gobjects(1, 4);
for j = 1:3
    H.arrow(j) = quiver3(axP, 0, 0, 0, 0, 0, 0, 0, 'Color', H.arrowColor(j,:), 'LineWidth', 2.5, 'MaxHeadSize', 0.6);
    H.arrow(j).Parent = H.layer.torque;
end
H.arrow(4) = quiver3(axP, 0, 0, 0, 0, 0, 0, 0, 'Color', [1 1 1], 'LineWidth', 1.5, 'MaxHeadSize', 0.6);
H.arrow(4).Parent = H.layer.torque;
H.arrow_len_m = L;

%% ===== Proximity view: line of sight, docking axis, approach trace =====
H.layer.los = hggroup('Parent', axP);
H.los = line([0 0], [0 0], [0 0], 'Color', [0.8 0.8 0.8], 'LineStyle', ':', 'Parent', H.layer.los);
H.dock = line([0 0], [0 0], [0 0], 'Color', [1 0.9 0.2], 'LineWidth', 1.5, 'Parent', H.layer.los);
H.layer.trace = hggroup('Parent', axP);
H.trace = line(V.rho_O(1,1), V.rho_O(1,2), V.rho_O(1,3), 'Color', [1 0.6 0.2], 'Parent', H.layer.trace);
H.probe_B = Mx.chaser_probe_B*kc;
N = numel(V.t);
H.line_idx = unique([round(linspace(1, N, 500)), N]);   % [-] samples drawn on the trace and arc lines
H.axP = axP;

%% ===== Proximity view: limits, camera, lights, HUD =====
lo = min([V.rho_O; -60 -60 -60], [], 1) - 20;
hi = max([V.rho_O; 60 60 60], [], 1) + 20;
axis(axP, 'equal');
xlim(axP, [lo(1) hi(1)]);
ylim(axP, [lo(2) hi(2)]);
zlim(axP, [lo(3) hi(3)]);
axP.Clipping = 'off';
styleAxes(axP, space, fg);
xlabel(axP, 'o_1 [m]');
ylabel(axP, 'o_2 [m]');
zlabel(axP, 'o_3 [m]');
light(axP, 'Position', [0.3 0.2 -1], 'Style', 'infinite');
light(axP, 'Position', [-0.5 -0.8 0.4], 'Style', 'infinite', 'Color', [0.4 0.4 0.45]);
H.hud = text(axP, 0.01, 0.99, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
             'Color', fg, 'FontSize', 9, 'FontName', 'FixedWidth', 'Interpreter', 'none');
title(axP, sprintf('Proximity view, LVLH centred on the ISS (ISS mesh: %s)', meshLabel(Mx.iss_source)), ...
      'Color', fg, 'Interpreter', 'none');
axP.UserData = struct('camera', '');
hold(axP, 'off');
end

%% ===== Local functions =====
function drawParts(parts, parent)
% Draw mesh parts as lit patches under a transform.
%
% Inputs:
%   parts  - struct array, mesh parts (name, V, F, color)
%   parent - hgtransform handle
% Outputs:
%   (none)

for k = 1:numel(parts)
    patch('Parent', parent, 'Vertices', parts(k).V, 'Faces', parts(k).F, 'FaceColor', parts(k).color, ...
          'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'AmbientStrength', 0.45);
end
end

function drawTriad(parent, len, labels, style)
% Red/green/blue axis triad under a transform.
%
% Inputs:
%   parent - hgtransform handle                   [-]
%   len    - double [1], axis length              [m]
%   labels - cell [1x3] of char, axis labels      [-]
%   style  - char, line style                     [-]
% Outputs:
%   (none)

col = [1 0.25 0.25; 0.25 1 0.25; 0.35 0.5 1];
for j = 1:3
    e = zeros(3,1);
    e(j) = len;
    line([0 e(1)], [0 e(2)], [0 e(3)], 'Color', col(j,:), 'LineStyle', style, 'LineWidth', 2, 'Parent', parent);
    if ~isempty(labels{j})
        text(e(1), e(2), e(3), [' ' labels{j}], 'Color', col(j,:), 'FontSize', 8, 'Parent', parent);
    end
end
end

function styleAxes(ax, bg, fg)
% Dark space background with light axis lines and labels.
%
% Inputs:
%   ax - axes handle            [-]
%   bg - double [1x3], RGB      [-]
%   fg - double [1x3], RGB      [-]
% Outputs:
%   (none)

ax.Color = bg;
ax.XColor = fg;
ax.YColor = fg;
ax.ZColor = fg;
ax.GridColor = [0.4 0.4 0.4];
grid(ax, 'on');
end

function [img, src] = earthTexture(file)
% Earth texture for the sphere: image file, else topo.mat, else none.
%
% Inputs:
%   file - char, equi-rectangular Earth image (north at the top)   [-]
% Outputs:
%   img  - uint8/double [MxNx3] RGB with row 1 = south (sphere order), or [] [-]
%   src  - char, which source was used                             [-]

img = [];
src = 'plain sphere';
if ~isempty(file) && exist(file, 'file') == 2
    img = flipud(imread(file));
    if size(img, 3) == 1
        img = repmat(img, [1 1 3]);
    end
    src = file;
elseif exist('topo.mat', 'file') == 2
    S = load('topo.mat', 'topo');
    h = circshift(S.topo, [0 180]);                  % [m] elevation, row 1 = south, column 1 = -180 deg
    img = zeros([size(h) 3]);
    sea = h < 0;                                     % ocean mask
    land = min(max(h, 0)/4000, 1);                   % normalised land height
    depth = min(max(-h, 0)/6000, 1);                 % normalised ocean depth
    img(:,:,1) = ~sea.*(0.25 + 0.45*land) + sea*0.05;
    img(:,:,2) = ~sea.*(0.45 + 0.25*land) + sea*0.20;
    img(:,:,3) = ~sea.*(0.15 + 0.35*land) + sea.*(0.65 - 0.30*depth);
    src = 'topo.mat';
end
end

function s = meshLabel(src)
% Short label for the ISS mesh source.
%
% Inputs:
%   src - char, Mx.iss_source  [-]
% Outputs:
%   s   - char, 'built-in' or 'NASA model'  [-]

if strcmp(src, 'built-in')
    s = 'built-in';
else
    s = 'NASA model';
end
end

function v = optField(s, name, default)
% Field of an options struct, or the default when it is missing or empty.
%
% Inputs:
%   s       - struct, options                          [-]
%   name    - char, field name                         [-]
%   default - any, value used when the field is absent  [-]
% Outputs:
%   v       - any, the option value                     [-]

if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
