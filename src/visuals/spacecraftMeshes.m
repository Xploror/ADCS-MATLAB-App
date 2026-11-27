function Mx = spacecraftMeshes(opts)
% Triangle meshes of the chaser (always built in) and the ISS (NASA STL if present, else built in).
%
% Inputs:
%   opts - struct (optional); missing fields take defaults:
%          iss_file - char, STL file of the ISS ('' = assets/models/iss.stl) [-]
%          iss_R    - double [3x3], rotation applied to the STL vertices     [-]
%          iss_span - double [1], size the STL is scaled to (largest extent) [m]
% Outputs:
%   Mx   - struct with fields
%          chaser, iss - struct arrays of parts {name (char), V [Kx3] vertices
%                        in body axes [m], F [Mx3] triangle indices [-],
%                        color [1x3] RGB [-]}
%          iss_source  - char, 'built-in' or the STL file used          [-]
%          port_S      - double [3x1], ISS target port position (S axes) [m]
%          port_axis_S - double [3x1], unit outward port axis (S axes)   [-]
%          chaser_probe_B - double [3x1], tip of the chaser docking probe [m]
%
% Body axes: chaser b1 is the docking axis; ISS x_S points along the
% velocity (+XVV), z_S to nadir (+ZLV), the truss lies along y_S. The
% ISS target port is the aft port on -x_S (approach from behind).

%% ===== Options =====
if nargin < 1 || isempty(opts)
    opts = struct();
end
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));   % [-] project root
iss_file = optField(opts, 'iss_file', fullfile(root, 'assets', 'models', 'iss.stl'));
iss_R = optField(opts, 'iss_R', eye(3));             % [-] orientation fix for the STL
iss_span = optField(opts, 'iss_span', 109);          % [m] ISS truss length (largest extent)

%% ===== Chaser (built in, about 3 m bus) =====
grey = [0.70 0.70 0.72];
blue = [0.10 0.18 0.45];
c = struct('name', {}, 'V', {}, 'F', {}, 'color', {});
c(end+1) = boxPart('bus', [-1.5 1.5; -1.1 1.1; -1.1 1.1], grey);
c(end+1) = cylPart('docking probe', [1.5; 0; 0], [2.4; 0; 0], 0.35, [0.85 0.85 0.85]);
c(end+1) = boxPart('solar array +y', [-0.8 0.8; 1.1 6.1; -0.03 0.03], blue);
c(end+1) = boxPart('solar array -y', [-0.8 0.8; -6.1 -1.1; -0.03 0.03], blue);
Mx.chaser = c;
Mx.chaser_probe_B = [2.4; 0; 0];

%% ===== ISS: NASA STL if available, else built in =====
Mx.iss_source = 'built-in';
s = struct('name', {}, 'V', {}, 'F', {}, 'color', {});
if ~isempty(iss_file) && exist(iss_file, 'file') == 2 && exist('stlread', 'file') == 2 % NASA STL exists
    TR = stlread(iss_file);
    P = TR.Points*iss_R';
    ctr = (max(P, [], 1) + min(P, [], 1))/2;         % [m] bounding-box centre
    P = P - repmat(ctr, size(P, 1), 1);
    span = max(max(P, [], 1) - min(P, [], 1));
    P = P*(iss_span/span);                           % [m] scaled to the ISS size
    s(end+1) = struct('name', 'ISS (NASA model)', 'V', P, 'F', TR.ConnectivityList, 'color', [0.80 0.80 0.80]);
    Mx.iss_source = iss_file;
    Mx.port_S = [min(P(:,1)); 0; 0];
else % ISS built-in
    gold = [0.55 0.45 0.20];
    s(end+1) = boxPart('truss', [-1.5 1.5; -54.5 54.5; -5.5 -2.5], grey);
    s(end+1) = cylPart('aft module', [-28; 0; 0], [-12; 0; 0], 2.1, [0.85 0.85 0.85]);
    s(end+1) = cylPart('core modules', [-12; 0; 0], [8; 0; 0], 2.2, [0.90 0.90 0.90]);
    s(end+1) = cylPart('forward node', [8; 0; 0], [22; 0; 0], 2.1, [0.85 0.85 0.85]);
    yc = [-48 -36 36 48];                            % [m] array centres along the truss
    for k = 1:numel(yc)
        s(end+1) = boxPart(sprintf('array %d fwd', k), [3 36; yc(k) - 5.75 yc(k) + 5.75; -4.05 -3.95], gold); %#ok<AGROW>
        s(end+1) = boxPart(sprintf('array %d aft', k), [-36 -3; yc(k) - 5.75 yc(k) + 5.75; -4.05 -3.95], gold); %#ok<AGROW>
    end
    Mx.port_S = [-28; 0; 0];
end
Mx.iss = s;
Mx.port_axis_S = [-1; 0; 0];
end

%% ===== Local functions =====
function p = boxPart(name, lim, color)
% Axis-aligned box as 12 triangles.
%
% Inputs:
%   name  - char, part name                                     [-]
%   lim   - double [3x2], [xmin xmax; ymin ymax; zmin zmax]       [m]
%   color - double [1x3], RGB                                   [-]
% Outputs:
%   p     - struct, part with fields name, V [8x3], F [12x3], color

[X, Y, Z] = ndgrid(lim(1,:), lim(2,:), lim(3,:));
V = [X(:), Y(:), Z(:)];                              % vertex k = 1 + ix + 2 iy + 4 iz
Q = fliplr([1 3 7 5; 2 6 8 4; 1 5 6 2; 3 4 8 7; 1 2 4 3; 5 7 8 6]);   % quads, wound outward
F = [Q(:, [1 2 3]); Q(:, [1 3 4])];
p = struct('name', name, 'V', V, 'F', F, 'color', color);
end

function p = cylPart(name, a, b, r, color)
% Closed cylinder from point a to point b as triangles (24 segments).
%
% Inputs:
%   name  - char, part name                  [-]
%   a, b  - double [3x1], axis end points    [m]
%   r     - double [1], radius               [m]
%   color - double [1x3], RGB                [-]
% Outputs:
%   p     - struct, part with fields name, V, F, color

ns = 24;                                             % [-] segments
ax = (b - a)/norm(b - a);
tmp = [1; 0; 0];
if abs(ax'*tmp) > 0.9
    tmp = [0; 1; 0];
end
e1 = cross(ax, tmp);
e1 = e1/norm(e1);
e2 = cross(ax, e1);
ang = (0:ns - 1)*2*pi/ns;
ring = e1*cos(ang) + e2*sin(ang);                    % [3 x ns] unit ring
V = [(repmat(a, 1, ns) + r*ring)'; (repmat(b, 1, ns) + r*ring)'; a'; b'];
F = zeros(4*ns, 3);
for k = 1:ns
    kn = mod(k, ns) + 1;
    F(4*k - 3, :) = [k, kn, ns + kn];                % side
    F(4*k - 2, :) = [k, ns + kn, ns + k];            % side
    F(4*k - 1, :) = [2*ns + 1, kn, k];               % cap at a
    F(4*k, :)     = [2*ns + 2, ns + k, ns + kn];     % cap at b
end
p = struct('name', name, 'V', V, 'F', F, 'color', color);
end

function v = optField(s, name, default)
% Field of an options struct, or the default when it is missing or empty.
%
% Inputs:
%   s       - struct, options
%   name    - char, field name
%   default - any, value used when the field is absent
% Outputs:
%   v       - any, the option value

if isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = default;
end
end
