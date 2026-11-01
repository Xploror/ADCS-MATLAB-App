function test_visual_meshes()
%TEST_VISUAL_MESHES Checks of spacecraftMeshes: valid triangle meshes, sizes, port, and the NASA STL loader (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

%% ===== Built-in meshes (no STL file) =====
Mx = spacecraftMeshes(struct('iss_file', 'no_such_file.stl'));
assert(strcmp(Mx.iss_source, 'built-in'), 'missing STL must fall back to the built-in ISS');
checkParts(Mx.chaser, 'chaser');
checkParts(Mx.iss, 'ISS');
[lo, hi] = extent(Mx.iss);
assert(abs((hi(2) - lo(2)) - 109) < 1e-9, 'built-in ISS truss is not 109 m');
[lo, hi] = extent(Mx.chaser);
assert(all(hi - lo < 15) && hi(1) > 2, 'chaser size or docking probe unexpected');
assert(norm(Mx.chaser_probe_B - [2.4; 0; 0]) < 1e-12, 'probe tip');
assert(norm(Mx.port_axis_S - [-1; 0; 0]) < 1e-12 && Mx.port_S(1) < 0, 'aft port on -x_S');

%% ===== NASA STL loader (MATLAB only: needs stlwrite / stlread) =====
if exist('stlwrite', 'file') == 2 && exist('stlread', 'file') == 2
    f = [tempname(), '.stl'];
    P = [0 0 0; 10 0 0; 0 4 0; 0 0 2];               % [m] small tetrahedron, largest extent 10
    T = [1 3 2; 1 2 4; 2 3 4; 3 1 4];
    stlwrite(triangulation(T, P), f);
    cleanup = onCleanup(@() delete(f));
    Mx = spacecraftMeshes(struct('iss_file', f));
    assert(strcmp(Mx.iss_source, f), 'STL file not used');
    assert(numel(Mx.iss) == 1, 'STL must load as one part');
    checkParts(Mx.iss, 'STL ISS');
    [lo, hi] = extent(Mx.iss);
    assert(abs(max(hi - lo) - 109) < 1e-6, 'STL not scaled to 109 m');
    assert(norm((hi + lo)/2) < 1e-6, 'STL not centred');
    assert(abs(Mx.port_S(1) - lo(1)) < 1e-9, 'STL port not on the aft face');
end
end

%% ===== Local functions =====
function checkParts(parts, tag)
%CHECKPARTS Every part has finite vertices, valid triangle indices and an RGB colour.
%
% Inputs:
%   parts - struct array, mesh parts (name, V, F, color)  [-]
%   tag   - char, label used in error messages            [-]
% Outputs:
%   (none; throws on failure)

assert(~isempty(parts), [tag ': no parts']);
for k = 1:numel(parts)
    p = parts(k);
    assert(size(p.V, 2) == 3 && all(isfinite(p.V(:))), [tag ': bad vertices in ' p.name]);
    assert(size(p.F, 2) == 3 && all(p.F(:) >= 1) && all(p.F(:) <= size(p.V, 1)), [tag ': bad faces in ' p.name]);
    assert(numel(p.color) == 3 && all(p.color >= 0 & p.color <= 1), [tag ': bad colour in ' p.name]);
end
end

function [lo, hi] = extent(parts)
%EXTENT Bounding box of all parts.
%
% Inputs:
%   parts - struct array, mesh parts  [-]
% Outputs:
%   lo, hi - double [1x3], minimum and maximum coordinates  [m]

P = cat(1, parts.V);
lo = min(P, [], 1);
hi = max(P, [], 1);
end
