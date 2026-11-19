function check_visual_playback(visible)
% Opt-in check of the 3D renderer: builds the scene, steps 200 frames, verifies transforms and arrows.
%
% Inputs:
%   visible - logical [1] (optional), true = show the window and report the
%             measured frame rate; false (default) = invisible, functional only [-]
% Outputs:
%   (none; throws an error if any check fails, prints the frame rate)
%
% Not auto-discovered by run_all_tests (it needs MATLAB graphics).

%% ===== Guards =====
if nargin < 1
    visible = false;
end
% if exist('OCTAVE_VERSION', 'builtin') ~= 0 || ~usejava('jvm') || (~visible && batchStartupOptionUsed)
%     fprintf('check_visual_playback: SKIP (needs MATLAB graphics)\n');
%     return;
% end

%% ===== Scene =====
cfg = scn_nominal();
cfg.scenario.t_final_s = 60;                          % [s] short run
[SC, RG, AG, BG] = buildSimParams(cfg);
res = simulateADCS_ref(SC, RG, AG, BG, 2);
V = buildVisualTrajectory(res, struct('mag_k', 10));
Mx = spacecraftMeshes();
fig = uifigure('Visible', visible, 'Position', [100 100 1200 600]);
cleanup = onCleanup(@() delete(fig));
g = uigridlayout(fig, [1 2]);
axG = uiaxes(g);
axP = uiaxes(g);
H = drawScene3D(axG, axP, V, Mx, struct('chaser_scale', 5));

%% ===== Step through the run =====
N = numel(V.t);
idx = round(linspace(1, N, 200));
cams = {'iss', 'chaser', 'docking'};
t0 = tic;
for k = 1:numel(idx)
    updateScene3D(H, V, idx(k), struct('camera', cams{mod(k, 3) + 1}, 'draw', visible));
end
if visible
    drawnow;
    fps = numel(idx)/toc(t0);
    fprintf('check_visual_playback: %.1f frames per second (drawnow limitrate caps at 20)\n', fps);
end

%% ===== Transforms and arrows equal the library geometry =====
m = idx(end);
T = H.chsTf.Matrix;
assert(norm(T(1:3,1:3) - V.M_BO_disp(:,:,m)) < 1e-12 && norm(T(1:3,4) - V.rho_O(m,:).') < 1e-9, ...
       'chaser transform does not match M_BO_disp / rho_O');
S = H.chsScale.Matrix;
for j = 1:3
    a = [H.arrow(j).UData; H.arrow(j).VData; H.arrow(j).WData];
    aO = T(1:3,1:3)*S(1:3,1:3)*a;                     % [m] arrow in O as drawn
    assert(norm(aO - 5*V.A_O(:,j,m)) < 1e-9, 'torque arrow does not match A_O');
end
assert(norm(H.refTf.Matrix(1:3,1:3) - V.M_RO(:,:,m)) < 1e-12, 'reference transform');
fprintf('check_visual_playback: PASS (%d frames, Earth: %s)\n', numel(idx), H.earth_source);
end
