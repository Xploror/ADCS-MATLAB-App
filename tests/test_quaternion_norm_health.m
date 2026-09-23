function test_quaternion_norm_health()
%TEST_QUATERNION_NORM_HEALTH Raw quaternion norm stays near 1 in a nominal run (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

cfg = scn_nominal();
cfg.scenario.t_final_s = 100;                  % [s] shortened nominal run
r = runSimulation(cfg, 1, 'reference');
dev = max(abs(r.qnorm - 1));
assert(all(isfinite(r.qnorm)), 'qnorm not finite');
assert(dev < 1e-3, sprintf('max |qnorm-1| = %.3g', dev));
qn = sqrt(sum(r.q.^2, 2));
assert(max(abs(qn - 1)) < 1e-12, 'logged q is not normalised');
end
