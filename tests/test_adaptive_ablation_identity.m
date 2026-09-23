function test_adaptive_ablation_identity()
%TEST_ADAPTIVE_ABLATION_IDENTITY Adaptive SMC with Gamma = 0 equals robust SMC (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

%% ===== Configuration: Gamma = 0, theta_hat0 = theta_nom =====
cfg = scn_inertia_uncertainty_pm25();          % J_true ~= J_nom so the laws are exercised
cfg.scenario.t_final_s = 30;                   % [s] short run
cfg.gains.adaptive = cfg.gains.robust;
cfg.gains.adaptive.Gamma_diag = zeros(6,1);                            % [kg*m^2*s^2] adaptation off
cfg.gains.adaptive.theta_hat0_from_Jnom = 1;                           % [-] start the estimate at J_nom
cfg.gains.adaptive.theta_hat0 = inertiaToTheta(cfg.scenario.J_nom);   % [kg*m^2] same as J_nom
cfg.gains.adaptive.proj_frac = 0.6;                                    % [-] projection-box half-width fraction

%% ===== Single-call identity =====
[SC, RG, AG, BG] = buildSimParams(cfg);
seedRNG(5);
for k = 1:20
    q = qNormalize(randn(4,1)); w = 0.01*randn(3,1);
    [q_r, w_r, wdot_r] = referenceAttitude(10*k, SC);
    h_w = randn(3,1);
    t1 = controllerSelect(1, q, w, q_r, w_r, wdot_r, h_w, AG.theta_hat0, SC, RG, AG, BG);
    [t2, thd] = controllerSelect(2, q, w, q_r, w_r, wdot_r, h_w, AG.theta_hat0, SC, RG, AG, BG);
    assert(max(abs(t1 - t2)) <= 1e-12, 'controller outputs differ');
    assert(all(thd == 0), 'theta_hat_dot must be 0 with Gamma = 0');
end

%% ===== Closed-loop identity =====
r = runSimulation(cfg, [1 2], 'reference');
d = max(abs(r(1).tau_cmd(:) - r(2).tau_cmd(:)));
assert(d <= 1e-12, sprintf('closed-loop tau_cmd differs by %.3g', d));
end
