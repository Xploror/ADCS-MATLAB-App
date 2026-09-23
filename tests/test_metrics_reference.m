function test_metrics_reference()
%TEST_METRICS_REFERENCE computeMetrics/evaluatePassFail on a hand-computed synthetic result.
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

%% ===== Synthetic result =====
t = (0:0.5:10).';                              % [s] 21 samples
N = numel(t);
e = 2*ones(N,1);                               % [deg]
e(t >= 3) = 0.2;                               % settles at t = 3 s
e(t == 5) = 0.6;                               % excursion -> settle at 5.5 s
e(t >= 9) = 0.1;                               % last 10% window t >= 9: [0.1 0.1 0.1]
tau = zeros(N,3);
tau(t <= 2, 1) = 0.1;                          % |tau| = 0.1 on [0, 2]
tau(t == 1, 2) = -0.15;                        % peak 0.15 at t = 1
flags = zeros(N,6);
flags(t <= 1, 1) = 1;                          % torque-sat flags at t = 0, 0.5, 1
flags(t == 4, 5) = 1;                          % momentum flag at t = 4
tcmd = zeros(N,3);
tcmd(2,1) = 1;                                 % TV = |1| + |-1| = 2 over 10 s -> 0.2
res = struct('t', t, 'att_err_deg', e, 'tau_rw', tau, 'tau_cmd', tcmd, ...
             'h_w', [linspace(0, -3, N).', zeros(N,2)], 'sat_flags', flags, ...
             'qnorm', 1 + [zeros(N-1,1); 2e-4], 'theta_hat', repmat([1 2 3 0 0 0], N, 1), ...
             'ctrl_id', 2);
res.SC = struct('theta_true', [1; 2; 4; 0; 0; 0]);
cfg = struct('metrics', struct('settle_thresh_deg', 0.5, 'ss_window_frac', 0.1)); % [deg], [-] settling threshold, ss window fraction

%% ===== Metrics =====
M = computeMetrics(res, cfg);
tol = 1e-12;                                             % [-] numerical comparison tolerance
assert(abs(M.settle_time_s - 5.5) < tol, 'settle time');
assert(abs(M.ss_err_deg - 0.1) < tol, 'ss error');
assert(abs(M.final_err_deg - 0.1) < tol && abs(M.peak_err_deg - 2) < tol, 'final/peak error');
assert(abs(M.peak_torque_Nm - 0.15) < tol, 'peak torque');
% |tau| samples: 0.1 at t=0,0.5,1.5,2; sqrt(0.1^2+0.15^2) at t=1; 0 afterwards
n1 = sqrt(0.1^2 + 0.15^2);
L1 = 0.5*(0.5*(0.1+0.1) + 0.5*(0.1+n1) + 0.5*(n1+0.1) + 0.5*(0.1+0.1) + 0.5*(0.1+0));
L2 = 0.5*(0.5*(0.01+0.01) + 0.5*(0.01+n1^2) + 0.5*(n1^2+0.01) + 0.5*(0.01+0.01) + 0.5*(0.01+0));
assert(abs(M.effort_L1 - L1) < tol, 'effort L1');
assert(abs(M.effort_L2 - L2) < tol, 'effort L2');
assert(abs(M.peak_h_Nms - 3) < tol, 'peak h');
assert(abs(M.sat_time_torque_s - 1.5) < tol, 'torque saturation time');
assert(abs(M.sat_time_mom_s - 0.5) < tol, 'momentum saturation time');
assert(abs(M.chatter_TV - 0.2) < tol, 'chatter TV');
assert(abs(M.qnorm_max_dev - 2e-4) < 1e-15, 'qnorm deviation');
assert(abs(M.theta_err_final_pct - 100*1/norm([1 2 4])) < 1e-9, 'theta error');
assert(M.all_finite, 'all_finite');

%% ===== Pass/fail =====
P = struct('ss_err_max_deg', 0.5, 'settle_time_max_s', Inf, 'qnorm_tol', 1e-3); % [deg], [s], [-] pass criteria
[ok, why] = evaluatePassFail(M, P);
assert(ok && isempty(why), 'should pass');
P.settle_time_max_s = 5;                                 % [s] tighter than the settling time -> must fail
[ok, why] = evaluatePassFail(M, P);
assert(~ok && numel(why) == 1, 'settle-time criterion');
P.settle_time_max_s = Inf; P.qnorm_tol = 1e-4;           % [s], [-] only the qnorm criterion can fail
assert(~evaluatePassFail(M, P), 'qnorm criterion');

%% ===== Non-adaptive, non-settling, non-finite =====
res.ctrl_id = 1;                                         % [-] robust SMC (non-adaptive)
res.att_err_deg(end) = 0.9;
res.w = [NaN 0 0; zeros(N-1,3)];
M = computeMetrics(res, cfg);
assert(isnan(M.theta_err_final_pct), 'theta error must be NaN for non-adaptive');
assert(isnan(M.settle_time_s), 'settle time must be NaN if never settled');
assert(~M.all_finite, 'NaN must clear all_finite');
end
