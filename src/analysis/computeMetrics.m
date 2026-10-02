function M = computeMetrics(res, cfg)
% Performance metrics of one simulation result.
%
% Inputs:
%   res - struct, result struct                         [mixed]
%   cfg - struct, user config; uses cfg.metrics.settle_thresh_deg [deg]
%         and cfg.metrics.ss_window_frac [-] (defaults 0.5 and 0.1 if absent)
% Outputs:
%   M   - struct with fields
%         settle_time_s [s], ss_err_deg [deg], final_err_deg [deg], peak_err_deg [deg],
%         peak_torque_Nm [N*m], effort_L1 [N*m*s], effort_L2 [N^2*m^2*s],
%         peak_h_Nms [N*m*s], sat_time_torque_s [s], sat_time_mom_s [s],
%         chatter_TV [N*m/s], qnorm_max_dev [-], theta_err_final_pct [%],
%         all_finite (logical)
%
% Numerical conventions: integrals use the trapezoidal rule; saturation
% times use forward rectangles (flag(k) held over [t_k, t_k+1)).

%% ===== Settings =====
thresh = 0.5;                                            % [deg] default settling threshold
frac = 0.1;                                              % default steady-state window fraction
if nargin > 1 && isstruct(cfg) && isfield(cfg, 'metrics')
    if isfield(cfg.metrics, 'settle_thresh_deg'), thresh = cfg.metrics.settle_thresh_deg; end
    if isfield(cfg.metrics, 'ss_window_frac'),    frac   = cfg.metrics.ss_window_frac;    end
end
t = res.t;
e = res.att_err_deg;
N = numel(t);
T = t(end) - t(1);

%% ===== Attitude-error metrics =====
M = struct();
idx = find(~(e <= thresh), 1, 'last');       % last sample above threshold (NaN counts as above)
if isempty(idx)
    M.settle_time_s = t(1);
elseif idx == N
    M.settle_time_s = NaN;
else
    M.settle_time_s = t(idx + 1);
end
win = t >= t(end) - frac*T;
M.ss_err_deg    = mean(e(win));
M.final_err_deg = e(end);
M.peak_err_deg  = max(e);

%% ===== Actuator metrics =====
tn = sqrt(sum(res.tau_rw.^2, 2));
M.peak_torque_Nm = max(abs(res.tau_rw(:)));
M.effort_L1 = localTrapz(t, tn);
M.effort_L2 = localTrapz(t, tn.^2);
M.peak_h_Nms = max(abs(res.h_w(:)));
dtk = diff(t);
if N > 1
    M.sat_time_torque_s = sum(double(any(res.sat_flags(1:N-1, 1:3) > 0.5, 2)).*dtk);
    M.sat_time_mom_s    = sum(double(any(res.sat_flags(1:N-1, 4:6) > 0.5, 2)).*dtk);
    M.chatter_TV = sum(sum(abs(diff(res.tau_cmd, 1, 1))))/max(T, eps);
else
    M.sat_time_torque_s = 0;
    M.sat_time_mom_s = 0;
    M.chatter_TV = 0;
end

%% ===== Health and estimation metrics =====
M.qnorm_max_dev = max(abs(res.qnorm(:) - 1));
M.theta_err_final_pct = NaN;
if res.ctrl_id == 2
    th_true = res.SC.theta_true(:);
    th_end = res.theta_hat(end, :).';
    M.theta_err_final_pct = 100*norm(th_end - th_true)/norm(th_true);
end
ts = {'t','q','w','q_r','w_r','q_e','w_e','att_err_deg','s','tau_cmd','tau_rw','h_w', ...
      'theta_hat','tau_d','tau_gg','tau_aero','tau_srp','tau_mag','qnorm','sat_flags'};
ok = true;
for k = 1:numel(ts)
    if isfield(res, ts{k})
        v = res.(ts{k});
        ok = ok && all(isfinite(v(:)));
    end
end
M.all_finite = ok;
end

%% ===== Local functions =====
function I = localTrapz(t, y)
% Trapezoidal integral of y(t).
%
% Inputs:
%   t - double [Nx1], time samples                                        [s]
%   y - double [Nx1], integrand samples                                   [any]
% Outputs:
%   I - double [1], integral                                              [any*s]

if numel(t) < 2
    I = 0;
else
    I = sum(0.5*(y(1:end-1) + y(2:end)).*diff(t));
end
end
