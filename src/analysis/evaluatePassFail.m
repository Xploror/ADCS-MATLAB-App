function [pass, reasons] = evaluatePassFail(M, passCfg)
%EVALUATEPASSFAIL Check a metrics struct against the scenario pass criteria.
%
% Inputs:
%   M       - struct, metrics from computeMetrics                         [mixed]
%   passCfg - struct, cfg.pass: ss_err_max_deg [deg], settle_time_max_s [s],
%             qnorm_tol [-]
% Outputs:
%   pass    - logical [1], true if every criterion is met                 [-]
%   reasons - cell [1xR] of char, one message per failed criterion
%             (empty cell if pass)                                        [-]

reasons = {};

%% ===== Criteria =====
if ~M.all_finite
    reasons{end+1} = 'non-finite values in the result';
end
if ~(M.ss_err_deg <= passCfg.ss_err_max_deg)
    reasons{end+1} = sprintf('ss error %.3g deg > %.3g deg', M.ss_err_deg, passCfg.ss_err_max_deg);
end
if ~(M.qnorm_max_dev <= passCfg.qnorm_tol)
    reasons{end+1} = sprintf('|qnorm-1| %.3g > %.3g', M.qnorm_max_dev, passCfg.qnorm_tol);
end
if isfinite(passCfg.settle_time_max_s)
    if isnan(M.settle_time_s)
        reasons{end+1} = 'never settled';
    elseif M.settle_time_s > passCfg.settle_time_max_s
        reasons{end+1} = sprintf('settle time %.4g s > %.4g s', M.settle_time_s, passCfg.settle_time_max_s);
    end
end

%% ===== Verdict =====
pass = isempty(reasons);
end
