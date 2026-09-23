function [header, rows] = metricsTable(results, metrics, passFlags)
%METRICSTABLE Tabulate metrics of several results (for uitable, printing and CSV).
%
% Inputs:
%   results   - struct [1xK], result structs (uses ctrl_name, engine)    [mixed]
%   metrics   - struct [1xK] (or cell {1xK}) of metrics from computeMetrics [mixed]
%   passFlags - logical [1xK], pass verdicts; optional (empty -> '-')    [-]
% Outputs:
%   header    - cell [1xC] of char, column titles with units             [-]
%   rows      - cell [KxC], char (controller, engine, pass) or double
%               (metric values, in the units of the header)              [mixed]

%% ===== Column definition =====
cols = { ...
  'settle_time_s',       'Settle [s]'; ...
  'ss_err_deg',          'SS err [deg]'; ...
  'final_err_deg',       'Final err [deg]'; ...
  'peak_err_deg',        'Peak err [deg]'; ...
  'peak_torque_Nm',      'Peak torque [N*m]'; ...
  'effort_L1',           'Effort L1 [N*m*s]'; ...
  'effort_L2',           'Effort L2 [N^2*m^2*s]'; ...
  'peak_h_Nms',          'Peak h [N*m*s]'; ...
  'sat_time_torque_s',   'Torque sat [s]'; ...
  'sat_time_mom_s',      'Momentum sat [s]'; ...
  'chatter_TV',          'Chatter TV [N*m/s]'; ...
  'qnorm_max_dev',       '|qnorm-1| max [-]'; ...
  'theta_err_final_pct', 'Theta err [%]'};
header = [{'Controller', 'Engine'}, cols(:,2).', {'Finite', 'Pass'}];

%% ===== Rows =====
K = numel(results);
if nargin < 3
    passFlags = [];
end
rows = cell(K, numel(header));
for k = 1:K
    if iscell(metrics)
        Mk = metrics{k};
    else
        Mk = metrics(k);
    end
    rows{k,1} = results(k).ctrl_name;
    rows{k,2} = results(k).engine;
    for c = 1:size(cols, 1)
        rows{k, 2 + c} = double(Mk.(cols{c,1}));
    end
    if Mk.all_finite
        rows{k, end-1} = 'yes';
    else
        rows{k, end-1} = 'NO';
    end
    if isempty(passFlags)
        rows{k, end} = '-';
    elseif passFlags(k)
        rows{k, end} = 'PASS';
    else
        rows{k, end} = 'FAIL';
    end
end
end
