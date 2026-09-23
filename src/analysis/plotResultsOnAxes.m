function plotResultsOnAxes(ax, results, quantity, cfg)
%PLOTRESULTSONAXES Overlay one quantity of all results on an axes or uiaxes handle.
%
% Inputs:
%   ax       - axes or uiaxes handle to draw into (cleared first)        [-]
%   results  - struct [1xK], result structs (docs/DESIGN_SPEC.md 6)     [mixed]
%   quantity - char, one of:
%              'att_err'   principal error angle, log scale              [deg]
%              'w_e'       rate-error norm                               [deg/s]
%              'tau_rw'    wheel torque norm, dashed +tau_max line       [N*m]
%              'h_w'       per-sample max |h_w| over axes, dashed h_max  [N*m*s]
%              'qnorm'     |qnorm - 1|, log scale                        [-]
%              'theta_hat' theta_hat_i/theta_true_i of the adaptive run  [-]
%   cfg      - struct, user config (optional; cfg.metrics.settle_thresh_deg
%              draws the settling threshold on 'att_err')                [mixed]
% Outputs:
%   (none; draws into ax)

%% ===== Setup =====
if nargin < 4
    cfg = [];
end
colors = [42 120 214; 235 104 52; 27 175 122]/255;       % [-] RGB per ctrl_id 1..3 (blue/orange/aqua, CVD-validated categorical slots 1-3; same as the technical report)
cla(ax);
hold(ax, 'on');
h = [];
labels = {};
logY = false;

%% ===== Quantity-specific plotting =====
switch quantity
    case 'att_err'
        for k = 1:numel(results)
            r = results(k);
            h(end+1) = plot(ax, r.t, max(r.att_err_deg, 1e-6), 'Color', localColor(colors, r.ctrl_id), 'LineWidth', 1.2); %#ok<AGROW>
            labels{end+1} = r.ctrl_name; %#ok<AGROW>
        end
        if isstruct(cfg) && isfield(cfg, 'metrics') && isfield(cfg.metrics, 'settle_thresh_deg') && ~isempty(results)
            tt = [results(1).t(1), results(1).t(end)];
            plot(ax, tt, cfg.metrics.settle_thresh_deg*[1 1], 'k:', 'LineWidth', 1);
        end
        logY = true;
        yl = 'Attitude error [deg]';
        ti = 'Principal attitude error';

    case 'w_e'
        for k = 1:numel(results)
            r = results(k);
            v = sqrt(sum(r.w_e.^2, 2))*180/pi;
            h(end+1) = plot(ax, r.t, v, 'Color', localColor(colors, r.ctrl_id), 'LineWidth', 1.2); %#ok<AGROW>
            labels{end+1} = r.ctrl_name; %#ok<AGROW>
        end
        yl = '|\omega_e| [deg/s]';
        ti = 'Rate error norm';

    case 'tau_rw'
        for k = 1:numel(results)
            r = results(k);
            v = sqrt(sum(r.tau_rw.^2, 2));
            h(end+1) = plot(ax, r.t, v, 'Color', localColor(colors, r.ctrl_id), 'LineWidth', 1.2); %#ok<AGROW>
            labels{end+1} = r.ctrl_name; %#ok<AGROW>
        end
        if ~isempty(results)
            tt = [results(1).t(1), results(1).t(end)];
            h(end+1) = plot(ax, tt, results(1).SC.tau_max*[1 1], 'k--', 'LineWidth', 1);
            labels{end+1} = '\tau_{max} (per wheel)';
        end
        yl = '|\tau_{rw}| [N m]';
        ti = 'Wheel torque norm';

    case 'h_w'
        for k = 1:numel(results)
            r = results(k);
            v = max(abs(r.h_w), [], 2);
            h(end+1) = plot(ax, r.t, v, 'Color', localColor(colors, r.ctrl_id), 'LineWidth', 1.2); %#ok<AGROW>
            labels{end+1} = r.ctrl_name; %#ok<AGROW>
        end
        if ~isempty(results)
            tt = [results(1).t(1), results(1).t(end)];
            h(end+1) = plot(ax, tt, results(1).SC.h_max*[1 1], 'k--', 'LineWidth', 1);
            labels{end+1} = 'h_{max}';
        end
        yl = 'max_i |h_{w,i}| [N m s]';
        ti = 'Wheel momentum (largest axis)';

    case 'qnorm'
        for k = 1:numel(results)
            r = results(k);
            h(end+1) = plot(ax, r.t, max(abs(r.qnorm - 1), 1e-16), 'Color', localColor(colors, r.ctrl_id), 'LineWidth', 1.2); %#ok<AGROW>
            labels{end+1} = r.ctrl_name; %#ok<AGROW>
        end
        logY = true;
        yl = '|q_{norm} - 1| [-]';
        ti = 'Quaternion norm health';

    case 'theta_hat'
        styles = {'-', '--', ':', '-.', '-', '--'};           % [-] line style per theta element (Jxx..Jyz)
        widths = [1.2 1.2 1.5 1.2 2.2 2.2];                   % [pt] line width per theta element (products thicker)
        pnames = {'J_{xx}', 'J_{yy}', 'J_{zz}', 'J_{xy}', 'J_{xz}', 'J_{yz}'}; % [-] legend names of the theta elements
        ia = [];
        for k = 1:numel(results)
            if results(k).ctrl_id == 2
                ia = k;
                break;
            end
        end
        if isempty(ia)
            text(ax, 0.5, 0.5, 'No adaptive (controller 2) run selected', 'Units', 'normalized', 'HorizontalAlignment', 'center');
        else
            r = results(ia);
            th_true = r.SC.theta_true(:).';
            for i = 1:6
                if abs(th_true(i)) > 1e-9
                    h(end+1) = plot(ax, r.t, r.theta_hat(:, i)/th_true(i), 'Color', colors(2,:), ...
                        'LineStyle', styles{i}, 'LineWidth', widths(i)); %#ok<AGROW>
                    labels{end+1} = [pnames{i} ' (' r.ctrl_name ')']; %#ok<AGROW>
                end
            end
            plot(ax, [r.t(1), r.t(end)], [1 1], 'k--', 'LineWidth', 1);
        end
        yl = '\theta_{hat,i} / \theta_{true,i} [-]';
        ti = 'Adaptive inertia estimate';

    otherwise
        hold(ax, 'off');
        error('plotResultsOnAxes:quantity', 'Unknown quantity ''%s''.', quantity);
end

%% ===== Decoration =====
if logY
    set(ax, 'YScale', 'log');
else
    set(ax, 'YScale', 'linear');
end
xlabel(ax, 'Time [s]');
ylabel(ax, yl);
title(ax, ti);
grid(ax, 'on');
if ~isempty(h)
    legend(ax, h, labels, 'Location', 'best');
else
    legend(ax, 'off');
end
hold(ax, 'off');
end

%% ===== Local functions =====
function c = localColor(colors, id)
%LOCALCOLOR Fixed RGB colour of a controller id.
%
% Inputs:
%   colors - double [3x3], one RGB row per controller id                 [-]
%   id     - double [1], controller id (1..3)                            [-]
% Outputs:
%   c      - double [1x3], RGB colour                                    [-]

c = colors(min(max(round(id), 1), 3), :);
end
