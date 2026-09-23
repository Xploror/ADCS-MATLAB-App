function [h_w_dot, tau_rw, sat_flags] = reactionWheelModel(tau_cmd, h_w, SC)
%REACTIONWHEELMODEL Three body-aligned reaction wheels with torque and momentum limits.
%
% Inputs:
%   tau_cmd - double [3x1], commanded body torque                        [N*m]
%   h_w     - double [3x1], wheel angular momentum, B axes               [N*m*s]
%   SC      - struct, uses SC.tau_max [N*m], SC.h_max [N*m*s]
% Outputs:
%   h_w_dot   - double [3x1], wheel momentum rate                        [N*m]
%   tau_rw    - double [3x1], reaction torque applied to the body        [N*m]
%   sat_flags - double [6x1], 0/1 flags: (1:3) torque at/over the limit,
%               (4:6) momentum cut-off active                            [-]
%
% Note: controllerSelect already clamps tau_cmd to +-tau_max, so a strict
% `|tau_cmd| > tau_max` test could never fire. The torque flag therefore uses
% |tau_cmd| >= tau_max*(1 - 1e-9), i.e. `the command is on the limit`.

%% ===== Torque saturation =====
tau_s = min(max(tau_cmd, -SC.tau_max), SC.tau_max);
h_w_dot = -tau_s;
sat_flags = zeros(6,1);

%% ===== Momentum cut-off and flags =====
for i = 1:3
    if abs(tau_cmd(i)) >= SC.tau_max*(1 - 1e-9)
        sat_flags(i) = 1;
    end
    if (h_w(i) >= SC.h_max && h_w_dot(i) > 0) || (h_w(i) <= -SC.h_max && h_w_dot(i) < 0)
        h_w_dot(i) = 0;
        sat_flags(3+i) = 1;
    end
end
tau_rw = -h_w_dot;
end
