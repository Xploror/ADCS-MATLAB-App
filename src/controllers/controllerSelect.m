function [tau_cmd, theta_hat_dot, s] = controllerSelect(ctrl_id, q_m, w_m, q_r, w_r, wdot_r, h_w, theta_hat, SC, RG, AG, BG)
%CONTROLLERSELECT Dispatch to the selected control law, add wheel-gyro compensation, clamp.
%
% Inputs:
%   ctrl_id   - double [1], 1 robust SMC, 2 adaptive SMC, 3 PD benchmark [-]
%   q_m       - double [4x1], measured attitude quaternion               [-]
%   w_m       - double [3x1], measured body rate, B axes                 [rad/s]
%   q_r       - double [4x1], reference quaternion                       [-]
%   w_r       - double [3x1], reference rate, R axes                     [rad/s]
%   wdot_r    - double [3x1], reference rate derivative, R axes          [rad/s^2]
%   h_w       - double [3x1], wheel angular momentum, B axes             [N*m*s]
%   theta_hat - double [6x1], adaptive inertia estimate                  [kg*m^2]
%   SC        - struct, uses SC.J_nom, SC.theta_nom, SC.tau_max [N*m], SC.h_max [N*m*s],
%               SC.gyro_comp [-], SC.shortest_path [-] (never SC.J_true)
%   RG, AG, BG - structs, robust/adaptive/baseline gains (docs/DESIGN_SPEC.md 4.2)
% Outputs:
%   tau_cmd       - double [3x1], clamped torque command                 [N*m]
%   theta_hat_dot - double [6x1], estimate derivative (0 unless ctrl 2)  [kg*m^2/s]
%   s             - double [3x1], sliding variable (0 for ctrl 3)        [rad/s]

%% ===== Control law =====
tau_c = zeros(3,1);
theta_hat_dot = zeros(6,1);
s = zeros(3,1);
if ctrl_id < 1.5
    [tau_c, s] = robustControlLaw(q_m, w_m, q_r, w_r, wdot_r, SC, RG);
elseif ctrl_id < 2.5
    [tau_c, theta_hat_dot, s] = adaptiveControlLaw(q_m, w_m, q_r, w_r, wdot_r, theta_hat, SC, AG);
else
    tau_c = baselineControlLaw(q_m, w_m, q_r, w_r, wdot_r, SC, BG);
end

%% ===== Common post-processing =====
tau_cmd = tau_c + SC.gyro_comp*cross(w_m, h_w);        % [N*m] unclamped command incl. gyro compensation

%% ===== Adaptation anti-windup (controller 2 only) =====
% While the actuator cannot deliver the commanded torque, the closed loop no
% longer matches the model used to derive the adaptive law; integrating
% theta_hat then drives it onto the projection bounds (observed in the slew
% scenarios). The estimate is therefore held constant while any torque
% command exceeds tau_max or any wheel is at its momentum limit.
if ctrl_id > 1.5 && ctrl_id < 2.5 && AG.freeze_on_sat > 0.5
    torque_sat = any(abs(tau_cmd) > SC.tau_max);
    mom_sat    = any(abs(h_w) >= SC.h_max*(1 - 1e-6));
    if torque_sat || mom_sat
        theta_hat_dot = zeros(6,1);
    end
end

%% ===== Actuator envelope =====
tau_cmd = min(max(tau_cmd, -SC.tau_max), SC.tau_max);
end
