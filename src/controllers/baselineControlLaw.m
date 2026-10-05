function tau_c = baselineControlLaw(q, w, q_r, w_r, wdot_r, SC, BG)
% Quaternion PD benchmark controller.
%
% Inputs:
%   q      - double [4x1], measured attitude quaternion B w.r.t. N
%   w      - double [3x1], measured body rate, B axes in rad/s
%   q_r    - double [4x1], reference quaternion
%   w_r    - double [3x1], reference rate, R axes in rad/s
%   wdot_r - double [3x1], reference rate derivative, R axes (unused by PD) in rad/s^2
%   SC     - struct
%   BG     - struct, Baseline Gain properties
% Outputs:
%   tau_c  - double [3x1], control torque before wheel-gyro compensation
%            and clamping in N*m

[q_e, w_e] = attitudeErrors(q, w, q_r, w_r, wdot_r, zeros(3,1), SC.shortest_path);
tau_c = -BG.Kp_diag.*q_e(2:4) - BG.Kd_diag.*w_e;
end
