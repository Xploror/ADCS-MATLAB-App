function tau_c = baselineControlLaw(q, w, q_r, w_r, wdot_r, SC, BG)
%BASELINECONTROLLAW Quaternion PD benchmark (Wie & Barba), controller 3.
%
% Inputs:
%   q      - double [4x1], measured attitude quaternion B w.r.t. N       [-]
%   w      - double [3x1], measured body rate, B axes                    [rad/s]
%   q_r    - double [4x1], reference quaternion                          [-]
%   w_r    - double [3x1], reference rate, R axes                        [rad/s]
%   wdot_r - double [3x1], reference rate derivative, R axes (unused by PD) [rad/s^2]
%   SC     - struct, uses SC.shortest_path [-]
%   BG     - struct, Kp_diag [3x1] [N*m], Kd_diag [3x1] [N*m*s]
% Outputs:
%   tau_c  - double [3x1], -Kp.*q_ev - Kd.*w_e                           [N*m]

[q_e, w_e] = attitudeErrors(q, w, q_r, w_r, wdot_r, zeros(3,1), SC.shortest_path);
q_ev = [q_e(2); q_e(3); q_e(4)];
tau_c = -BG.Kp_diag.*q_ev - BG.Kd_diag.*w_e;
end
