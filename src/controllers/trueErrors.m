function [q_e, w_e, att_err_deg] = trueErrors(q, w, q_r, w_r, SC)
%TRUEERRORS True-state attitude and rate errors for logging and metrics.
%
% Inputs:
%   q   - double [4x1], true attitude quaternion B w.r.t. N              [-]
%   w   - double [3x1], true body rate, B axes                           [rad/s]
%   q_r - double [4x1], reference quaternion R w.r.t. N                  [-]
%   w_r - double [3x1], reference rate, R axes                           [rad/s]
%   SC  - struct, uses SC.shortest_path [-]
% Outputs:
%   q_e         - double [4x1], error quaternion (C(q_e) = C_BR)         [-]
%   w_e         - double [3x1], rate error w - C_BR w_r, B axes          [rad/s]
%   att_err_deg - double [1], principal error angle                      [deg]

q_e = qMult(q, qConj(q_r));
if (SC.shortest_path > 0.5) && (q_e(1) < 0)
    q_e = -q_e;
end
w_e = w - qToDCM(q_e)*w_r;
att_err_deg = qErrorAngleDeg(q_e);
end
