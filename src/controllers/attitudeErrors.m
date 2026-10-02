function [q_e, w_e, w_dB, qev_dot, w_rv, w_rv_dot, s] = attitudeErrors(q, w, q_r, w_r, wdot_r, lambda_diag, shortest_path)
% Shared tracking errors, virtual reference rate and sliding variable.
%
% Inputs:
%   q             - double [4x1], (measured) attitude quaternion B w.r.t. N
%   w             - double [3x1], (measured) body rate w.r.t. N, B axes
%   q_r           - double [4x1], reference quaternion R w.r.t. N
%   w_r           - double [3x1], reference rate w.r.t. N, R axes in rad/s
%   wdot_r        - double [3x1], reference rate derivative, R axes in rad/s^2
%   lambda_diag   - double [3x1], diagonal of the sliding-surface slope Lambda
%   shortest_path - double [1], 1 = flip q_e so that q_e0 >= 0
% Outputs:
%   q_e      - double [4x1], error quaternion, C(q_e) = C_BR
%   w_e      - double [3x1], rate error w - C_BR w_r, B axes in rad/s
%   w_dB     - double [3x1], reference rate expressed in B axes in rad/s
%   qev_dot  - double [3x1], derivative of the error-quaternion vector part
%   w_rv     - double [3x1], virtual reference rate w_dB - Lambda q_ev in rad/s
%   w_rv_dot - double [3x1], derivative of w_rv, B axes in rad/s^2
%   s        - double [3x1], sliding variable in rad/s
%
% Equations: docs/DESIGN_SPEC.md section 2.

%% ===== Attitude error =====
q_e = qMult(q, qConj(q_r));
if (shortest_path > 0.5) && (q_e(1) < 0)
    q_e = -q_e;
end
q_ev = q_e(2:4);
C_e = qToDCM(q_e);

%% ===== Rate errors =====
w_dB = C_e*w_r;
w_e = w - w_dB;
qev_dot = 0.5*(q_e(1)*eye(3) + skew3(q_ev))*w_e;

%% ===== Virtual reference and sliding variable =====
Lambda = diag(lambda_diag);
w_rv = w_dB - Lambda*q_ev;
w_rv_dot = -skew3(w_e)*w_dB + C_e*wdot_r - Lambda*qev_dot;
s = w_e + Lambda*q_ev;
end
