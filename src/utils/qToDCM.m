function C = qToDCM(q)
% Direction cosine matrix C_BN from a scalar-first attitude quaternion.
%
% Inputs:
%   q - double [4x1], attitude quaternion of B w.r.t. N, scalar-first
%       (need not be exactly unit; it is used as given)
% Outputs:
%   C - double [3x3], direction cosine matrix C_BN (v_B = C_BN * v_N)
%
% Convention (docs/DESIGN_SPEC.md 1.2):
%   C_BN = (q0^2 - qv'*qv) I + 2 qv qv' - 2 q0 [qv x]

q0 = q(1);
qv = q(2:4);
C  = (q0^2 - qv'*qv)*eye(3) + 2*(qv*qv') - 2*q0*skew3(qv);
end
