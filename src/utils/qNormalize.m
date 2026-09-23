function qn = qNormalize(q)
%QNORMALIZE Normalise a quaternion to unit length (identity if the input is zero).
%
% Inputs:
%   q  - double [4x1], quaternion, scalar-first                          [-]
% Outputs:
%   qn - double [4x1], unit quaternion (q/|q|, or [1;0;0;0] if |q|=0)    [-]

nq = sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3) + q(4)*q(4));
qn = [1; 0; 0; 0];
if nq > 0
    qn = [q(1); q(2); q(3); q(4)]/nq;
end
end
