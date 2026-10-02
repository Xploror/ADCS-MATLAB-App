function qn = qNormalize(q)
% Normalize a quaternion to unit length (identity if the input is zero).
%
% Inputs:
%   q  - double [4x1], quaternion, scalar-first
% Outputs:
%   qn - double [4x1], unit quaternion (q/|q|, or [1;0;0;0] if |q|=0)

nq = norm(q);
qn = [1; 0; 0; 0]; % default qn
if nq > 0
    qn = q/nq;
end
end
