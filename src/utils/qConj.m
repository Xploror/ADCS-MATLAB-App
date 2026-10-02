function qc = qConj(q)
% Scaler-first Quaternion conjugate
%
% Inputs:
%   q  - double [4x1], quaternion, scalar-first
% Outputs:
%   qc - double [4x1], conjugate [q0; -qv]

qc = [q(1); -q(2:4)];
end
