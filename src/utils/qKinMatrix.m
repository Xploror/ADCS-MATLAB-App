function B = qKinMatrix(q)
% Quaternion kinematics matrix B(q) such that qdot = 0.5*B(q)*w.
%
% Inputs:
%   q - double [4x1], attitude quaternion of B w.r.t. N, scalar-first
% Outputs:
%   B - double [4x3], [-qv' ; q0*I3 + [qv x]]
%       (w is the body rate w.r.t. N expressed in B axes, [rad/s])

qv = q(2:4);
B = [-qv'; q(1)*eye(3) + skew3(qv)];
end
