function q = qMult(qa, qb)
% Quaternion composition such that qToDCM(qMult(qa,qb)) = qToDCM(qa)*qToDCM(qb).
%
% Inputs:
%   qa - double [4x1], left quaternion, scalar-first
%   qb - double [4x1], right quaternion, scalar-first
% Outputs:
%   q  - double [4x1], product [a0 b0 - av.bv ; a0 bv + b0 av - av x bv]

a0 = qa(1); av = qa(2:4);
b0 = qb(1); bv = qb(2:4);
q = zeros(4,1);
q(1)   = a0*b0 - av'*bv;
q(2:4) = a0*bv + b0*av - cross(av, bv);
end
