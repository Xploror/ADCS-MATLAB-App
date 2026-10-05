function q = dcmToQ(C)
% Scalar-first quaternion from a direction cosine matrix (Shepperd's method).
%
% Inputs:
%   C - double [3x3], proper orthogonal direction cosine matrix C_BN
% Outputs:
%   q - double [4x1], unit quaternion of B w.r.t. N, scalar-first, q0>=0
%
% Shepperd (1978): choose the numerically largest of {q0^2, q1^2, q2^2, q3^2}
% to avoid dividing by a small number (robust near 180 deg rotations).
% Relations for C_BN = (q0^2-|qv|^2)I + 2 qv qv' - 2 q0 [qv x]:
%   4 q0 q1 = C23 - C32,  4 q0 q2 = C31 - C13,  4 q0 q3 = C12 - C21
%   4 q1 q2 = C12 + C21,  4 q1 q3 = C13 + C31,  4 q2 q3 = C23 + C32

%% ===== Select the largest component =====
tr = C(1,1) + C(2,2) + C(3,3);
cand = [tr; C(1,1); C(2,2); C(3,3)];
[~, imax] = max(cand);

%% ===== Compute quaternion components =====
q = zeros(4,1);
if imax == 1
    q0 = 0.5*sqrt(max(1 + tr, 0));
    f  = 0.25/q0;
    q(1) = q0;
    q(2) = (C(2,3) - C(3,2))*f;
    q(3) = (C(3,1) - C(1,3))*f;
    q(4) = (C(1,2) - C(2,1))*f;
elseif imax == 2
    q1 = 0.5*sqrt(max(1 + 2*C(1,1) - tr, 0));
    f  = 0.25/q1;
    q(2) = q1;
    q(1) = (C(2,3) - C(3,2))*f;
    q(3) = (C(1,2) + C(2,1))*f;
    q(4) = (C(1,3) + C(3,1))*f;
elseif imax == 3
    q2 = 0.5*sqrt(max(1 + 2*C(2,2) - tr, 0));
    f  = 0.25/q2;
    q(3) = q2;
    q(1) = (C(3,1) - C(1,3))*f;
    q(2) = (C(1,2) + C(2,1))*f;
    q(4) = (C(2,3) + C(3,2))*f;
else
    q3 = 0.5*sqrt(max(1 + 2*C(3,3) - tr, 0));
    f  = 0.25/q3;
    q(4) = q3;
    q(1) = (C(1,2) - C(2,1))*f;
    q(2) = (C(1,3) + C(3,1))*f;
    q(3) = (C(2,3) + C(3,2))*f;
end

%% ===== Sign convention and normalisation =====
if q(1) < 0
    q = -q;
end
q = qNormalize(q);
end
