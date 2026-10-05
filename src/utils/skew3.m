function S = skew3(v)
% Cross-product (skew-symmetric) matrix so that skew3(a)*b = cross(a,b).
%
% Inputs:
%   v - double [3x1], vector
% Outputs:
%   S - double [3x3], [0 -v3 v2; v3 0 -v1; -v2 v1 0]

S = [0,    -v(3),  v(2);
     v(3),  0,    -v(1);
    -v(2),  v(1),  0];
end
