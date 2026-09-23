function y = satVec(x)
%SATVEC Elementwise unit saturation sat(x) = min(max(x,-1),1).
%
% Inputs:
%   x - double [nx1], input vector                                        [-]
% Outputs:
%   y - double [nx1], saturated vector with entries in [-1, 1]           [-]

y = min(max(x, -1), 1);
end
