function theta = inertiaToTheta(J)
% Inertia parameter vector from a symmetric inertia matrix.
%
% Inputs:
%   J     - double [3x3], symmetric inertia matrix in kg*m^2
% Outputs:
%   theta - double [6x1], [Jxx; Jyy; Jzz; Jxy; Jxz; Jyz]

theta = [J(1,1); J(2,2); J(3,3); J(1,2); J(1,3); J(2,3)];
end
