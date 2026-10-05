function J = thetaToInertia(theta)
% Symmetric inertia matrix from the inertia parameter vector.
%
% Inputs:
%   theta - double [6x1], [Jxx; Jyy; Jzz; Jxy; Jxz; Jyz] in kg*m^2
% Outputs:
%   J     - double [3x3], symmetric inertia matrix

J = [theta(1), theta(4), theta(5);
     theta(4), theta(2), theta(6);
     theta(5), theta(6), theta(3)];
end
