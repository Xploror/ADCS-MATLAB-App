function L = inertiaLinearMap(v)
% Linear map L(v) such that J*v = L(v)*theta.
%
% Inputs:
%   v - double [3x1], vector multiplied by the inertia
% Outputs:
%   L - double [3x6], map for theta

L = [v(1), 0,    0,    v(2), v(3), 0;
     0,    v(2), 0,    v(1), 0,    v(3);
     0,    0,    v(3), 0,    v(1), v(2)];
end
