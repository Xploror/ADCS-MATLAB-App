function L = inertiaLinearMap(v)
%INERTIALINEARMAP Linear map L(v) such that J*v = L(v)*theta.
%
% Inputs:
%   v - double [3x1], vector multiplied by the inertia                    [any]
% Outputs:
%   L - double [3x6], map for theta = [Jxx; Jyy; Jzz; Jxy; Jxz; Jyz]     [same as v]

L = [v(1), 0,    0,    v(2), v(3), 0;
     0,    v(2), 0,    v(1), 0,    v(3);
     0,    0,    v(3), 0,    v(1), v(2)];
end
