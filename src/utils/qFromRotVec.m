function q = qFromRotVec(phi_rad)
%QFROMROTVEC Quaternion from a rotation vector (axis times angle).
%
% Inputs:
%   phi_rad - double [3x1], rotation vector, |phi| = angle, direction = axis [rad]
% Outputs:
%   q       - double [4x1], unit quaternion, scalar-first                     [-]

p = [phi_rad(1); phi_rad(2); phi_rad(3)];
a = sqrt(p.'*p);
if a < 1e-8
    % small-angle series: q ~ [1 - a^2/8; p/2 (1 - a^2/24)], then normalise
    q = qNormalize([1 - a*a/8; 0.5*p*(1 - a*a/24)]);
else
    q = [cos(0.5*a); (p/a)*sin(0.5*a)];
end
end
