function q = qFromRotVec(phi_rad)
% Quaternion from a rotation vector (axis times angle).
%
% Inputs:
%   phi_rad - double [3x1], rotation vector, |phi| = angle, direction = axis
% Outputs:
%   q       - double [4x1], unit quaternion, scalar-first

ang = sqrt(phi_rad'*phi_rad);
if ang < 1e-8
    % small-angle series: q ~ [1 - a^2/8; p/2 (1 - a^2/24)], then normalise
    q = qNormalize([1 - ang^2/8; 0.5*phi_rad*(1 - ang^2/24)]);
else
    q = [cos(0.5*ang); (phi_rad/ang)*sin(0.5*ang)];
end
end
