function q = qFromAxisAngle(axis, angle_rad)
% Quaternion of a frame rotated by +angle about a (normalised) axis.
%
% Inputs:
%   axis      - double [3x1], rotation axis (normalised internally;
%               a zero axis returns the identity quaternion)
%   angle_rad - double [1], rotation angle
% Outputs:
%   q         - double [4x1], [cos(phi/2); axis*sin(phi/2)], scalar-first
%               Its DCM is cos(phi) I + (1-cos(phi)) axis axis' - sin(phi) [e axis].

n_axis = sqrt(axis'*axis);
q = [1; 0; 0; 0];
if n_axis > 1e-15 % zero-check
    axis = axis/n_axis;
    q = [cos(0.5*angle_rad); axis*sin(0.5*angle_rad)];
end
end
