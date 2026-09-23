function q = qFromAxisAngle(axis, angle_rad)
%QFROMAXISANGLE Quaternion of a frame rotated by +angle about a (normalised) axis.
%
% Inputs:
%   axis      - double [3x1], rotation axis (normalised internally;
%               a zero axis returns the identity quaternion)            [-]
%   angle_rad - double [1], rotation angle                               [rad]
% Outputs:
%   q         - double [4x1], [cos(phi/2); e*sin(phi/2)], scalar-first  [-]
%               Its DCM is cos(phi) I + (1-cos(phi)) e e' - sin(phi) [e x].

e = [axis(1); axis(2); axis(3)];
ne = sqrt(e.'*e);
q = [1; 0; 0; 0];
if ne > 1e-15
    e = e/ne;
    q = [cos(0.5*angle_rad); e*sin(0.5*angle_rad)];
end
end
