function [r_hat_N, v_hat_N, C_ON] = orbitGeometry(t, SC)
%ORBITGEOMETRY Circular-orbit position/velocity directions and the LVLH DCM at time t.
%
% Inputs:
%   t  - double [1], simulation time (argument of latitude theta = n*t)  [s]
%   SC - struct, sim constants; uses SC.n (mean motion)                   [rad/s]
% Outputs:
%   r_hat_N - double [3x1], unit radial (zenith) direction in N           [-]
%   v_hat_N - double [3x1], unit velocity (along-track) direction in N    [-]
%   C_ON    - double [3x3], LVLH DCM, rows o1 = v_hat, o2 = -orbit normal,
%             o3 = nadir = -r_hat (docs/DESIGN_SPEC.md 1.1)               [-]

th = SC.n*t;
c = cos(th);
s = sin(th);
r_hat_N = [c; s; 0];
v_hat_N = [-s; c; 0];
C_ON = [-s,  c,  0;
         0,  0, -1;
        -c, -s,  0];
end
