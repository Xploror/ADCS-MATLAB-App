function [R, y, z] = approachProfile(t, SC)
%APPROACHPROFILE Analytic exponential closing profile of the chaser w.r.t. the ISS.
%
% Inputs:
%   t  - double [1], simulation time                                      [s]
%   SC - struct, uses SC.R0 [m], SC.Rf [m], SC.T_close [s], SC.y0 [m], SC.z0 [m]
% Outputs:
%   R  - double [1], along-track range, R = Rf + (R0-Rf) exp(-t/T_close) [m]
%   y  - double [1], cross-track offset in O, y0*(R/R0)^2                 [m]
%   z  - double [1], radial (nadir-axis) offset in O, z0*(R/R0)^2         [m]
%
% The chaser position in O relative to the ISS is [-R; y; z].

R = SC.Rf + (SC.R0 - SC.Rf)*exp(-t/SC.T_close);
ratio = R/SC.R0;
y = SC.y0*ratio*ratio;
z = SC.z0*ratio*ratio;
end
