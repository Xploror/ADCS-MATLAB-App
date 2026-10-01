function [R, y, z] = approachProfile(t, SC)
% Analytic exponential closing profile of the chaser w.r.t. the ISS.
%
% Inputs:
%   t  - double [1], simulation time in secs
%   SC - struct
% Outputs:
%   R  - double [1], along-track range, R = Rf + (R0-Rf) exp(-t/T_close) in mtrs
%   y  - double [1], cross-track offset in O, y0*(R/R0)^2 in mtrs
%   z  - double [1], radial (nadir-axis) offset in O, z0*(R/R0)^2 in mtrs
%
% The chaser position in O relative to the ISS is [-R; y; z].

R = SC.Rf + (SC.R0 - SC.Rf)*exp(-t/SC.T_close);
ratio = R/SC.R0;
y = SC.y0*ratio^2;
z = SC.z0*ratio^2;
end
