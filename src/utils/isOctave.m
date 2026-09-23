function tf = isOctave()
%ISOCTAVE True when running under GNU Octave, false under MATLAB.
%
% Inputs:
%   (none)
% Outputs:
%   tf - logical [1], true if the interpreter is GNU Octave               [-]

tf = exist('OCTAVE_VERSION', 'builtin') ~= 0;
end
