function tf = isSimulinkAvailable()
% True if running in MATLAB with an installed and licensed Simulink.
%
% Outputs:
%   tf - logical [1], true when the Simulink engine can be used
%
% Always false in GNU Octave. Any error during the check yields false.
tf = false;
try
    if exist('OCTAVE_VERSION', 'builtin') ~= 0
        return;                                          % GNU Octave: no Simulink
    end
    tf = logical(license('test', 'Simulink')) && ~isempty(ver('simulink'));
catch
    tf = false;
end
end
