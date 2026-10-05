function app = launchADCSApp()
%LAUNCHADCSAPP Set up the project paths if needed and open the ADCS comparison GUI.
%
% Inputs:
%   (none)
% Outputs:
%   app - ADCS_MATLAB_App, handle of the running app (delete(app) closes it) [-]
%
% Example:
%   app = launchADCSApp();

%% ===== Paths (startup_ADCS runs in this function's workspace, never in base) =====
if exist('initDefaults', 'file') ~= 2
    root = fileparts(fileparts(mfilename('fullpath')));   % [-] project root folder
    run(fullfile(root, 'startup_ADCS.m'));
end

%% ===== Launch =====
app = ADCS_MATLAB_App();
end
