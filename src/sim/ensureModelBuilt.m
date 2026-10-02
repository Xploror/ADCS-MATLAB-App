function mdl = ensureModelBuilt()
% Build the Simulink harness if it is missing, then load it.
%
% Inputs:
%   (none)
% Outputs:
%   mdl - char [1xM], model name 'ADCS_ComparisonHarness' (optional output)
%
% If models/ADCS_ComparisonHarness.slx is missing, build_ADCS_model(false) is
% called. If only the data dictionary is missing, the model is rebuilt with
% overwrite = true (the .slx is unusable without its dictionary). models/ is
% added to the MATLAB path so the dictionary can be resolved by file name.

%% ===== Paths =====
mdl       = 'ADCS_ComparisonHarness';                    %  model name
rootDir   = fileparts(fileparts(fileparts(mfilename('fullpath')))); %  project root (src/sim/..)
modelsDir = fullfile(rootDir, 'models');                 %  models folder
slxPath   = fullfile(modelsDir, [mdl '.slx']);           %  model file
ddPath    = fullfile(modelsDir, 'ADCS_Params.sldd');     %  dictionary file
if ~any(strcmp(strsplit(path, pathsep), modelsDir))
    addpath(modelsDir);
end

%% ===== Build if necessary =====
if ~isfile(slxPath)
    build_ADCS_model(false);
elseif ~isfile(ddPath)
    warning('ensureModelBuilt:noDictionary', ...
        'ADCS_Params.sldd is missing; rebuilding the model and dictionary.');
    build_ADCS_model(true);
end

%% ===== Load =====
if ~bdIsLoaded(mdl)
    load_system(slxPath);
end
end
