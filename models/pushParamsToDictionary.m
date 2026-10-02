function pushParamsToDictionary(cfg, ctrl_id)
% Overwrite the ADCS_Params.sldd entries with values derived from a user config.
%
% Inputs:
%   cfg     - struct, user-level configuration (see initDefaults / spec 4.1) [mixed]
%   ctrl_id - double [1], controller to select: 1 Robust SMC,
%             2 Adaptive SMC, 3 PD benchmark (optional, default 1)
% Outputs:
%   (none; ADCS_Params.sldd is modified and saved)
%
% Writes SC, RobustGains, AdaptiveGains, BaselineGains and CONTROLLER_SELECT
% to the 'Design Data' section. Use this when you want to run the model
% interactively (Run button in Simulink) with a given configuration. The app
% and runSimulinkBatch do not need it: they override the same variables per
% run through Simulink.SimulationInput.setVariable.

%% ===== Arguments =====
if nargin < 2 || isempty(ctrl_id)
    ctrl_id = 1;                                         % [-] default controller: Robust SMC
end
if ~isscalar(ctrl_id) || ~any(ctrl_id == [1 2 3])
    error('pushParamsToDictionary:badCtrl', 'ctrl_id must be 1, 2 or 3.');
end
modelsDir = fileparts(mfilename('fullpath'));            % [-] absolute path of models/
ddPath    = fullfile(modelsDir, 'ADCS_Params.sldd');     % [-] dictionary file
if ~isfile(ddPath)
    error('pushParamsToDictionary:noDictionary', ...
        '%s does not exist. Run build_ADCS_model(true) first.', ddPath);
end

%% ===== Derive sim-level parameters =====
[SC, RG, AG, BG] = buildSimParams(cfg);
names  = {'SC', 'RobustGains', 'AdaptiveGains', 'BaselineGains', 'CONTROLLER_SELECT'};
values = {SC, RG, AG, BG, double(ctrl_id)};

%% ===== Write entries =====
dd  = Simulink.data.dictionary.open(ddPath);
sec = getSection(dd, 'Design Data');
for k = 1:numel(names)
    if exist(sec, names{k})
        entry = getEntry(sec, names{k});
        setValue(entry, values{k});
    else
        addEntry(sec, names{k}, values{k});
    end
end
saveChanges(dd);
close(dd);
end
