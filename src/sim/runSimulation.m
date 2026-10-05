function results = runSimulation(cfg, ctrl_ids, engine)
% Run the experiment on a chosen engine (supports MATLAB/Simulink(priority))
%
% Inputs:
%   cfg      - struct, user-level configuration (initDefaults / scn_*)
%   ctrl_ids - double [1xK], controller ids (1 robust, 2 adaptive, 3 PD);
%              optional, default [1 2 3]
%   engine   - char, 'auto' | 'simulink' | 'reference'; optional,
%              default 'auto' (Simulink if available, else reference)
% Outputs:
%   results  - struct [1xK], result structs

%% ===== Defaults and parameter build =====
if nargin < 2 || isempty(ctrl_ids)
    ctrl_ids = [1 2 3];                                  % default: all three controllers
end
if nargin < 3 || isempty(engine)
    engine = 'auto';                                     % default engine: Simulink if available, else reference
end
engine = lower(engine);
[SC, RG, AG, BG] = buildSimParams(cfg);

%% ===== Engine selection =====
if strcmp(engine, 'auto')
    if isSimulinkAvailable()
        engine = 'simulink';
    else
        engine = 'reference';
    end
end

%% ===== Run =====
if strcmp(engine, 'simulink')
    ensureModelBuilt();
    results = runSimulinkBatch(SC, RG, AG, BG, ctrl_ids);
elseif strcmp(engine, 'reference')
    for k = 1:numel(ctrl_ids)
        r = simulateADCS_ref(SC, RG, AG, BG, ctrl_ids(k));
        if k == 1
            results = repmat(r, 1, numel(ctrl_ids));
        end
        results(k) = r;
    end
else
    error('runSimulation:badEngine', 'Unknown engine ''%s'' (use auto, simulink or reference).', engine);
end
end
