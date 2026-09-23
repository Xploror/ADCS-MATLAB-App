function [mdl, info] = build_ADCS_model(overwrite, injectMiswire)
%BUILD_ADCS_MODEL Programmatically generate the Simulink harness ADCS_ComparisonHarness and its data dictionary.
%
% Inputs:
%   overwrite     - logical/double [1], true = rebuild even if the .slx exists  [-]
%                   (optional, default false)
%   injectMiswire - logical/double [1], TEST HOOK: true = deliberately re-route
%                   TrueErr input w_r to the q_r signal after grouping, to
%                   exercise the wire verify-and-repair path (optional,
%                   default false; see tests/check_model_wiring_repair.m)    [-]
% Outputs:
%   mdl       - char [1xM], name of the generated model ('ADCS_ComparisonHarness') [-]
%   info      - struct, build diagnostics: nWires (wires verified) [-],
%               nRepaired (wires re-routed after grouping) [-]
%
% Generated files (next to this script, in models/):
%   ADCS_ComparisonHarness.slx  - the three-controller comparison harness
%   ADCS_Params.sldd            - data dictionary holding SC, RobustGains,
%                                 AdaptiveGains, BaselineGains, CONTROLLER_SELECT
%
% Usage:
%   startup_ADCS;  build_ADCS_model(true);
%
% ---------------------------------------------------------------------------
% BLOCK DIAGRAM (flat view before grouping; [Sub] = subsystem after grouping)
% ---------------------------------------------------------------------------
%
%  [RefGenerator]                                   [TrueErrors]
%   Clock --t--+--> RefGen --q_r,w_r,wdot_r--+----> TrueErr --> q_e,w_e,att_err_deg (log, Term)
%              |    (referenceAttitude)      |         ^  ^
%              |                             |         |  |
%              |   [DisturbanceTorques]      |         |  |
%              +--> Dist(t,q) --tau_d--------|-----+   |  |
%                   (tau_gg..tau_mag: log, Term)   |   |  |
%                    ^                       |     |   |  |
%   [Sensors]        | q                     v     |   |  |
%   RandAtt,RandGyro,RandBias --n_*--> Sensor --q_m,w_m--> Ctrl --tau_cmd--> RW --tau_rw--+
%   IntBias(b) <--b_dot------------------ ^ ^      [ControllerSelect]  [ReactionWheelActuator]
%                                         | |      Ctrl --s (log, Term)     |  |         |
%                                         | |      IntTheta <--theta_hat_dot   |  h_w_dot  |
%                                         | |      IntTheta --theta_hat--> Ctrl  v         |
%                                         | |                           IntHw --h_w--> Ctrl,RW,Dyn
%                                         | |                                              |
%  [RigidBodyDynamics3DOF]                | |                                              v
%   Dyn(q_raw,w,tau_rw,tau_d,h_w) --qdot_raw--> IntQ --q_raw--> QNorm --q--> Sensor,Dist,TrueErr
%        \--wdot--> IntW --w--> Sensor, Dyn, TrueErr     QNorm --qnorm (log, Term)
%
% Every feedback path is closed through an Integrator (IntQ, IntW, IntHw,
% IntTheta, IntBias). Integrators have no direct feedthrough, so the diagram
% contains no algebraic loop.
%
% ALGEBRAIC-LOOP RULE (DESIGN_SPEC section 7): the normalisation
% q = q_raw/|q_raw| is its own MATLAB Function block (QNorm). A MATLAB
% Function block has direct feedthrough from every input to every output. If
% q were computed inside the rigidBodyDerivatives block (which has
% tau_rw and tau_d as inputs), the path q -> Dist -> tau_d -> Dyn -> q would
% contain no integrator and Simulink would report an algebraic loop. As a
% separate block fed only by the q_raw integrator, QNorm breaks that path.
%
% Construction strategy: every block is placed FLAT at the top level, wired,
% line-named and logged, and checked for unconnected ports. Only then are the
% blocks grouped with Simulink.BlockDiagram.createSubsystem. Port numbers of
% each MATLAB Function block are derived from ONE ordered list of input names
% (see addMatlabFcnBlock), which is used both to generate the script and to
% wire the lines.

%% ===== Arguments and paths =====
if nargin < 1 || isempty(overwrite)
    overwrite = false;                                   % [-] default: keep an existing model
end
if nargin < 2 || isempty(injectMiswire)
    injectMiswire = false;                               % [-] default: no fault injection
end
info = struct('nWires', 0, 'nRepaired', 0);              % [-] build diagnostics
mdl       = 'ADCS_ComparisonHarness';                    % [-] model name (spec section 7)
ddName    = 'ADCS_Params.sldd';                          % [-] data dictionary file name
modelsDir = fileparts(mfilename('fullpath'));            % [-] absolute path of models/
rootDir   = fileparts(modelsDir);                        % [-] project root
slxPath   = fullfile(modelsDir, [mdl '.slx']);           % [-] model file
ddPath    = fullfile(modelsDir, ddName);                 % [-] dictionary file

if isfile(slxPath) && ~overwrite
    fprintf('build_ADCS_model: %s already exists (overwrite = false), nothing to do.\n', slxPath);
    return;
end

% The dictionary is resolved by file name on the MATLAB path, so models/
% must be on the path. src/ and config/ are needed for buildSimParams.
if ~any(strcmp(strsplit(path, pathsep), modelsDir))
    addpath(modelsDir);
end
if exist('buildSimParams', 'file') ~= 2 || exist('initDefaults', 'file') ~= 2
    if exist(fullfile(rootDir, 'startup_ADCS.m'), 'file') == 2
        run(fullfile(rootDir, 'startup_ADCS.m'));
    else
        addpath(genpath(fullfile(rootDir, 'src')));
        addpath(genpath(fullfile(rootDir, 'config')));
    end
end

%% ===== Clean up any previous model / dictionary =====
if bdIsLoaded(mdl)
    close_system(mdl, 0);                                 % discard unsaved changes
end
try
    Simulink.data.dictionary.closeAll(ddName, '-discard');
catch
    % dictionary was not open - nothing to close
end
if isfile(slxPath)
    delete(slxPath);
end
if isfile(ddPath)
    delete(ddPath);
end

%% ===== Default parameter values =====
cfg = initDefaults();                                    % user-level defaults
[SC, RG, AG, BG] = buildSimParams(cfg);                  % sim-level numeric structs
CONTROLLER_SELECT = 1;                                   % [-] 1=Robust SMC, 2=Adaptive SMC, 3=PD

%% ===== Data dictionary =====
dd  = Simulink.data.dictionary.create(ddPath);
sec = getSection(dd, 'Design Data');
addEntry(sec, 'SC',                SC);
addEntry(sec, 'RobustGains',       RG);
addEntry(sec, 'AdaptiveGains',     AG);
addEntry(sec, 'BaselineGains',     BG);
addEntry(sec, 'CONTROLLER_SELECT', CONTROLLER_SELECT);
saveChanges(dd);
close(dd);

%% ===== New model and configuration =====
load_system('simulink');                                 % make the library available to add_block
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step');              % must precede 'Solver'
set_param(mdl, 'Solver', 'ode4', ...                     % RK4, same scheme as the reference engine
    'FixedStep', 'SC.dt', ...                            % [s] overridden numerically by the app
    'StopTime', 'SC.t_final', ...                        % [s] overridden numerically by the app
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout', ...
    'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'on', ...
    'SaveOutput', 'off', ...
    'SaveState', 'off');
set_param(mdl, 'AlgebraicLoopMsg', 'error');             % fail loudly if a loop is ever introduced
set_param(mdl, 'DataDictionary', ddName);
try
    % Avoid 'defined in both base workspace and data dictionary' errors.
    set_param(mdl, 'EnableAccessToBaseWorkspace', 'off');
catch
    % parameter not available in this release - see README_models.md
end

%% ===== Block registry (port-name lists) =====
% B.<blockName>.inNames / .outNames hold the ordered port names. All wiring
% goes through these lists, so a port number is never typed twice.
B = struct();

% Column x-positions for the flat layout (arrangeSystem tidies up later).
cx = [30 200 420 700 950 1200 1450];                     % [px] column left edges
bw = 160;  bh = 90;                                      % [px] default MATLAB Fcn block size

%% ===== Sources: Clock and Random Number blocks =====
B = addSimpleBlock(mdl, B, 'Clock', 'simulink/Sources/Clock', {}, {'t'}, ...
    [cx(1) 40 cx(1)+30 70], {});
% Column-vector parameters with VectorParams1D off give [3x1] outputs that
% match the fixed [3 1] Sensor ports (deviation D12).
noiseSpecs = { 'RandAtt',  'SC.noise_seed+[1;2;3]';     % n_att  (attitude noise)
               'RandGyro', 'SC.noise_seed+[4;5;6]';     % n_gyro (gyro white noise)
               'RandBias', 'SC.noise_seed+[7;8;9]' };   % n_bias (bias random walk)
for i = 1:size(noiseSpecs, 1)
    y0 = 300 + 70*(i-1);                                 % [px] vertical position
    B = addSimpleBlock(mdl, B, noiseSpecs{i,1}, 'simulink/Sources/Random Number', ...
        {}, {'n'}, [cx(1) y0 cx(1)+40 y0+30], ...
        {'Mean', '[0;0;0]', ...                          % [-] zero-mean
         'Variance', '[1;1;1]', ...                      % [-] unit variance
         'VectorParams1D', 'off', ...                    % [-] keep the [3x1] column shape
         'Seed', noiseSpecs{i,2}, ...                    % [-] per-axis seeds
         'SampleTime', 'SC.dt'});                        % [s] held for one step
end

%% ===== MATLAB Function blocks =====
% The time input is called 'tsim' inside the wrappers because 't' is a
% Stateflow keyword (absolute time); the line itself is still named 't'.
% addMatlabFcnBlock(mdl, B, blkName, libFcn, inNames, paramNames, outNames, callArgs, pos)
B = addMatlabFcnBlock(mdl, B, 'RefGen', 'referenceAttitude', ...
    {'tsim'}, {'SC'}, {'q_r', 'w_r', 'wdot_r'}, ...
    {'tsim', 'SC'}, [cx(2) 20 cx(2)+bw 20+bh]);

B = addMatlabFcnBlock(mdl, B, 'Dist', 'disturbanceTorques', ...
    {'tsim', 'q'}, {'SC'}, {'tau_d', 'tau_gg', 'tau_aero', 'tau_srp', 'tau_mag'}, ...
    {'tsim', 'q', 'SC'}, [cx(3) 150 cx(3)+bw 150+bh+40]);

B = addMatlabFcnBlock(mdl, B, 'Sensor', 'sensorModel', ...
    {'q', 'w', 'b', 'n_att', 'n_gyro', 'n_bias'}, {'SC'}, {'q_m', 'w_m', 'b_dot'}, ...
    {'q', 'w', 'b', 'n_att', 'n_gyro', 'n_bias', 'SC'}, [cx(3) 300 cx(3)+bw 300+bh+60]);

B = addMatlabFcnBlock(mdl, B, 'Ctrl', 'controllerSelect', ...
    {'q_m', 'w_m', 'q_r', 'w_r', 'wdot_r', 'h_w', 'theta_hat'}, ...
    {'SC', 'RobustGains', 'AdaptiveGains', 'BaselineGains', 'CONTROLLER_SELECT'}, ...
    {'tau_cmd', 'theta_hat_dot', 's'}, ...
    {'CONTROLLER_SELECT', 'q_m', 'w_m', 'q_r', 'w_r', 'wdot_r', 'h_w', 'theta_hat', ...
     'SC', 'RobustGains', 'AdaptiveGains', 'BaselineGains'}, ...
    [cx(4) 250 cx(4)+bw+20 250+bh+100]);

B = addMatlabFcnBlock(mdl, B, 'RW', 'reactionWheelModel', ...
    {'tau_cmd', 'h_w'}, {'SC'}, {'h_w_dot', 'tau_rw', 'sat_flags'}, ...
    {'tau_cmd', 'h_w', 'SC'}, [cx(5) 280 cx(5)+bw 280+bh]);

B = addMatlabFcnBlock(mdl, B, 'Dyn', 'rigidBodyDerivatives', ...
    {'q_raw', 'w', 'tau_rw', 'tau_d', 'h_w'}, {'SC'}, {'qdot_raw', 'wdot'}, ...
    {'q_raw', 'w', 'tau_rw', 'tau_d', 'h_w', 'SC'}, [cx(6) 450 cx(6)+bw 450+bh+40]);

B = addMatlabFcnBlock(mdl, B, 'QNorm', 'quatNormalizeState', ...
    {'q_raw'}, {}, {'q', 'qnorm'}, ...
    {'q_raw'}, [cx(7)+150 450 cx(7)+150+bw 450+bh]);

B = addMatlabFcnBlock(mdl, B, 'TrueErr', 'trueErrors', ...
    {'q', 'w', 'q_r', 'w_r'}, {'SC'}, {'q_e', 'w_e', 'att_err_deg'}, ...
    {'q', 'w', 'q_r', 'w_r', 'SC'}, [cx(6) 20 cx(6)+bw 20+bh+20]);

%% ===== Integrators =====
intPath = 'simulink/Continuous/Integrator';              % library path
B = addSimpleBlock(mdl, B, 'IntQ', intPath, {'u'}, {'y'}, [cx(7) 460 cx(7)+40 500], ...
    {'InitialCondition', 'SC.q0'});                      % [-] raw quaternion state
B = addSimpleBlock(mdl, B, 'IntW', intPath, {'u'}, {'y'}, [cx(7) 540 cx(7)+40 580], ...
    {'InitialCondition', 'SC.w0'});                      % [rad/s] body rate state
B = addSimpleBlock(mdl, B, 'IntHw', intPath, {'u'}, {'y'}, [cx(6) 300 cx(6)+40 340], ...
    {'InitialCondition', 'SC.h_w0', ...                  % [N*m*s] wheel momentum
     'LimitOutput', 'on', ...
     'UpperSaturationLimit', 'SC.h_max', ...             % [N*m*s]
     'LowerSaturationLimit', '-SC.h_max'});              % [N*m*s]
B = addSimpleBlock(mdl, B, 'IntTheta', intPath, {'u'}, {'y'}, [cx(5) 420 cx(5)+40 460], ...
    {'InitialCondition', 'AdaptiveGains.theta_hat0', ... % [kg*m^2] inertia estimate
     'LimitOutput', 'on', ...
     'UpperSaturationLimit', 'AdaptiveGains.theta_max', ... % [kg*m^2]
     'LowerSaturationLimit', 'AdaptiveGains.theta_min'});   % [kg*m^2]
B = addSimpleBlock(mdl, B, 'IntBias', intPath, {'u'}, {'y'}, [cx(4) 500 cx(4)+40 540], ...
    {'InitialCondition', '[0;0;0]'});                    % [rad/s] gyro bias state

%% ===== Terminators for outputs that are logged but not consumed =====
termSpecs = { 'Term_tau_gg',    [cx(4) 130];             % Dist out2
              'Term_tau_aero',  [cx(4) 160];             % Dist out3
              'Term_tau_srp',   [cx(4) 190];             % Dist out4
              'Term_tau_mag',   [cx(4) 220];             % Dist out5
              'Term_s',         [cx(5) 520];             % Ctrl out3
              'Term_sat_flags', [cx(6) 380];             % RW out3
              'Term_qnorm',     [cx(7)+350 500];         % QNorm out2
              'Term_q_e',       [cx(7) 20];              % TrueErr out1
              'Term_w_e',       [cx(7) 60];              % TrueErr out2
              'Term_att_err',   [cx(7) 100] };           % TrueErr out3
for i = 1:size(termSpecs, 1)
    p = termSpecs{i,2};                                  % [px] top-left corner
    B = addSimpleBlock(mdl, B, termSpecs{i,1}, 'simulink/Sinks/Terminator', ...
        {'u'}, {}, [p(1) p(2) p(1)+20 p(2)+20], {});
end

%% ===== Wiring =====
% W: {srcBlock, srcOutName, dstBlock, dstInName}. The same table is used to
% re-verify (and if needed repair) the connectivity after grouping (D12).
W = { ...
      % --- time
      'Clock',    't',   'RefGen', 'tsim';
      'Clock',    't',   'Dist',   'tsim';
      % --- reference
      'RefGen',   'q_r',    'Ctrl',    'q_r';
      'RefGen',   'q_r',    'TrueErr', 'q_r';
      'RefGen',   'w_r',    'Ctrl',    'w_r';
      'RefGen',   'w_r',    'TrueErr', 'w_r';
      'RefGen',   'wdot_r', 'Ctrl',    'wdot_r';
      % --- plant states
      'IntQ',     'y',      'QNorm',   'q_raw';
      'IntQ',     'y',      'Dyn',     'q_raw';
      'QNorm',    'q',      'Sensor',  'q';
      'QNorm',    'q',      'Dist',    'q';
      'QNorm',    'q',      'TrueErr', 'q';
      'QNorm',    'qnorm',  'Term_qnorm', 'u';
      'IntW',     'y',      'Sensor',  'w';
      'IntW',     'y',      'Dyn',     'w';
      'IntW',     'y',      'TrueErr', 'w';
      % --- sensors
      'IntBias',  'y',      'Sensor',  'b';
      'RandAtt',  'n',      'Sensor',  'n_att';
      'RandGyro', 'n',      'Sensor',  'n_gyro';
      'RandBias', 'n',      'Sensor',  'n_bias';
      'Sensor',   'b_dot',  'IntBias', 'u';
      'Sensor',   'q_m',    'Ctrl',    'q_m';
      'Sensor',   'w_m',    'Ctrl',    'w_m';
      % --- controller
      'IntHw',    'y',      'Ctrl',    'h_w';
      'IntHw',    'y',      'RW',      'h_w';
      'IntHw',    'y',      'Dyn',     'h_w';
      'IntTheta', 'y',      'Ctrl',    'theta_hat';
      'Ctrl',     'tau_cmd',       'RW',       'tau_cmd';
      'Ctrl',     'theta_hat_dot', 'IntTheta', 'u';
      'Ctrl',     's',             'Term_s',   'u';
      % --- actuator
      'RW',       'h_w_dot',   'IntHw',          'u';
      'RW',       'tau_rw',    'Dyn',            'tau_rw';
      'RW',       'sat_flags', 'Term_sat_flags', 'u';
      % --- disturbances
      'Dist',     'tau_d',    'Dyn',           'tau_d';
      'Dist',     'tau_gg',   'Term_tau_gg',   'u';
      'Dist',     'tau_aero', 'Term_tau_aero', 'u';
      'Dist',     'tau_srp',  'Term_tau_srp',  'u';
      'Dist',     'tau_mag',  'Term_tau_mag',  'u';
      % --- rigid-body dynamics
      'Dyn',      'qdot_raw', 'IntQ', 'u';
      'Dyn',      'wdot',     'IntW', 'u';
      % --- true errors
      'TrueErr',  'q_e',         'Term_q_e',     'u';
      'TrueErr',  'w_e',         'Term_w_e',     'u';
      'TrueErr',  'att_err_deg', 'Term_att_err', 'u';
    };
for i = 1:size(W, 1)
    wire(mdl, B, W{i,1}, W{i,2}, W{i,3}, W{i,4});
end

%% ===== Signal names and logging (names = res fields, spec sections 6/7) =====
% logSpecs: {sourceBlock, sourceOutName, loggedSignalName}
logSpecs = { 'QNorm',   'q',           'q';
             'QNorm',   'qnorm',       'qnorm';
             'IntW',    'y',           'w';
             'RefGen',  'q_r',         'q_r';
             'RefGen',  'w_r',         'w_r';
             'TrueErr', 'q_e',         'q_e';
             'TrueErr', 'w_e',         'w_e';
             'TrueErr', 'att_err_deg', 'att_err_deg';
             'Ctrl',    's',           's';
             'Ctrl',    'tau_cmd',     'tau_cmd';
             'RW',      'tau_rw',      'tau_rw';
             'RW',      'sat_flags',   'sat_flags';
             'IntHw',   'y',           'h_w';
             'IntTheta','y',           'theta_hat';
             'Dist',    'tau_d',       'tau_d';
             'Dist',    'tau_gg',      'tau_gg';
             'Dist',    'tau_aero',    'tau_aero';
             'Dist',    'tau_srp',     'tau_srp';
             'Dist',    'tau_mag',     'tau_mag' };
for i = 1:size(logSpecs, 1)
    logSignal(B, logSpecs{i,1}, logSpecs{i,2}, logSpecs{i,3});
end
% Descriptive (non-logged) names on internal lines, for readability only.
nameOnly = { 'Clock',    't',             't';
             'RefGen',   'wdot_r',        'wdot_r';
             'IntQ',     'y',             'q_raw';
             'IntBias',  'y',             'b';
             'Sensor',   'q_m',           'q_m';
             'Sensor',   'w_m',           'w_m';
             'Sensor',   'b_dot',         'b_dot';
             'Ctrl',     'theta_hat_dot', 'theta_hat_dot';
             'RW',       'h_w_dot',       'h_w_dot';
             'Dyn',      'qdot_raw',      'qdot_raw';
             'Dyn',      'wdot',          'wdot' };
for i = 1:size(nameOnly, 1)
    nameLine(B, nameOnly{i,1}, nameOnly{i,2}, nameOnly{i,3});
end

%% ===== Connectivity check (flat model) =====
checkAllPortsConnected(B);

%% ===== Group into subsystems (spec section 7) =====
groups = { 'RefGenerator',          {'Clock', 'RefGen'};
           'Sensors',               {'RandAtt', 'RandGyro', 'RandBias', 'Sensor', 'IntBias'};
           'ControllerSelect',      {'Ctrl', 'IntTheta', 'Term_s'};
           'ReactionWheelActuator', {'RW', 'IntHw', 'Term_sat_flags'};
           'DisturbanceTorques',    {'Dist', 'Term_tau_gg', 'Term_tau_aero', 'Term_tau_srp', 'Term_tau_mag'};
           'RigidBodyDynamics3DOF', {'Dyn', 'IntQ', 'IntW', 'QNorm', 'Term_qnorm'};
           'TrueErrors',            {'TrueErr', 'Term_q_e', 'Term_w_e', 'Term_att_err'} };
for i = 1:size(groups, 1)
    groupBlocks(mdl, groups{i,2}, groups{i,1});
    for k = 1:numel(groups{i,2})                         % update registry paths
        B.(groups{i,2}{k}).path = [mdl '/' groups{i,1} '/' groups{i,2}{k}];
    end
end

%% ===== Verify every wire across subsystem boundaries, repair if needed (D12) =====
% In R2026a, createSubsystem was seen to re-route TrueErr input w_r to the
% q_r signal (both come from RefGenerator and both lines branch). Each wire
% in W is therefore traced through the Inport/Outport boundaries back to its
% leaf source and compared with the intended source; mismatches are re-wired
% through a dedicated Inport/Outport pair and then checked again.
if injectMiswire
    injectWireFault(mdl, B);                             % test hook: reproduce the R2026a mis-wire
end
nFixed = 0;                                              % [-] number of repaired wires
for i = 1:size(W, 1)
    if ~wireIsCorrect(B, W(i,:))
        repairWire(mdl, B, W(i,:));
        nFixed = nFixed + 1;
        fprintf('build_ADCS_model: repaired wire %s.%s -> %s.%s after grouping\n', W{i,:});
    end
end
for i = 1:size(W, 1)
    if ~wireIsCorrect(B, W(i,:))
        error('build_ADCS_model:wiring', 'Wire %s.%s -> %s.%s is wrong after grouping and repair.', W{i,:});
    end
end
fprintf('build_ADCS_model: %d wires verified after grouping (%d repaired)\n', size(W, 1), nFixed);
info.nWires    = size(W, 1);
info.nRepaired = nFixed;
checkNoOpenPorts(mdl);                                   % every port at top level and in the groups is connected

%% ===== Post-grouping verification =====
% Every registered block must exist at its new path, all its ports must still
% be connected (now partly to Inport/Outport blocks), and logging is
% re-applied on the (moved) source ports so it cannot be lost in grouping.
blks = fieldnames(B);
for i = 1:numel(blks)
    if getSimulinkBlockHandle(B.(blks{i}).path) == -1
        error('build_ADCS_model:lostBlock', 'Block %s not found at %s after grouping.', ...
            blks{i}, B.(blks{i}).path);
    end
end
checkAllPortsConnected(B);
for i = 1:size(logSpecs, 1)
    logSignal(B, logSpecs{i,1}, logSpecs{i,2}, logSpecs{i,3});
end

%% ===== Layout and save =====
try
    Simulink.BlockDiagram.arrangeSystem(mdl);
    for i = 1:size(groups, 1)
        Simulink.BlockDiagram.arrangeSystem([mdl '/' groups{i,1}]);
    end
catch
    % arrangeSystem unavailable or failed - layout is cosmetic only
end
save_system(mdl, slxPath);
fprintf('build_ADCS_model: wrote %s and %s\n', slxPath, ddPath);

end

%% ===================================================================
%% ===== Local helper functions =====
%% ===================================================================

function B = addSimpleBlock(mdl, B, blkName, libPath, inNames, outNames, pos, params)
%ADDSIMPLEBLOCK Add a library block, set its parameters and register its port names.
%
% Inputs:
%   mdl      - char, model name                                        [-]
%   B        - struct, block registry (fields = block names)           [-]
%   blkName  - char, block name (valid MATLAB identifier)              [-]
%   libPath  - char, library block path, e.g. 'simulink/Sinks/Terminator' [-]
%   inNames  - cell 1xNi of char, names of input ports in port order    [-]
%   outNames - cell 1xNo of char, names of output ports in port order   [-]
%   pos      - double [1x4], block Position [left top right bottom]    [px]
%   params   - cell 1x2P, name/value pairs passed to set_param          [-]
% Outputs:
%   B        - struct, registry with B.(blkName).inNames/.outNames/.path added [-]
blkPath = [mdl '/' blkName];
add_block(libPath, blkPath, 'Position', pos);
if ~isempty(params)
    set_param(blkPath, params{:});
end
B.(blkName) = struct('inNames', {inNames}, 'outNames', {outNames}, 'path', blkPath);
end

function B = addMatlabFcnBlock(mdl, B, blkName, libFcn, inNames, paramNames, outNames, callArgs, pos)
%ADDMATLABFCNBLOCK Add a MATLAB Function block whose script is a thin wrapper around a library function.
%
% Inputs:
%   mdl        - char, model name                                        [-]
%   B          - struct, block registry                                  [-]
%   blkName    - char, block name                                        [-]
%   libFcn     - char, library function called by the wrapper            [-]
%   inNames    - cell 1xNi of char, signal inputs; ORDER = port order    [-]
%   paramNames - cell 1xNp of char, Stateflow Parameter-scope data       [-]
%   outNames   - cell 1xNo of char, outputs; ORDER = port order and the
%                output order of libFcn                                   [-]
%   callArgs   - cell, argument list of libFcn in its own order; each
%                entry must be a member of inNames or paramNames          [-]
%   pos        - double [1x4], block Position                            [px]
% Outputs:
%   B          - struct, registry with B.(blkName) added                 [-]

%% ===== Consistency checks on the name lists =====
allData = [inNames(:); paramNames(:)];
if numel(unique(allData)) ~= numel(allData)
    error('build_ADCS_model:dupName', 'Duplicate input/parameter name in block %s.', blkName);
end
for k = 1:numel(callArgs)
    if ~any(strcmp(callArgs{k}, allData))
        error('build_ADCS_model:badArg', 'Call argument %s of %s is not an input or parameter.', ...
            callArgs{k}, blkName);
    end
end
for k = 1:numel(allData)
    if ~any(strcmp(allData{k}, callArgs))
        error('build_ADCS_model:unusedArg', 'Input/parameter %s of %s is not passed to %s.', ...
            allData{k}, blkName, libFcn);
    end
end

%% ===== Generate the wrapper script =====
% Parameters come LAST in the fcn signature, so after they are switched to
% Scope 'Parameter' the remaining input ports keep numbers 1..Ni.
codeStr = makeWrapperScript(libFcn, inNames, paramNames, outNames, callArgs);

%% ===== Create the block and set its code via the Stateflow API =====
blkPath = [mdl '/' blkName];
add_block('simulink/User-Defined Functions/MATLAB Function', blkPath, 'Position', pos);
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blkPath);
if isempty(ch)
    error('build_ADCS_model:noChart', 'Could not find the Stateflow.EMChart of %s.', blkPath);
end
ch = ch(1);
ch.Script = codeStr;

%% ===== Remove stale data (e.g. the default u / y) not in the new signature =====
known   = [allData; outNames(:)];
allDefs = ch.find('-isa', 'Stateflow.Data');
for k = 1:numel(allDefs)
    if ~any(strcmp(allDefs(k).Name, known))
        delete(allDefs(k));
    end
end

%% ===== Mark parameter arguments =====
for k = 1:numel(paramNames)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', paramNames{k});
    if isempty(d)
        % Fallback: create the data object explicitly.
        d = Stateflow.Data(ch);
        d.Name = paramNames{k};
    end
    d(1).Scope = 'Parameter';
end

%% ===== Verify (and if needed enforce) port order =====
enforcePortOrder(ch, blkPath, inNames, 'Input');
enforcePortOrder(ch, blkPath, outNames, 'Output');

%% ===== Explicit sizes and types for every port (deviation D12) =====
% With inherited (-1) sizes, Simulink sometimes has to analyse a block before
% the sizes of its inputs have propagated around the feedback loops. It then
% analyses the body with guessed sizes and reports spurious errors (seen in
% R2026a: "Incorrect dimensions for matrix multiplication" in trueErrors).
% Fixing every port size and type removes that dependence on propagation order.
sigNames = [inNames(:); outNames(:)];
for k = 1:numel(sigNames)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', sigNames{k});
    d(1).Props.Array.Size = portSize(sigNames{k});       % e.g. '[4 1]'
    d(1).DataType = 'double';
end

B.(blkName) = struct('inNames', {inNames}, 'outNames', {outNames}, 'path', blkPath);
end

function sz = portSize(name)
%PORTSIZE Fixed size of a MATLAB Function block port, looked up by its signal name.
%
% Inputs:
%   name - char, port/signal name used in the wrappers                   [-]
% Outputs:
%   sz   - char, Stateflow size string, e.g. '[4 1]'                     [-]
switch name
    case {'tsim', 'qnorm', 'att_err_deg'}
        sz = '[1 1]';                                    % scalars
    case {'q', 'q_raw', 'qdot_raw', 'q_r', 'q_m', 'q_e'}
        sz = '[4 1]';                                    % quaternions and their rates
    case {'w', 'wdot', 'w_r', 'wdot_r', 'w_m', 'w_e', 's', 'b', 'b_dot', ...
          'n_att', 'n_gyro', 'n_bias', 'h_w', 'h_w_dot', 'tau_cmd', 'tau_rw', ...
          'tau_d', 'tau_gg', 'tau_aero', 'tau_srp', 'tau_mag'}
        sz = '[3 1]';                                    % body-axis 3-vectors
    case {'theta_hat', 'theta_hat_dot', 'sat_flags'}
        sz = '[6 1]';                                    % inertia parameters / 3 torque + 3 momentum flags
    otherwise
        error('build_ADCS_model:noSize', 'No fixed port size defined for signal %s.', name);
end
end

function codeStr = makeWrapperScript(libFcn, inNames, paramNames, outNames, callArgs)
%MAKEWRAPPERSCRIPT Build the MATLAB Function block code that forwards to a library function.
%
% Inputs:
%   libFcn     - char, library function name                             [-]
%   inNames    - cell of char, signal inputs in port order               [-]
%   paramNames - cell of char, parameter names                           [-]
%   outNames   - cell of char, outputs in port order                     [-]
%   callArgs   - cell of char, arguments of libFcn in its order          [-]
% Outputs:
%   codeStr    - char [1xL], script text (lines separated by char(10))   [-]
%
% Signal inputs are forwarded as name(:) so that 1-D Simulink vectors (e.g.
% from Random Number blocks) always reach the library as column vectors.

%% ===== Signature and call arguments =====
sigList = [inNames(:)', paramNames(:)'];              % fcn signature: signals first, then parameters
args = cell(1, numel(callArgs));
for k = 1:numel(callArgs)
    if any(strcmp(callArgs{k}, inNames))
        args{k} = [callArgs{k} '(:)'];
    else
        args{k} = callArgs{k};
    end
end

%% ===== Assemble the script text =====
outList = ['[' strjoin(outNames, ', ') ']'];
lines = { ['function ' outList ' = fcn(' strjoin(sigList, ', ') ')'], ...
          ['%#codegen'], ...
          ['% Auto-generated by build_ADCS_model.m - do not edit by hand.'], ...
          ['% Thin wrapper: all numerics live in ' libFcn '.m (src/).'], ...
          [outList ' = ' libFcn '(' strjoin(args, ', ') ');'] };
codeStr = strjoin(lines, char(10));
end

function enforcePortOrder(ch, blkPath, names, scope)
%ENFORCEPORTORDER Check that Stateflow data of a given scope have Port = position in NAMES.
%
% Inputs:
%   ch      - Stateflow.EMChart, chart of the MATLAB Function block      [-]
%   blkPath - char, block path (for messages)                            [-]
%   names   - cell of char, expected order of the ports                  [-]
%   scope   - char, 'Input' or 'Output'                                  [-]
% Outputs:
%   (none; errors if the order cannot be established)

%% ===== Assign port numbers =====
for k = 1:numel(names)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', names{k}, 'Scope', scope);
    if isempty(d)
        error('build_ADCS_model:missingData', '%s: no %s data named %s after setting the script.', ...
            blkPath, scope, names{k});
    end
    if d(1).Port ~= k
        d(1).Port = k;                                   % Stateflow renumbers the others
    end
end

%% ===== Verify the final order =====
for k = 1:numel(names)
    d = ch.find('-isa', 'Stateflow.Data', 'Name', names{k}, 'Scope', scope);
    if d(1).Port ~= k
        error('build_ADCS_model:portOrder', '%s: %s %s is on port %d, expected %d.', ...
            blkPath, scope, names{k}, d(1).Port, k);
    end
end
end

function wire(mdl, B, src, srcOut, dst, dstIn)
%WIRE Connect a named output port to a named input port (port numbers from the registry).
%
% Inputs:
%   mdl    - char, model name                                            [-]
%   B      - struct, block registry                                      [-]
%   src    - char, source block name                                     [-]
%   srcOut - char, output-port name of the source block                  [-]
%   dst    - char, destination block name                                [-]
%   dstIn  - char, input-port name of the destination block              [-]
% Outputs:
%   (none; adds a line or a branch to mdl)
sp = portIndex(B, src, srcOut, 'outNames');
dp = portIndex(B, dst, dstIn, 'inNames');
add_line(mdl, sprintf('%s/%d', src, sp), sprintf('%s/%d', dst, dp), 'autorouting', 'on');
end

function idx = portIndex(B, blk, portName, listField)
%PORTINDEX Look up a port number by name in the block registry.
%
% Inputs:
%   B         - struct, block registry                                   [-]
%   blk       - char, block name                                         [-]
%   portName  - char, port name                                          [-]
%   listField - char, 'inNames' or 'outNames'                            [-]
% Outputs:
%   idx       - double [1], 1-based port number                          [-]
if ~isfield(B, blk)
    error('build_ADCS_model:noBlock', 'Block %s is not registered.', blk);
end
idx = find(strcmp(B.(blk).(listField), portName));
if numel(idx) ~= 1
    error('build_ADCS_model:noPort', 'Block %s has no unique %s entry ''%s''.', blk, listField, portName);
end
end

function logSignal(B, blk, outName, sigName)
%LOGSIGNAL Name the line leaving an output port and enable signal logging on that port.
%
% Inputs:
%   B       - struct, block registry (with current block paths)          [-]
%   blk     - char, source block name                                    [-]
%   outName - char, output-port name in the registry                     [-]
%   sigName - char, signal/logging name (a res field name)               [-]
% Outputs:
%   (none)
ph = nameLine(B, blk, outName, sigName);
set_param(ph, 'DataLogging', 'on');
try
    % Pin the logged element name to sigName regardless of line names
    % (createSubsystem may split the named line into inner/outer segments).
    set_param(ph, 'DataLoggingNameMode', 'Custom');
    set_param(ph, 'DataLoggingName', sigName);
catch
    % fall back to the line name (SignalName mode)
end
end

function ph = nameLine(B, blk, outName, sigName)
%NAMELINE Set the name of the line connected to a named output port.
%
% Inputs:
%   B       - struct, block registry (with current block paths)          [-]
%   blk     - char, source block name                                    [-]
%   outName - char, output-port name in the registry                     [-]
%   sigName - char, line name                                            [-]
% Outputs:
%   ph      - double [1], handle of the output port                      [-]
k   = portIndex(B, blk, outName, 'outNames');
phs = get_param(B.(blk).path, 'PortHandles');
ph  = phs.Outport(k);
lh  = get_param(ph, 'Line');
if lh == -1
    error('build_ADCS_model:noLine', 'Output %s of %s is not connected.', outName, blk);
end
set_param(lh, 'Name', sigName);
end

function checkAllPortsConnected(B)
%CHECKALLPORTSCONNECTED Error if any registered block has an unconnected input or output port.
%
% Inputs:
%   B   - struct, block registry (with current block paths)              [-]
% Outputs:
%   (none; errors listing every unconnected port)

%% ===== Collect port-count and connection problems =====
blks = fieldnames(B);
bad  = {};
for i = 1:numel(blks)
    phs = get_param(B.(blks{i}).path, 'PortHandles');
    nIn = numel(B.(blks{i}).inNames);
    nOut = numel(B.(blks{i}).outNames);
    if numel(phs.Inport) ~= nIn || numel(phs.Outport) ~= nOut
        bad{end+1} = sprintf('%s: has %d/%d ports, registry expects %d/%d', blks{i}, ...
            numel(phs.Inport), numel(phs.Outport), nIn, nOut); %#ok<AGROW>
    end
    for k = 1:numel(phs.Inport)
        if get_param(phs.Inport(k), 'Line') == -1
            bad{end+1} = sprintf('%s/in%d', blks{i}, k); %#ok<AGROW>
        end
    end
    for k = 1:numel(phs.Outport)
        if get_param(phs.Outport(k), 'Line') == -1
            bad{end+1} = sprintf('%s/out%d', blks{i}, k); %#ok<AGROW>
        end
    end
end

%% ===== Report =====
if ~isempty(bad)
    error('build_ADCS_model:unconnected', 'Port problems: %s', strjoin(bad, '; '));
end
end

function groupBlocks(mdl, blkNames, subName)
%GROUPBLOCKS Group top-level blocks into a new subsystem and rename it.
%
% Inputs:
%   mdl      - char, model name                                          [-]
%   blkNames - cell of char, names of top-level blocks to group          [-]
%   subName  - char, name for the created subsystem                      [-]
% Outputs:
%   (none)

%% ===== Create the subsystem =====
before = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'SubSystem');
h = zeros(1, numel(blkNames));
for k = 1:numel(blkNames)
    h(k) = get_param([mdl '/' blkNames{k}], 'Handle');
end
Simulink.BlockDiagram.createSubsystem(h);

%% ===== Identify and rename it =====
after  = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'SubSystem');
newSub = setdiff(after, before);
if numel(newSub) ~= 1
    error('build_ADCS_model:group', 'Could not identify the subsystem created for %s.', subName);
end
set_param(newSub{1}, 'Name', subName);
end

function tf = wireIsCorrect(B, w)
%WIREISCORRECT True if the destination port of a wire is fed by the intended source port.
%
% Inputs:
%   B  - struct, block registry (with post-grouping paths)               [-]
%   w  - cell [1x4], {srcBlock, srcOutName, dstBlock, dstInName}          [-]
% Outputs:
%   tf - logical [1], true if the traced leaf source matches             [-]
[sb, sp] = traceLeafSource(B.(w{3}).path, portIndex(B, w{3}, w{4}, 'inNames'));
tf = sb ~= -1 && strcmp(getfullname(sb), B.(w{1}).path) && ...
     sp == portIndex(B, w{1}, w{2}, 'outNames');
end

function [sb, sp] = traceLeafSource(blkPath, inPort)
%TRACELEAFSOURCE Follow the line into an input port back through subsystem boundaries.
%
% Inputs:
%   blkPath - char, destination block path                               [-]
%   inPort  - double [1], 1-based input port number                      [-]
% Outputs:
%   sb      - double [1], handle of the leaf source block (-1 if open)   [-]
%   sp      - double [1], 1-based output port number on sb               [-]

%% ===== Start at the line entering the destination port =====
sb = -1;  sp = 0;
phs = get_param(blkPath, 'PortHandles');
ln  = get_param(phs.Inport(inPort), 'Line');

%% ===== Walk backwards across Inport / Outport boundaries =====
for guard = 1:20                                         % [-] max boundary crossings
    if ln == -1, return; end
    srcPort = get_param(ln, 'SrcPortHandle');
    if srcPort == -1, return; end
    blk = get_param(get_param(srcPort, 'Parent'), 'Handle');
    pn  = get_param(srcPort, 'PortNumber');
    bt  = get_param(blk, 'BlockType');
    parent = get_param(blk, 'Parent');
    if strcmp(bt, 'Inport') && ~strcmp(parent, bdroot(parent))
        % crossing up: the subsystem input port with the same number
        php = get_param(parent, 'PortHandles');
        ln  = get_param(php.Inport(str2double(get_param(blk, 'Port'))), 'Line');
    elseif strcmp(bt, 'SubSystem') && ~strcmp(get_param(blk, 'SFBlockType'), 'MATLAB Function')
        % crossing down: the Outport block with the same number inside
        ob  = find_system(blk, 'LookUnderMasks', 'all', 'SearchDepth', 1, ...
                          'BlockType', 'Outport', 'Port', num2str(pn));
        if isempty(ob), return; end                      % open boundary -> reported as wrong
        obp = get_param(ob(1), 'PortHandles');
        ln  = get_param(obp.Inport(1), 'Line');
    else
        %% ===== Leaf block reached =====
        sb = blk;  sp = pn;
        return;
    end
end
end

function repairWire(mdl, B, w)
%REPAIRWIRE Re-route one wire so its destination port is fed by the intended source port, leaving no orphans.
%
% Inputs:
%   mdl - char, model name                                               [-]
%   B   - struct, block registry (with post-grouping paths)              [-]
%   w   - cell [1x4], {srcBlock, srcOutName, dstBlock, dstInName}         [-]
% Outputs:
%   (none; modifies the model)
%
% Strategy (critic findings F1/F4):
%   * source and destination in the same subsystem -> re-draw the inner line;
%   * the destination is fed by its own subsystem Inport (no other users) ->
%     re-point the TOP-LEVEL line into that Inport to the correct source
%     subsystem output (no new blocks, no orphans);
%   * the Inport is shared with other (correct) destinations -> detach only
%     this destination and feed it from a new dedicated Inport.
% A source-subsystem output left without any line is closed with a Terminator.

%% ===== Ports and containing subsystems =====
sp  = portIndex(B, w{1}, w{2}, 'outNames');              % [-] source output port
dp  = portIndex(B, w{3}, w{4}, 'inNames');               % [-] destination input port
srcSys  = get_param(B.(w{1}).path, 'Parent');            % [-] subsystem holding the source
dstSys  = get_param(B.(w{3}).path, 'Parent');            % [-] subsystem holding the destination
srcName = get_param(B.(w{1}).path, 'Name');
dstName = get_param(B.(w{3}).path, 'Name');
dstPH   = get_param(B.(w{3}).path, 'PortHandles');
ln      = get_param(dstPH.Inport(dp), 'Line');           % line currently entering the destination port

%% ===== Same subsystem: re-draw the inner line =====
if strcmp(srcSys, dstSys)
    if ln ~= -1
        s = get_param(ln, 'SrcPortHandle');
        delete_line(dstSys, sprintf('%s/%d', get_param(get_param(s, 'Parent'), 'Name'), ...
                    get_param(s, 'PortNumber')), sprintf('%s/%d', dstName, dp));
    end
    add_line(dstSys, sprintf('%s/%d', srcName, sp), sprintf('%s/%d', dstName, dp), 'autorouting', 'on');
    return;
end

%% ===== Source side: reuse or create a subsystem output carrying the source port =====
oPort = sourceSubsystemPort(srcSys, B.(w{1}).path, srcName, sp, w{2});

%% ===== Destination side: which Inport feeds the destination now? =====
inBlk = -1;                                              % [-] feeding Inport block handle
if ln ~= -1
    s = get_param(ln, 'SrcPortHandle');
    if s ~= -1 && strcmp(get_param(get_param(s, 'Parent'), 'BlockType'), 'Inport')
        inBlk = get_param(get_param(s, 'Parent'), 'Handle');
    end
end
nUsers = 0;                                              % [-] destinations fed by that Inport
if inBlk ~= -1
    pc = get_param(inBlk, 'PortConnectivity');
    nUsers = numel(pc(end).DstBlock);
end
srcSysName = get_param(srcSys, 'Name');
dstSysName = get_param(dstSys, 'Name');

if inBlk ~= -1 && nUsers == 1
    %% ===== Case A: dedicated Inport -> re-point the top-level line =====
    xPort = str2double(get_param(inBlk, 'Port'));        % [-] subsystem input number
    sysPH = get_param(dstSys, 'PortHandles');
    lTop  = get_param(sysPH.Inport(xPort), 'Line');
    if lTop ~= -1
        oldSrc = get_param(lTop, 'SrcPortHandle');
        delete_line(mdl, sprintf('%s/%d', get_param(get_param(oldSrc, 'Parent'), 'Name'), ...
                    get_param(oldSrc, 'PortNumber')), sprintf('%s/%d', dstSysName, xPort));
        closeIfOpen(mdl, oldSrc);
    end
    add_line(mdl, sprintf('%s/%d', srcSysName, oPort), sprintf('%s/%d', dstSysName, xPort), ...
             'autorouting', 'on');
else
    %% ===== Case B: shared (or no) Inport -> dedicated new Inport =====
    if ln ~= -1
        s = get_param(ln, 'SrcPortHandle');
        delete_line(dstSys, sprintf('%s/%d', get_param(get_param(s, 'Parent'), 'Name'), ...
                    get_param(s, 'PortNumber')), sprintf('%s/%d', dstName, dp));
    end
    ib = add_block('simulink/Sources/In1', [dstSys '/' w{4} '_fix'], 'MakeNameUnique', 'on');
    add_line(dstSys, [get_param(ib, 'Name') '/1'], sprintf('%s/%d', dstName, dp), 'autorouting', 'on');
    add_line(mdl, sprintf('%s/%d', srcSysName, oPort), ...
             sprintf('%s/%d', dstSysName, str2double(get_param(ib, 'Port'))), 'autorouting', 'on');
end
end

function oPort = sourceSubsystemPort(srcSys, srcPath, srcName, sp, tag)
%SOURCESUBSYSTEMPORT Number of the subsystem output that carries a given inner source port (created if absent).
%
% Inputs:
%   srcSys  - char, path of the subsystem holding the source block       [-]
%   srcPath - char, full path of the source block                        [-]
%   srcName - char, name of the source block                             [-]
%   sp      - double [1], 1-based output port of the source block        [-]
%   tag     - char, base name for a new Outport block                    [-]
% Outputs:
%   oPort   - double [1], 1-based output port number of srcSys           [-]

%% ===== Reuse an existing Outport fed by the source port =====
oPort = 0;
obs = find_system(srcSys, 'LookUnderMasks', 'all', 'SearchDepth', 1, 'BlockType', 'Outport');
for k = 1:numel(obs)
    obp = get_param(obs{k}, 'PortHandles');
    l2  = get_param(obp.Inport(1), 'Line');
    if l2 ~= -1
        s2 = get_param(l2, 'SrcPortHandle');
        if s2 ~= -1 && strcmp(get_param(s2, 'Parent'), srcPath) && get_param(s2, 'PortNumber') == sp
            oPort = str2double(get_param(obs{k}, 'Port'));
            return;
        end
    end
end

%% ===== Otherwise add one (new highest port number: existing lines keep their numbers) =====
ob = add_block('simulink/Sinks/Out1', [srcSys '/' tag '_fix'], 'MakeNameUnique', 'on');
add_line(srcSys, sprintf('%s/%d', srcName, sp), [get_param(ob, 'Name') '/1'], 'autorouting', 'on');
oPort = str2double(get_param(ob, 'Port'));
end

function closeIfOpen(mdl, portH)
%CLOSEIFOPEN Terminate a top-level subsystem output port that no longer drives any line.
%
% Inputs:
%   mdl   - char, model name                                             [-]
%   portH - double [1], output-port handle to check                      [-]
% Outputs:
%   (none; adds a Terminator if the port is open)
if get_param(portH, 'Line') == -1
    tb = add_block('simulink/Sinks/Terminator', [mdl '/Term_fix'], 'MakeNameUnique', 'on');
    add_line(mdl, sprintf('%s/%d', get_param(get_param(portH, 'Parent'), 'Name'), ...
             get_param(portH, 'PortNumber')), [get_param(tb, 'Name') '/1'], 'autorouting', 'on');
end
end

function injectWireFault(mdl, B)
%INJECTWIREFAULT Test hook: re-route TrueErr input w_r to the q_r signal at top level (the R2026a symptom).
%
% Inputs:
%   mdl - char, model name                                               [-]
%   B   - struct, block registry (with post-grouping paths)              [-]
% Outputs:
%   (none; modifies the model)

%% ===== Subsystem inputs feeding TrueErr q_r and w_r =====
dstSys  = get_param(B.TrueErr.path, 'Parent');
dstSysName = get_param(dstSys, 'Name');
ph  = get_param(B.TrueErr.path, 'PortHandles');
xq  = localInportNumber(ph.Inport(portIndex(B, 'TrueErr', 'q_r', 'inNames')));
xw  = localInportNumber(ph.Inport(portIndex(B, 'TrueErr', 'w_r', 'inNames')));
sysPH = get_param(dstSys, 'PortHandles');

%% ===== Replace the top-level source of the w_r input by the q_r source =====
srcQ = get_param(get_param(sysPH.Inport(xq), 'Line'), 'SrcPortHandle');
srcW = get_param(get_param(sysPH.Inport(xw), 'Line'), 'SrcPortHandle');
delete_line(mdl, sprintf('%s/%d', get_param(get_param(srcW, 'Parent'), 'Name'), get_param(srcW, 'PortNumber')), ...
            sprintf('%s/%d', dstSysName, xw));
add_line(mdl, sprintf('%s/%d', get_param(get_param(srcQ, 'Parent'), 'Name'), get_param(srcQ, 'PortNumber')), ...
         sprintf('%s/%d', dstSysName, xw), 'autorouting', 'on');
fprintf('build_ADCS_model: TEST HOOK injected mis-wire TrueErr.w_r <- q_r\n');
end

function x = localInportNumber(inPortH)
%LOCALINPORTNUMBER Port number of the Inport block that drives a given block input port.
%
% Inputs:
%   inPortH - double [1], input-port handle of a block inside a subsystem [-]
% Outputs:
%   x       - double [1], the 'Port' number of the driving Inport block   [-]
s = get_param(get_param(inPortH, 'Line'), 'SrcPortHandle');
x = str2double(get_param(get_param(s, 'Parent'), 'Port'));
end

function checkNoOpenPorts(mdl)
%CHECKNOOPENPORTS Error if any block at the top level or inside the grouped subsystems has an open port.
%
% Inputs:
%   mdl - char, model name                                               [-]
% Outputs:
%   (none; errors listing every unconnected port, critic finding F3)

%% ===== Scan depth 1 (subsystems) and depth 2 (their contents) =====
blks = find_system(mdl, 'SearchDepth', 2, 'Type', 'Block');
bad  = {};
for i = 1:numel(blks)
    phs = get_param(blks{i}, 'PortHandles');
    for k = 1:numel(phs.Inport)
        if get_param(phs.Inport(k), 'Line') == -1
            bad{end+1} = sprintf('%s/in%d', blks{i}, k); %#ok<AGROW>
        end
    end
    for k = 1:numel(phs.Outport)
        if get_param(phs.Outport(k), 'Line') == -1
            bad{end+1} = sprintf('%s/out%d', blks{i}, k); %#ok<AGROW>
        end
    end
end
if ~isempty(bad)
    error('build_ADCS_model:openPorts', 'Unconnected ports after grouping: %s', strjoin(bad, ', '));
end
end
