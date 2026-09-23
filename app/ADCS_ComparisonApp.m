classdef ADCS_ComparisonApp < matlab.apps.AppBase
%ADCS_COMPARISONAPP GUI to configure, run and compare the three ADCS attitude controllers.
%
%   App Designer-structured programmatic app (matlab.apps.AppBase with
%   registerApp). It edits the user-level configuration struct cfg
%   (docs/DESIGN_SPEC.md section 4.1), runs
%   runSimulation(cfg, ctrl_ids, engine), evaluates computeMetrics /
%   evaluatePassFail and plots the results with plotResultsOnAxes. It holds
%   no physics or metric code of its own: everything is delegated to the
%   library functions of DESIGN_SPEC.md section 5.
%
%   Controllers: 1 Robust SMC, 2 Adaptive SMC (online inertia estimation),
%   3 PD Benchmark. Scenario: LEO chaser rendezvousing with the ISS,
%   3-axis rotational dynamics only.
%
%   Usage:
%       app = ADCS_ComparisonApp();   % or: app = launchADCSApp();
%       delete(app);                  % closes the window
%
%   The widget <-> cfg mapping lives ONLY in readWidgetsToCfg (widgets ->
%   cfg) and its mirror writeCfgToWidgets (cfg -> widgets).
%
%   Target: MATLAB R2021a or newer (uifigure components only).

    %% ===== Public component properties =====
    properties (Access = public)
        % ----- Figure-level -----
        UIFigure                matlab.ui.Figure                 % main application window [-]
        MainGrid                matlab.ui.container.GridLayout   % root grid: tab group + status bar [-]
        TabGroup                matlab.ui.container.TabGroup     % top-level tab group (4 tabs) [-]
        ScenarioTab             matlab.ui.container.Tab          % tab 1: scenario and initial conditions [-]
        GainsTab                matlab.ui.container.Tab          % tab 2: controller gains [-]
        RunTab                  matlab.ui.container.Tab          % tab 3: run controls and log [-]
        ResultsTab              matlab.ui.container.Tab          % tab 4: plots and metrics [-]
        StatusLabel             matlab.ui.control.Label          % status bar text at the bottom [-]

        % ----- Tab 1: layout -----
        ScenarioGrid            matlab.ui.container.GridLayout   % 3x3 grid holding the scenario panels [-]

        % ----- Tab 1: Preset panel -----
        PresetPanel             matlab.ui.container.Panel        % panel 'Preset' [-]
        PresetDropDown          matlab.ui.control.DropDown       % scenario preset; Items = display names, ItemsData = function names [-]
        LoadPresetButton        matlab.ui.control.Button         % loads cfg = feval(preset function) [-]
        SaveConfigButton        matlab.ui.control.Button         % saves the current cfg to a .mat file (saveConfig) [-]
        LoadConfigButton        matlab.ui.control.Button         % loads a cfg from a .mat file (loadConfig) [-]
        ConfigNameLabel         matlab.ui.control.Label          % shows cfg.name / cfg.description [-]

        % ----- Tab 1: Attitude error panel -----
        AttitudePanel           matlab.ui.container.Panel        % panel 'Attitude error' [-]
        AxisEditFields                                           % 1x3 NumericEditField, rotation axis of the initial error (normalised on read) [-]
        AngleEditField          matlab.ui.control.NumericEditField % initial attitude error angle [deg]
        AngleSlider             matlab.ui.control.Slider         % slider linked to AngleEditField, 0..180 [deg]
        RateEditFields                                           % 1x3 NumericEditField, initial body-rate error w.r.t. reference [deg/s]

        % ----- Tab 1: Reference / orbit panel -----
        ReferencePanel          matlab.ui.container.Panel        % panel 'Reference / orbit' [-]
        TargetModeDropDown      matlab.ui.control.DropDown       % reference mode: 0 inertial, 1 LVLH, 2 docking approach, 3 attitude scan [-]
        AltitudeEditField       matlab.ui.control.NumericEditField % circular-orbit altitude [km]
        InclinationEditField    matlab.ui.control.NumericEditField % orbit inclination [deg]
        R0EditField             matlab.ui.control.NumericEditField % initial chaser-to-ISS range [m]
        RfEditField             matlab.ui.control.NumericEditField % final (asymptotic) range [m]
        TcloseEditField         matlab.ui.control.NumericEditField % closing time constant of the range profile [s]
        Yoff0EditField          matlab.ui.control.NumericEditField % initial cross-track offset y0 in LVLH [m]
        Zoff0EditField          matlab.ui.control.NumericEditField % initial radial offset z0 in LVLH [m]
        ScanAmpEditFields                                        % 1x3 NumericEditField, mode-3 scan amplitudes [deg]
        ScanFreqEditFields                                       % 1x3 NumericEditField, mode-3 scan frequencies [rad/s]

        % ----- Tab 1: Inertia panel -----
        InertiaPanel            matlab.ui.container.Panel        % panel 'Inertia' [-]
        JnomTable               matlab.ui.control.Table          % editable 3x3 nominal inertia J_nom (kept symmetric) [kg*m^2]
        UncSlider               matlab.ui.control.Slider         % inertia uncertainty 0..50 [%]
        UncEditField            matlab.ui.control.NumericEditField % inertia uncertainty, linked to UncSlider [%]
        UncModeDropDown         matlab.ui.control.DropDown       % uncertainty mode: 1 random per-parameter, 2 uniform scale [-]
        SeedEditField           matlab.ui.control.NumericEditField % RNG seed for the random perturbation [-]
        JtrueTable              matlab.ui.control.Table          % read-only 3x3 derived J_true from buildSimParams [kg*m^2]
        JtrueNoteLabel          matlab.ui.control.Label          % info.notes from buildSimParams (J_true validity) [-]

        % ----- Tab 1: Disturbances panel -----
        DisturbancePanel        matlab.ui.container.Panel        % panel 'Disturbances' [-]
        DistCheckBoxes                                           % 1x4 CheckBox: GG, Aero, SRP, Mag enable flags [-]
        DistScaleEditFields                                      % 1x4 NumericEditField: GG, Aero, SRP, Mag magnitude scale [-]

        % ----- Tab 1: Actuator & sensors panel -----
        ActuatorPanel           matlab.ui.container.Panel        % panel 'Actuator & sensors' [-]
        TauMaxEditField         matlab.ui.control.NumericEditField % per-wheel torque limit [N*m]
        HMaxEditField           matlab.ui.control.NumericEditField % per-wheel momentum limit [N*m*s]
        NoiseCheckBox           matlab.ui.control.CheckBox       % sensor-noise enable flag [-]
        AttNoiseEditField       matlab.ui.control.NumericEditField % attitude-sensor noise std [deg]
        GyroNoiseEditField      matlab.ui.control.NumericEditField % gyro white-noise std [deg/s]

        % ----- Tab 1: Simulation panel -----
        SimulationPanel         matlab.ui.container.Panel        % panel 'Simulation' [-]
        TfinalEditField         matlab.ui.control.NumericEditField % simulation stop time [s]
        DtEditField             matlab.ui.control.NumericEditField % fixed integration step [s]
        SettleThreshEditField   matlab.ui.control.NumericEditField % settling-time threshold on attitude error [deg]

        % ----- Tab 2: Controller gains -----
        GainsTabGroup           matlab.ui.container.TabGroup     % nested tab group with one sub-tab per controller [-]
        RobustGainsTab          matlab.ui.container.Tab          % sub-tab 'Robust SMC' [-]
        AdaptiveGainsTab        matlab.ui.container.Tab          % sub-tab 'Adaptive SMC' [-]
        BaselineGainsTab        matlab.ui.container.Tab          % sub-tab 'PD Benchmark' [-]
        AblationCheckBox        matlab.ui.control.CheckBox       % keep adaptive Lambda, K, eta, phi = robust (clean ablation) [-]
        WnEditField             matlab.ui.control.NumericEditField % PD design natural frequency wn [rad/s]
        ZetaEditField           matlab.ui.control.NumericEditField % PD design damping ratio zeta [-]
        ComputePDButton         matlab.ui.control.Button         % computes Kp/Kd from wn, zeta (pdGainsFromBandwidth) [-]
        LoadDefaultsButtons                                      % 1x3 Button, 'Load defaults' per sub-tab (UserData = ctrl key) [-]

        % ----- Tab 3: Run -----
        RunModeButtonGroup      matlab.ui.container.ButtonGroup  % run mode selector [-]
        SingleRadioButton       matlab.ui.control.RadioButton    % run one controller [-]
        CompareAllRadioButton   matlab.ui.control.RadioButton    % run all three controllers [-]
        ControllerDropDown      matlab.ui.control.DropDown       % controller for single runs, ItemsData = ctrl_id 1..3 [-]
        EngineDropDown          matlab.ui.control.DropDown       % engine: 'auto' | 'simulink' | 'reference' [-]
        RunButton               matlab.ui.control.Button         % starts the simulation [-]
        SimulinkLabel           matlab.ui.control.Label          % shows isSimulinkAvailable() [-]
        LogTextArea             matlab.ui.control.TextArea       % read-only run log [-]

        % ----- Tab 4: Results -----
        ResultsModeDropDown     matlab.ui.control.DropDown       % 'overlay' | 'single' display mode [-]
        ResultsCtrlDropDown     matlab.ui.control.DropDown       % which stored result to show in single mode (index into Results) [-]
        ExportFigureButton      matlab.ui.control.Button         % exports the app window / axes to an image [-]
        ExportCSVButton         matlab.ui.control.Button         % exports the metrics table to CSV [-]
        ResultAxes                                               % 1x6 UIAxes, plots in the order of PlotQuantities [-]
        MetricsTable            matlab.ui.control.Table          % metrics table from metricsTable() [mixed]
    end

    %% ===== Private state properties =====
    properties (Access = private)
        Cfg             % struct, user-level config cfg (DESIGN_SPEC 4.1), mirrors the widgets [mixed]
        RunCfg          % struct, snapshot of Cfg used for the last run (plots/metrics use it) [mixed]
        Results         % 1xK struct array of res (DESIGN_SPEC 6) from the last run [mixed]
        Metrics         % 1xK struct array of M (DESIGN_SPEC 5.1) from computeMetrics [mixed]
        PassFlags       % 1xK logical, pass/fail per result from evaluatePassFail [-]
        GainWidgets     % struct: GainWidgets.<ctrl>.<field> = 1xn uitable handle [-]
        ProjectRoot     % char, absolute path of the project root folder [-]
        WriteNotes = '' % char, notes from the last writeCfgToWidgets (clamped values etc.) [-]
    end

    %% ===== Private constants =====
    properties (Constant, Access = private)
        GainCtrls      = {'robust', 'adaptive', 'baseline'}              % cfg.gains keys, order = ctrl_id [-]
        GainTabTitles  = {'Robust SMC', 'Adaptive SMC', 'PD Benchmark'}  % sub-tab titles [-]
        AblationFields = {'lambda_diag', 'K_diag', 'eta', 'phi'}         % gain fields shared robust -> adaptive [-]
        DistNames      = {'Gravity gradient', 'Aerodynamic', 'SRP', 'Magnetic'} % dist_enable/dist_scale order [-]
        PlotQuantities = {'att_err', 'w_e', 'tau_rw', 'h_w', 'qnorm', 'theta_hat'} % plotResultsOnAxes quantities [-]
        PlotTitles     = {'Attitude error [deg]', 'Rate error |ω_e| [deg/s]', ...
                          'Wheel torque |τ| [N·m]', 'Wheel momentum max|h| [N·m·s]', ...
                          'Quaternion norm deviation [-]', 'Adaptive θ̂/θ_true [-]'} % axes titles [-]
    end

    %% ===== Callbacks: Scenario tab =====
    methods (Access = private)

        function startupFcn(app)
        %STARTUPFCN Initialise the config from initDefaults and populate data-driven widgets.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none) - app.Cfg and all widgets are initialised in place

            %% ===== Default configuration =====
            try
                app.Cfg = initDefaults();
            catch ME
                uialert(app.UIFigure, ['initDefaults() failed: ' ME.message], 'Startup error');
                setStatus(app, ['Startup error: ' ME.message]);
                return;
            end

            %% ===== Data-driven dropdowns =====
            try
                list = listScenarios();
                setDropDownItems(app, app.PresetDropDown, list(:, 2).', list(:, 1).');
            catch ME
                setStatus(app, ['listScenarios() failed: ' ME.message]);
            end
            names = controllerNames();
            setDropDownItems(app, app.ControllerDropDown, names, 1:numel(names));
            app.ControllerDropDown.Value = 1;

            %% ===== Simulink availability =====
            try
                hasSL = isSimulinkAvailable();
            catch
                hasSL = false;
            end
            if hasSL
                app.SimulinkLabel.Text = 'Simulink: available (Auto uses the Simulink harness)';
                app.SimulinkLabel.FontColor = [0.0 0.45 0.0];
            else
                app.SimulinkLabel.Text = 'Simulink: NOT available (Auto uses the MATLAB reference engine)';
                app.SimulinkLabel.FontColor = [0.70 0.35 0.0];
            end

            %% ===== Push config into widgets =====
            writeCfgToWidgets(app);
            refreshJTrue(app);
            refreshResults(app);
            appendLog(app, 'App started with initDefaults().');
            setStatus(app, 'Ready. Configure the scenario, then press Run on the Run tab.');
        end

        function LoadPresetButtonPushed(app, event)
        %LOADPRESETBUTTONPUSHED Load the selected scenario preset (cfg = feval(fname)) into all widgets.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData, unused               [-]
        % Outputs:
        %   (none) - app.Cfg replaced, widgets refreshed

            %% ===== Selected preset =====
            fname = app.PresetDropDown.Value;
            if isempty(fname) || ~ischar(fname)
                setStatus(app, 'No preset selected.');
                return;
            end

            %% ===== Evaluate preset and refresh widgets =====
            try
                cfg = feval(fname);
                app.Cfg = cfg;
                writeCfgToWidgets(app);
                refreshJTrue(app);
                appendLog(app, ['Loaded preset ' fname '.']);
                setStatus(app, ['Loaded preset: ' fname noteSuffix(app)]);
            catch ME
                uialert(app.UIFigure, ME.message, 'Preset load failed');
                setStatus(app, ['Preset load failed: ' ME.message]);
            end
        end

        function SaveConfigButtonPushed(app, event)
        %SAVECONFIGBUTTONPUSHED Save the current widget state as a cfg .mat file via saveConfig.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData, unused               [-]
        % Outputs:
        %   (none) - file written by saveConfig(file, cfg)

            %% ===== Read and validate =====
            readWidgetsToCfg(app);
            [ok, msg] = validateCfg(app);
            if ~ok
                uialert(app.UIFigure, msg, 'Invalid input');
                setStatus(app, ['Save aborted: ' strrep(msg, newline, ' ')]);
                return;
            end

            %% ===== Choose file and save =====
            d = fullfile(app.ProjectRoot, 'config', 'saved');           % default folder for saved configs [-]
            if ~isfolder(d), try, mkdir(d); catch, end, end              % create it if missing (best effort)
            defFile = fullfile(d, 'ADCS_config.mat');                    % default file name [-]
            [f, p] = uiputfile({'*.mat', 'ADCS configuration (*.mat)'}, 'Save configuration', defFile);
            figure(app.UIFigure);
            if isequal(f, 0)
                setStatus(app, 'Save cancelled.');
                return;
            end
            file = fullfile(p, f);
            try
                saveConfig(file, app.Cfg);
                appendLog(app, ['Configuration saved to ' file]);
                setStatus(app, ['Saved: ' file]);
            catch ME
                uialert(app.UIFigure, ME.message, 'Save failed');
                setStatus(app, ['Save failed: ' ME.message]);
            end
        end

        function LoadConfigButtonPushed(app, event)
        %LOADCONFIGBUTTONPUSHED Load a cfg .mat file via loadConfig and refresh all widgets.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData, unused               [-]
        % Outputs:
        %   (none) - app.Cfg replaced, widgets refreshed

            %% ===== Choose file =====
            defDir = fullfile(app.ProjectRoot, 'config', 'saved');      % default folder for saved configs [-]
            if ~isfolder(defDir), try, mkdir(defDir); catch, end, end    % create it if missing (best effort)
            [f, p] = uigetfile({'*.mat', 'ADCS configuration (*.mat)'}, 'Load configuration', [defDir filesep]);
            figure(app.UIFigure);
            if isequal(f, 0)
                setStatus(app, 'Load cancelled.');
                return;
            end

            %% ===== Load and refresh widgets =====
            file = fullfile(p, f);
            try
                app.Cfg = loadConfig(file);
                writeCfgToWidgets(app);
                refreshJTrue(app);
                appendLog(app, ['Configuration loaded from ' file]);
                setStatus(app, ['Loaded: ' file noteSuffix(app)]);
            catch ME
                uialert(app.UIFigure, ME.message, 'Load failed');
                setStatus(app, ['Load failed: ' ME.message]);
            end
        end

        function AngleSliderValueChanging(app, event)
        %ANGLESLIDERVALUECHANGING Live-update the angle field while the slider is dragged.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangingData, event.Value = angle [deg]
        % Outputs:
        %   (none)

            app.AngleEditField.Value = event.Value;
        end

        function AngleSliderValueChanged(app, event)
        %ANGLESLIDERVALUECHANGED Copy the final slider value into the angle field.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            app.AngleEditField.Value = app.AngleSlider.Value;
        end

        function AngleEditFieldValueChanged(app, event)
        %ANGLEEDITFIELDVALUECHANGED Copy the typed angle into the slider.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            app.AngleSlider.Value = app.AngleEditField.Value;
        end

        function UncSliderValueChanging(app, event)
        %UNCSLIDERVALUECHANGING Live-update the uncertainty field while the slider is dragged.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangingData, event.Value = pct   [%]
        % Outputs:
        %   (none)

            app.UncEditField.Value = event.Value;
        end

        function UncSliderValueChanged(app, event)
        %UNCSLIDERVALUECHANGED Copy the final slider value into the field and refresh J_true.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            app.UncEditField.Value = app.UncSlider.Value;
            refreshJTrue(app);
        end

        function UncEditFieldValueChanged(app, event)
        %UNCEDITFIELDVALUECHANGED Copy the typed uncertainty into the slider and refresh J_true.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            app.UncSlider.Value = app.UncEditField.Value;
            refreshJTrue(app);
        end

        function InertiaParamChanged(app, event)
        %INERTIAPARAMCHANGED Refresh J_true after the uncertainty mode or seed changed.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            refreshJTrue(app);
        end

        function JnomTableCellEdit(app, event)
        %JNOMTABLECELLEDIT Keep J_nom symmetric by mirroring the edited element, then refresh J_true.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.CellEditData: Indices [1x2] (row, col),
        %           NewData [1] new value, PreviousData [1] old value        [kg*m^2]
        % Outputs:
        %   (none) - JnomTable.Data updated in place

            %% ===== Validate the entry =====
            r = event.Indices(1);
            c = event.Indices(2);
            D = double(app.JnomTable.Data);
            v = event.NewData;
            if ~(isnumeric(v) && isscalar(v) && isfinite(v))
                D(r, c) = event.PreviousData;
                app.JnomTable.Data = D;
                setStatus(app, 'J_nom: enter a finite number (edit reverted).');
                return;
            end

            %% ===== Mirror and refresh =====
            D(r, c) = v;
            D(c, r) = v;
            app.JnomTable.Data = D;
            refreshJTrue(app);
            if r ~= c
                setStatus(app, sprintf(['J_nom(%d,%d) mirrored to J_nom(%d,%d). Gains are not ' ...
                    'recomputed automatically: use ''Load defaults'' on the gain tabs if needed.'], r, c, c, r));
            end
        end

        function NoiseCheckBoxValueChanged(app, event)
        %NOISECHECKBOXVALUECHANGED Enable or disable the noise-std fields with the noise checkbox.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            updateNoiseEnable(app);
        end

    end

    %% ===== Callbacks: Gains tab =====
    methods (Access = private)

        function GainTableCellEdit(app, event)
        %GAINTABLECELLEDIT Validate an edited gain element and keep the ablation mirror in sync.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.CellEditData; event.Source.UserData has
        %           ctrl (char), field (char), n (double); NewData [1]     [gain units]
        % Outputs:
        %   (none)

            %% ===== Validate the edited cell =====
            src  = event.Source;
            info = src.UserData;
            v    = event.NewData;
            if ~(isnumeric(v) && isscalar(v) && isfinite(v))
                D = double(src.Data);
                D(event.Indices(1), event.Indices(2)) = event.PreviousData;
                src.Data = D;
                setStatus(app, sprintf('%s.%s: enter a finite number (edit reverted).', info.ctrl, info.field));
                return;
            end

            %% ===== Propagate ablation mirror and report =====
            if strcmp(info.ctrl, 'robust') && any(strcmp(info.field, app.AblationFields))
                syncAblationWidgets(app);
            end
            setStatus(app, sprintf('Gain %s.%s updated.', info.ctrl, info.field));
        end

        function LoadGainDefaultsButtonPushed(app, event)
        %LOADGAINDEFAULTSBUTTONPUSHED Replace one controller's gains by its *_default(J_nom).
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData; event.Source.UserData
        %           = 'robust' | 'adaptive' | 'baseline'                   [-]
        % Outputs:
        %   (none) - gain widgets and app.Cfg.gains.<ctrl> updated

            ctrl = event.Source.UserData;
            try
                %% ===== Evaluate the default gains at the current J_nom =====
                J = readJnomFromTable(app);
                switch ctrl
                    case 'robust'
                        g = robust_default(J);
                    case 'adaptive'
                        g = adaptive_default(J);
                    case 'baseline'
                        g = baseline_default(J);
                    otherwise
                        error('ADCS_ComparisonApp:ctrl', 'Unknown controller key %s.', ctrl);
                end

                %% ===== Store, write widgets and report =====
                app.Cfg.gains.(ctrl) = g;
                msgs = writeGainWidgets(app, ctrl, g);
                syncAblationWidgets(app);
                if isempty(msgs)
                    setStatus(app, sprintf('Loaded %s_default(J_nom) gains.', ctrl));
                else
                    setStatus(app, strjoin(msgs, ' | '));
                end
            catch ME
                uialert(app.UIFigure, ME.message, 'Load defaults failed');
                setStatus(app, ['Load defaults failed: ' ME.message]);
            end
        end

        function ComputePDButtonPushed(app, event)
        %COMPUTEPDBUTTONPUSHED Fill Kp_diag/Kd_diag from pdGainsFromBandwidth(J_nom, wn, zeta).
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData, unused               [-]
        % Outputs:
        %   (none) - baseline Kp_diag [N*m] and Kd_diag [N*m*s] tables updated

            try
                %% ===== Design the PD gains =====
                J    = readJnomFromTable(app);
                wn   = app.WnEditField.Value;    % [rad/s]
                zeta = app.ZetaEditField.Value;  % [-]
                [Kp, Kd] = pdGainsFromBandwidth(J, wn, zeta);

                %% ===== Write the baseline gain tables =====
                W = app.GainWidgets.baseline;
                if isfield(W, 'Kp_diag')
                    W.Kp_diag.Data = reshape(double(Kp), 1, []);
                end
                if isfield(W, 'Kd_diag')
                    W.Kd_diag.Data = reshape(double(Kd), 1, []);
                end
                setStatus(app, sprintf('PD gains computed for wn = %.4g rad/s, zeta = %.3g.', wn, zeta));
            catch ME
                uialert(app.UIFigure, ME.message, 'PD gain computation failed');
                setStatus(app, ['PD gain computation failed: ' ME.message]);
            end
        end

        function AblationCheckBoxValueChanged(app, event)
        %ABLATIONCHECKBOXVALUECHANGED Apply or release the robust -> adaptive gain mirror.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            syncAblationWidgets(app);
            if app.AblationCheckBox.Value
                setStatus(app, 'Adaptive Lambda, K, eta, phi now mirror the Robust values.');
            else
                setStatus(app, 'Adaptive Lambda, K, eta, phi are now independent of Robust.');
            end
        end

    end

    %% ===== Callbacks: Run tab =====
    methods (Access = private)

        function RunModeSelectionChanged(app, event)
        %RUNMODESELECTIONCHANGED Enable the controller dropdown only for single-controller runs.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.SelectionChangedData, unused           [-]
        % Outputs:
        %   (none)

            app.ControllerDropDown.Enable = onOff(app, app.SingleRadioButton.Value);
        end

        function RunButtonPushed(app, event)
        %RUNBUTTONPUSHED Read widgets, run the simulation(s), compute metrics and show results.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData, unused               [-]
        % Outputs:
        %   (none) - app.Results, app.Metrics, app.PassFlags, app.RunCfg set

            %% ===== Lock the UI =====
            app.RunButton.Enable = 'off';
            drawnow;
            dlg = [];

            try
                %% ===== Read and validate inputs =====
                readWidgetsToCfg(app);
                [ok, msg] = validateCfg(app);
                if ~ok
                    uialert(app.UIFigure, msg, 'Invalid input');
                    appendLog(app, ['Run aborted: ' strrep(msg, newline, ' ')]);
                    setStatus(app, 'Run aborted: invalid input.');
                else
                    %% ===== Controller ids and engine =====
                    names = controllerNames();
                    if app.CompareAllRadioButton.Value
                        ids = 1:numel(names);
                    else
                        ids = app.ControllerDropDown.Value;
                    end
                    engine = app.EngineDropDown.Value;
                    runCfg = app.Cfg;

                    %% ===== Simulate =====
                    dlg = uiprogressdlg(app.UIFigure, 'Title', 'Simulation running', ...
                        'Message', sprintf('Running %s with engine ''%s'' ...', ...
                        strjoin(names(ids), ', '), engine), 'Indeterminate', 'on');
                    appendLog(app, sprintf('Run started: %s | engine=%s | t_final=%g s | dt=%g s', ...
                        strjoin(names(ids), ', '), engine, runCfg.scenario.t_final_s, runCfg.scenario.dt_s));
                    setStatus(app, 'Simulation running ...');
                    drawnow;
                    results = runSimulation(runCfg, ids, engine);

                    %% ===== Metrics and pass/fail =====
                    dlg.Message = 'Computing metrics ...';
                    drawnow;
                    K = numel(results);
                    passFlags = false(1, K);
                    metrics = [];
                    for k = 1:K
                        Mk = computeMetrics(results(k), runCfg);
                        [passK, reasons] = evaluatePassFail(Mk, runCfg.pass);
                        passFlags(k) = logical(passK);
                        if k == 1
                            metrics = Mk;
                        else
                            metrics(k) = Mk; %#ok<AGROW>
                        end
                        appendLog(app, formatResultLine(app, results(k), Mk, passFlags(k), reasons));
                    end

                    %% ===== Store and display =====
                    app.Results   = results;
                    app.Metrics   = metrics;
                    app.PassFlags = passFlags;
                    app.RunCfg    = runCfg;
                    updateResultsSelector(app);
                    dlg.Message = 'Plotting ...';
                    refreshResults(app);
                    close(dlg);
                    app.TabGroup.SelectedTab = app.ResultsTab;
                    setStatus(app, sprintf('Run finished: %d result(s), %d passed.', K, sum(passFlags)));
                end
            catch ME
                %% ===== Error path =====
                if ~isempty(dlg) && isvalid(dlg)
                    close(dlg);
                end
                appendLog(app, ['ERROR: ' ME.message]);
                if ~isempty(ME.stack)
                    appendLog(app, sprintf('   at %s (line %d)', ME.stack(1).name, ME.stack(1).line));
                end
                setStatus(app, ['Run failed: ' ME.message]);
                uialert(app.UIFigure, ME.message, 'Simulation failed');
            end

            %% ===== Unlock the UI =====
            app.RunButton.Enable = 'on';
            drawnow;
        end

    end

    %% ===== Callbacks: Results tab =====
    methods (Access = private)

        function ResultsModeDropDownValueChanged(app, event)
        %RESULTSMODEDROPDOWNVALUECHANGED Switch between overlay and single-run display.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            isSingle = strcmp(app.ResultsModeDropDown.Value, 'single');
            app.ResultsCtrlDropDown.Enable = onOff(app, isSingle && ~isempty(app.Results));
            refreshResults(app);
        end

        function ResultsCtrlDropDownValueChanged(app, event)
        %RESULTSCTRLDROPDOWNVALUECHANGED Re-plot after another stored result was selected.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ValueChangedData, unused               [-]
        % Outputs:
        %   (none)

            refreshResults(app);
        end

        function ExportFigureButtonPushed(app, event)
        %EXPORTFIGUREBUTTONPUSHED Export the app window (exportapp), falling back to per-axes exportgraphics.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData, unused               [-]
        % Outputs:
        %   (none) - image file(s) written

            %% ===== Choose file =====
            if isempty(app.Results)
                uialert(app.UIFigure, 'Run a simulation first.', 'Nothing to export');
                return;
            end
            defFile = fullfile(app.ProjectRoot, 'ADCS_results.png');
            [f, p] = uiputfile({'*.png', 'PNG image (*.png)'; '*.pdf', 'PDF (*.pdf)'; ...
                '*.jpg', 'JPEG image (*.jpg)'}, 'Export figure', defFile);
            figure(app.UIFigure);
            if isequal(f, 0)
                setStatus(app, 'Export cancelled.');
                return;
            end
            file = fullfile(p, f);

            %% ===== Export =====
            try
                exportapp(app.UIFigure, file);
                setStatus(app, ['Figure exported: ' file]);
                appendLog(app, ['Figure exported to ' file]);
            catch ME1
                try
                    [~, base, ext] = fileparts(file);
                    for i = 1:numel(app.ResultAxes)
                        fi = fullfile(p, [base '_' app.PlotQuantities{i} ext]);
                        exportgraphics(app.ResultAxes(i), fi);
                    end
                    setStatus(app, ['exportapp failed (' ME1.message '); exported each axes as ' ...
                        fullfile(p, [base '_<quantity>' ext])]);
                    appendLog(app, ['Axes exported individually to ' p]);
                catch ME2
                    uialert(app.UIFigure, ME2.message, 'Export failed');
                    setStatus(app, ['Export failed: ' ME2.message]);
                end
            end
        end

        function ExportCSVButtonPushed(app, event)
        %EXPORTCSVBUTTONPUSHED Export the metrics table to CSV via exportMetricsCSV.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   event - matlab.ui.eventdata.ButtonPushedData, unused               [-]
        % Outputs:
        %   (none) - CSV file written

            %% ===== Choose the output file =====
            if isempty(app.Results)
                uialert(app.UIFigure, 'Run a simulation first.', 'Nothing to export');
                return;
            end
            defFile = fullfile(app.ProjectRoot, 'ADCS_metrics.csv');
            [f, p] = uiputfile({'*.csv', 'CSV file (*.csv)'}, 'Export metrics', defFile);
            figure(app.UIFigure);
            if isequal(f, 0)
                setStatus(app, 'Export cancelled.');
                return;
            end

            %% ===== Build the table and write the CSV =====
            file = fullfile(p, f);
            try
                [header, rows] = metricsTable(app.Results, app.Metrics, app.PassFlags);
                exportMetricsCSV(file, header, rows);
                setStatus(app, ['Metrics exported: ' file]);
                appendLog(app, ['Metrics exported to ' file]);
            catch ME
                uialert(app.UIFigure, ME.message, 'Export failed');
                setStatus(app, ['Metrics export failed: ' ME.message]);
            end
        end

    end

    %% ===== Widget <-> cfg mapping (single source of truth) =====
    methods (Access = private)

        function readWidgetsToCfg(app)
        %READWIDGETSTOCFG Copy every widget value into app.Cfg (the ONLY widget -> cfg mapping).
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none) - app.Cfg updated in place; fields not exposed in the GUI
        %            (e.g. rho_kgm3, noise_seed) are kept unchanged

            cfg = app.Cfg;
            sc  = cfg.scenario;

            %% ===== Attitude error =====
            ax = [app.AxisEditFields.Value].';                      % [-] raw axis
            nrm = norm(ax);
            if nrm > 0
                ax = ax / nrm;                                      % [-] normalised axis
            end
            sc.att_err_axis      = ax;                              % [3x1] [-]
            sc.att_err_angle_deg = app.AngleEditField.Value;        % [deg]
            sc.rate_err0_degps   = [app.RateEditFields.Value].';    % [3x1] [deg/s]

            %% ===== Reference / orbit =====
            sc.target_mode    = app.TargetModeDropDown.Value;       % [-] 0|1|2|3
            sc.orbit_alt_km   = app.AltitudeEditField.Value;        % [km]
            sc.orbit_incl_deg = app.InclinationEditField.Value;     % [deg]
            sc.R0_m           = app.R0EditField.Value;              % [m]
            sc.Rf_m           = app.RfEditField.Value;              % [m]
            sc.T_close_s      = app.TcloseEditField.Value;          % [s]
            sc.y_off0_m       = app.Yoff0EditField.Value;           % [m]
            sc.z_off0_m       = app.Zoff0EditField.Value;           % [m]
            sc.scan_amp_deg    = [app.ScanAmpEditFields.Value].';  % [3x1] [deg]
            sc.scan_freq_radps = [app.ScanFreqEditFields.Value].'; % [3x1] [rad/s]

            %% ===== Inertia =====
            sc.J_nom      = readJnomFromTable(app);                 % [3x3] [kg*m^2]
            sc.J_unc_pct  = app.UncEditField.Value;                 % [%]
            sc.J_unc_mode = app.UncModeDropDown.Value;              % [-] 1|2
            sc.J_unc_seed = app.SeedEditField.Value;                % [-]

            %% ===== Disturbances =====
            sc.dist_enable = double([app.DistCheckBoxes.Value]);    % [1x4] [-] GG Aero SRP Mag
            sc.dist_scale  = [app.DistScaleEditFields.Value];       % [1x4] [-]

            %% ===== Actuator & sensors =====
            sc.wheel_tau_max_Nm     = app.TauMaxEditField.Value;    % [N*m]
            sc.wheel_h_max_Nms      = app.HMaxEditField.Value;      % [N*m*s]
            sc.noise_enable         = double(app.NoiseCheckBox.Value); % [-] 0|1
            sc.att_noise_std_deg    = app.AttNoiseEditField.Value;  % [deg]
            sc.gyro_noise_std_degps = app.GyroNoiseEditField.Value; % [deg/s]

            %% ===== Simulation =====
            sc.t_final_s = app.TfinalEditField.Value;               % [s]
            sc.dt_s      = app.DtEditField.Value;                   % [s]
            cfg.metrics.settle_thresh_deg = app.SettleThreshEditField.Value; % [deg]
            cfg.scenario = sc;

            %% ===== Gains (generated from gainMetadata) =====
            for c = 1:numel(app.GainCtrls)
                ctrl = app.GainCtrls{c};
                if isfield(cfg, 'gains') && isfield(cfg.gains, ctrl)
                    g = cfg.gains.(ctrl);
                else
                    g = struct();
                end
                flds = fieldnames(app.GainWidgets.(ctrl));
                for i = 1:numel(flds)
                    D = app.GainWidgets.(ctrl).(flds{i}).Data;
                    if iscell(D)
                        D = cell2mat(D);
                    end
                    g.(flds{i}) = double(reshape(D, [], 1));        % [nx1] [gain units]
                end
                cfg.gains.(ctrl) = g;
            end

            %% ===== Clean ablation: adaptive shares Lambda, K, eta, phi with robust =====
            if app.AblationCheckBox.Value
                for i = 1:numel(app.AblationFields)
                    f = app.AblationFields{i};
                    if isfield(cfg.gains.robust, f)
                        cfg.gains.adaptive.(f) = cfg.gains.robust.(f);
                    end
                end
            end

            app.Cfg = cfg;
        end

        function writeCfgToWidgets(app)
        %WRITECFGTOWIDGETS Copy app.Cfg into every widget (mirror of readWidgetsToCfg).
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none) - widgets updated; out-of-range values are clamped to the
        %            widget limits and reported in the status bar

            try
                cfg = app.Cfg;
                sc  = cfg.scenario;
                w   = {};   % collected warnings

                %% ===== Header =====
                nm = '';
                if isfield(cfg, 'name'), nm = cfg.name; end
                ds = '';
                if isfield(cfg, 'description'), ds = cfg.description; end
                app.ConfigNameLabel.Text = sprintf('Current: %s', nm);
                app.ConfigNameLabel.Tooltip = ds;

                %% ===== Attitude error =====
                for k = 1:3
                    w{end+1} = setNumericField(app, app.AxisEditFields(k), sc.att_err_axis(k), 'axis'); %#ok<AGROW>
                    w{end+1} = setNumericField(app, app.RateEditFields(k), sc.rate_err0_degps(k), 'rate_err0_degps'); %#ok<AGROW>
                end
                w{end+1} = setNumericField(app, app.AngleEditField, sc.att_err_angle_deg, 'att_err_angle_deg');
                app.AngleSlider.Value = app.AngleEditField.Value;

                %% ===== Reference / orbit =====
                w{end+1} = setDropDownValue(app, app.TargetModeDropDown, sc.target_mode, 'target_mode');
                w{end+1} = setNumericField(app, app.AltitudeEditField, sc.orbit_alt_km, 'orbit_alt_km');
                w{end+1} = setNumericField(app, app.InclinationEditField, sc.orbit_incl_deg, 'orbit_incl_deg');
                w{end+1} = setNumericField(app, app.R0EditField, sc.R0_m, 'R0_m');
                w{end+1} = setNumericField(app, app.RfEditField, sc.Rf_m, 'Rf_m');
                w{end+1} = setNumericField(app, app.TcloseEditField, sc.T_close_s, 'T_close_s');
                w{end+1} = setNumericField(app, app.Yoff0EditField, sc.y_off0_m, 'y_off0_m');
                w{end+1} = setNumericField(app, app.Zoff0EditField, sc.z_off0_m, 'z_off0_m');
                for k = 1:3
                    w{end+1} = setNumericField(app, app.ScanAmpEditFields(k), sc.scan_amp_deg(k), 'scan_amp_deg'); %#ok<AGROW>
                    w{end+1} = setNumericField(app, app.ScanFreqEditFields(k), sc.scan_freq_radps(k), 'scan_freq_radps'); %#ok<AGROW>
                end

                %% ===== Inertia =====
                app.JnomTable.Data = double(sc.J_nom);
                w{end+1} = setNumericField(app, app.UncEditField, sc.J_unc_pct, 'J_unc_pct');
                app.UncSlider.Value = app.UncEditField.Value;
                w{end+1} = setDropDownValue(app, app.UncModeDropDown, sc.J_unc_mode, 'J_unc_mode');
                w{end+1} = setNumericField(app, app.SeedEditField, sc.J_unc_seed, 'J_unc_seed');

                %% ===== Disturbances =====
                for k = 1:4
                    app.DistCheckBoxes(k).Value = logical(sc.dist_enable(k));
                    w{end+1} = setNumericField(app, app.DistScaleEditFields(k), sc.dist_scale(k), 'dist_scale'); %#ok<AGROW>
                end

                %% ===== Actuator & sensors =====
                w{end+1} = setNumericField(app, app.TauMaxEditField, sc.wheel_tau_max_Nm, 'wheel_tau_max_Nm');
                w{end+1} = setNumericField(app, app.HMaxEditField, sc.wheel_h_max_Nms, 'wheel_h_max_Nms');
                app.NoiseCheckBox.Value = logical(sc.noise_enable);
                w{end+1} = setNumericField(app, app.AttNoiseEditField, sc.att_noise_std_deg, 'att_noise_std_deg');
                w{end+1} = setNumericField(app, app.GyroNoiseEditField, sc.gyro_noise_std_degps, 'gyro_noise_std_degps');
                updateNoiseEnable(app);

                %% ===== Simulation =====
                w{end+1} = setNumericField(app, app.TfinalEditField, sc.t_final_s, 't_final_s');
                w{end+1} = setNumericField(app, app.DtEditField, sc.dt_s, 'dt_s');
                w{end+1} = setNumericField(app, app.SettleThreshEditField, cfg.metrics.settle_thresh_deg, 'settle_thresh_deg');

                %% ===== Gains =====
                for c = 1:numel(app.GainCtrls)
                    ctrl = app.GainCtrls{c};
                    if isfield(cfg.gains, ctrl)
                        w = [w, writeGainWidgets(app, ctrl, cfg.gains.(ctrl))]; %#ok<AGROW>
                    else
                        w{end+1} = sprintf('cfg.gains.%s missing', ctrl); %#ok<AGROW>
                    end
                end

                %% ===== Ablation checkbox reflects whether the config shares the gains =====
                same = true;
                for i = 1:numel(app.AblationFields)
                    f = app.AblationFields{i};
                    if isfield(cfg.gains.robust, f) && isfield(cfg.gains.adaptive, f)
                        same = same && isequal(double(cfg.gains.robust.(f)(:)), double(cfg.gains.adaptive.(f)(:)));
                    end
                end
                app.AblationCheckBox.Value = same;
                if ~same
                    w{end+1} = 'adaptive Lambda/K/eta/phi differ from robust: ablation lock switched off';
                end
                syncAblationWidgets(app);

                %% ===== Report =====
                w = w(~cellfun(@isempty, w));
                if isempty(w)
                    app.WriteNotes = '';
                else
                    app.WriteNotes = strjoin(w, ' | ');
                    appendLog(app, ['Config notes: ' app.WriteNotes]);
                    setStatus(app, ['Config loaded with notes: ' app.WriteNotes]);
                end
            catch ME
                app.WriteNotes = ['could not write config to widgets: ' ME.message];
                setStatus(app, ['Could not write config to widgets: ' ME.message]);
            end
        end

        function [ok, msg] = validateCfg(app)
        %VALIDATECFG Check the widget-derived app.Cfg for values that would break a run.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance (call readWidgetsToCfg first) [-]
        % Outputs:
        %   ok  - logical [1], true if every check passed                    [-]
        %   msg - char, newline-separated list of problems ('' if ok)        [-]

            sc = app.Cfg.scenario;
            m  = {};

            %% ===== Attitude error =====
            rawAxis = [app.AxisEditFields.Value];
            if all(rawAxis == 0)
                m{end+1} = 'The rotation axis must not be all zeros.';
            end
            if ~(sc.att_err_angle_deg >= 0 && sc.att_err_angle_deg <= 180)
                m{end+1} = 'The attitude error angle must lie in [0, 180] deg.';
            end

            %% ===== Time settings =====
            if ~(sc.dt_s > 0)
                m{end+1} = 'dt must be > 0 s.';
            end
            if ~(sc.t_final_s > sc.dt_s)
                m{end+1} = 't_final must be greater than dt.';
            end

            %% ===== Inertia =====
            J = sc.J_nom;
            if any(~isfinite(J(:)))
                m{end+1} = 'J_nom contains non-finite entries.';
            elseif min(eig(0.5*(J + J.'))) <= 0
                m{end+1} = 'J_nom must be positive definite.';
            end

            %% ===== Actuator =====
            if ~(sc.wheel_tau_max_Nm > 0) || ~(sc.wheel_h_max_Nms > 0)
                m{end+1} = 'Wheel torque and momentum limits must be > 0.';
            end

            %% ===== Gains =====
            for c = 1:numel(app.GainCtrls)
                ctrl = app.GainCtrls{c};
                g = app.Cfg.gains.(ctrl);
                flds = fieldnames(g);
                for i = 1:numel(flds)
                    v = g.(flds{i});
                    if isnumeric(v) && any(~isfinite(v(:)))
                        m{end+1} = sprintf('Gain %s.%s contains non-finite values.', ctrl, flds{i}); %#ok<AGROW>
                    end
                end
            end

            %% ===== Time grid consistency =====
            if sc.dt_s > 0 && sc.t_final_s > 0
                nStep = sc.t_final_s / sc.dt_s;                  % [-] number of fixed steps
                if abs(nStep - round(nStep)) > 1e-9 * max(1, abs(nStep))
                    m{end+1} = 't_final must be an integer multiple of dt.';
                end
            end

            %% ===== Gain sign and range checks =====
            smcCtrls = {'robust', 'adaptive'};                   % controllers with a boundary layer [-]
            for c = 1:numel(smcCtrls)
                ctrl = smcCtrls{c};
                g = app.Cfg.gains.(ctrl);
                if isfield(g, 'phi') && any(~(double(g.phi(:)) > 0))
                    m{end+1} = sprintf('Gain %s.phi must be > 0.', ctrl); %#ok<AGROW>
                end
                nonNeg = {'K_diag', 'eta', 'Gamma_diag'};        % gain fields that must be >= 0 [-]
                for i = 1:numel(nonNeg)
                    f = nonNeg{i};
                    if isfield(g, f) && any(~(double(g.(f)(:)) >= 0))
                        m{end+1} = sprintf('Gain %s.%s must be >= 0.', ctrl, f); %#ok<AGROW>
                    end
                end
            end
            ga = app.Cfg.gains.adaptive;
            if isfield(ga, 'proj_frac') && ~(all(double(ga.proj_frac(:)) > 0) && all(double(ga.proj_frac(:)) < 1))
                m{end+1} = 'Gain adaptive.proj_frac must lie in (0, 1).';
            end

            %% ===== Result =====
            ok  = isempty(m);
            msg = strjoin(m, newline);
        end

    end

    %% ===== Private helpers =====
    methods (Access = private)

        function J = readJnomFromTable(app)
        %READJNOMFROMTABLE Return the symmetrised J_nom from the editable table.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   J   - double [3x3], symmetric nominal inertia                    [kg*m^2]

            D = app.JnomTable.Data;
            if iscell(D)
                D = cell2mat(D);
            end
            D = double(D);
            J = 0.5 * (D + D.');
        end

        function refreshJTrue(app)
        %REFRESHJTRUE Read widgets, call buildSimParams and show the derived J_true.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none) - JtrueTable and JtrueNoteLabel updated; errors go to the status bar

            try
                %% ===== Derive J_true through buildSimParams =====
                readWidgetsToCfg(app);
                [SC, ~, ~, ~, info] = buildSimParams(app.Cfg);
                app.JtrueTable.Data = SC.J_true;

                %% ===== Show the validity note =====
                note = '';
                if isstruct(info) && isfield(info, 'notes') && ischar(info.notes)
                    note = info.notes;
                end
                app.JtrueNoteLabel.Text = note;
                if isstruct(info) && isfield(info, 'J_true_physical') && ~info.J_true_physical
                    app.JtrueNoteLabel.FontColor = [0.8 0.1 0.1];
                    setStatus(app, ['J_true warning: ' note]);
                else
                    app.JtrueNoteLabel.FontColor = [0.3 0.3 0.3];
                end
            catch ME
                %% ===== Failure: blank the table and report =====
                app.JtrueTable.Data = NaN(3);
                app.JtrueNoteLabel.Text = 'J_true could not be derived (see status bar).';
                app.JtrueNoteLabel.FontColor = [0.8 0.1 0.1];
                setStatus(app, ['buildSimParams failed: ' ME.message]);
            end
        end

        function msgs = writeGainWidgets(app, ctrl, g)
        %WRITEGAINWIDGETS Write one controller's gain struct into its generated uitables.
        %
        % Inputs:
        %   app  - ADCS_ComparisonApp, this app instance                       [-]
        %   ctrl - char, 'robust' | 'adaptive' | 'baseline'                  [-]
        %   g    - struct, user gain struct (DESIGN_SPEC 4.1)                [gain units]
        % Outputs:
        %   msgs - cell [1xm] of char, warnings (missing or wrong-size fields) [-]

            %% ===== Loop over the generated tables =====
            msgs = {};
            W = app.GainWidgets.(ctrl);
            flds = fieldnames(W);
            for i = 1:numel(flds)
                f   = flds{i};
                tbl = W.(f);
                n   = tbl.UserData.n;
                if ~isfield(g, f)
                    msgs{end+1} = sprintf('%s.%s missing (kept table value)', ctrl, f); %#ok<AGROW>
                    continue;
                end

                %% ===== Fit the value to the table width (pad or truncate) =====
                v  = double(g.(f)(:)).';
                vv = zeros(1, n);
                m  = min(n, numel(v));
                vv(1:m) = v(1:m);
                if numel(v) ~= n
                    msgs{end+1} = sprintf('%s.%s has %d elements, expected %d', ctrl, f, numel(v), n); %#ok<AGROW>
                end
                tbl.Data = vv;
            end
        end

        function syncAblationWidgets(app)
        %SYNCABLATIONWIDGETS Mirror robust Lambda, K, eta, phi into the adaptive tables when locked.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none) - adaptive tables updated and made read-only (or editable again)

            %% ===== Mirror or release each shared field =====
            on = app.AblationCheckBox.Value;                 % [-] true -> adaptive tables follow robust
            for i = 1:numel(app.AblationFields)
                f = app.AblationFields{i};
                if isfield(app.GainWidgets.robust, f) && isfield(app.GainWidgets.adaptive, f)
                    tA = app.GainWidgets.adaptive.(f);
                    tR = app.GainWidgets.robust.(f);
                    n  = tA.UserData.n;
                    if on
                        tA.Data = tR.Data;
                        tA.ColumnEditable = false(1, n);
                    else
                        tA.ColumnEditable = true(1, n);
                    end
                end
            end
        end

        function refreshResults(app)
        %REFRESHRESULTS Redraw the six result axes and the metrics table.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none)

            %% ===== Select the results to show =====
            if isempty(app.Results)
                for i = 1:numel(app.ResultAxes)
                    resetAxes(app, i);
                end
                app.MetricsTable.Data = {};
                return;
            end
            if strcmp(app.ResultsModeDropDown.Value, 'single')
                k = app.ResultsCtrlDropDown.Value;
                if ~isnumeric(k) || isempty(k) || k < 1 || k > numel(app.Results)
                    k = 1;
                end
                sel = app.Results(k);
            else
                sel = app.Results;
            end
            plotCfg = app.RunCfg;
            if isempty(plotCfg)
                plotCfg = app.Cfg;
            end

            %% ===== Axes =====
            errs = {};
            for i = 1:numel(app.ResultAxes)
                ax = app.ResultAxes(i);
                q  = app.PlotQuantities{i};
                resetAxes(app, i);
                try
                    if strcmp(q, 'theta_hat')
                        sub = sel([sel.ctrl_id] == 2);
                        if isempty(sub)
                            h = text(ax, 0, 0, 'Run the Adaptive controller to see inertia estimates', ...
                                'HorizontalAlignment', 'center');
                            h.Units = 'normalized';
                            h.Position = [0.5 0.5 0];
                        else
                            plotResultsOnAxes(ax, sub, q, plotCfg);
                        end
                    else
                        plotResultsOnAxes(ax, sel, q, plotCfg);
                    end
                catch ME
                    errs{end+1} = sprintf('%s: %s', q, ME.message); %#ok<AGROW>
                end
                title(ax, app.PlotTitles{i}, 'Interpreter', 'none');
            end

            %% ===== Metrics table =====
            try
                [header, rows] = metricsTable(app.Results, app.Metrics, app.PassFlags);
                app.MetricsTable.ColumnName = header;
                app.MetricsTable.Data = rows;
            catch ME
                errs{end+1} = ['metricsTable: ' ME.message];
            end
            if ~isempty(errs)
                setStatus(app, ['Plot problems: ' strjoin(errs, ' | ')]);
            end
            drawnow;
        end

        function resetAxes(app, i)
        %RESETAXES Clear result axes i (legend, children, limits) and restore its title.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        %   i   - double [1], index into ResultAxes / PlotQuantities           [-]
        % Outputs:
        %   (none)

            ax = app.ResultAxes(i);
            legend(ax, 'off');
            cla(ax);
            ax.XLimMode = 'auto';
            ax.YLimMode = 'auto';
            ax.YScale   = 'linear';
            title(ax, app.PlotTitles{i}, 'Interpreter', 'none');
        end

        function updateResultsSelector(app)
        %UPDATERESULTSSELECTOR Fill the single-run selector with the names of the stored results.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none)

            K = numel(app.Results);
            if K == 0
                setDropDownItems(app, app.ResultsCtrlDropDown, {'(no results)'}, 0);
                app.ResultsCtrlDropDown.Enable = 'off';
                return;
            end
            names = cell(1, K);
            for k = 1:K
                names{k} = app.Results(k).ctrl_name;
            end
            setDropDownItems(app, app.ResultsCtrlDropDown, names, 1:K);
            app.ResultsCtrlDropDown.Enable = onOff(app, strcmp(app.ResultsModeDropDown.Value, 'single'));
        end

        function line = formatResultLine(app, res, M, pass, reasons)
        %FORMATRESULTLINE Build one log line summarising a result and its metrics.
        %
        % Inputs:
        %   app     - ADCS_ComparisonApp, this app instance                    [-]
        %   res     - struct, one result (DESIGN_SPEC 6)                     [mixed]
        %   M       - struct, metrics of res (DESIGN_SPEC 5.1)               [mixed]
        %   pass    - logical [1], pass flag                                 [-]
        %   reasons - cell of char or char, failure reasons                  [-]
        % Outputs:
        %   line    - char, formatted log line                               [-]

            %% ===== Verdict =====
            if pass
                verdict = 'PASS';
            else
                verdict = 'FAIL';
            end

            %% ===== Compose the line (never throws) =====
            try
                if iscell(reasons)
                    rs = strjoin(reasons, '; ');
                elseif ischar(reasons)
                    rs = reasons;
                else
                    rs = '';
                end
                line = sprintf('%s [%s, %.2f s wall]: %s | settle %.1f s | ss err %.4g deg | peak tau %.4g N*m | peak h %.4g N*m*s', ...
                    res.ctrl_name, res.engine, res.wallclock_s, verdict, ...
                    getMetric(app, M, 'settle_time_s'), getMetric(app, M, 'ss_err_deg'), ...
                    getMetric(app, M, 'peak_torque_Nm'), getMetric(app, M, 'peak_h_Nms'));
                if ~isempty(rs)
                    line = [line ' | ' rs];
                end
            catch
                % Logging must never abort a successful run.
                line = sprintf('Result: %s (log formatting failed)', verdict);
            end
        end

        function v = getMetric(app, M, name)
        %GETMETRIC Return a scalar metric or NaN if the field is missing.
        %
        % Inputs:
        %   app  - ADCS_ComparisonApp, this app instance                       [-]
        %   M    - struct, metrics struct                                    [mixed]
        %   name - char, field name                                          [-]
        % Outputs:
        %   v    - double [1], metric value or NaN                           [metric units]

            v = NaN;
            if isfield(M, name) && isnumeric(M.(name)) && ~isempty(M.(name))
                v = double(M.(name)(1));
            end
        end

        function msg = setNumericField(app, h, v, name)
        %SETNUMERICFIELD Assign a value to a numeric edit field, clamping it into the field limits.
        %
        % Inputs:
        %   app  - ADCS_ComparisonApp, this app instance                       [-]
        %   h    - matlab.ui.control.NumericEditField, target field          [-]
        %   v    - double [1], value to show                                 [field units]
        %   name - char, cfg field name used in warnings                     [-]
        % Outputs:
        %   msg  - char, warning text ('' if the value was set unchanged)    [-]

            %% ===== Reject non-numeric and NaN values =====
            msg = '';
            if isempty(v) || (~isnumeric(v) && ~islogical(v))
                msg = sprintf('%s is not numeric (kept %g)', name, h.Value);
                return;
            end
            v = double(v(1));
            if isnan(v)
                msg = sprintf('%s is NaN (kept %g)', name, h.Value);
                return;
            end

            %% ===== Clamp into the field limits and assign =====
            lim = h.Limits;
            vc  = min(max(v, lim(1)), lim(2));
            lowerOpen = strcmp(char(h.LowerLimitInclusive), 'off');
            if lowerOpen && vc <= lim(1)
                msg = sprintf('%s = %g is not > %g (kept %g)', name, v, lim(1), h.Value);
                return;
            end
            if vc ~= v
                msg = sprintf('%s = %g clamped to %g', name, v, vc);
            end
            h.Value = vc;
        end

        function msg = setDropDownValue(app, dd, v, name)
        %SETDROPDOWNVALUE Select a dropdown entry by its ItemsData value if it exists.
        %
        % Inputs:
        %   app  - ADCS_ComparisonApp, this app instance                       [-]
        %   dd   - matlab.ui.control.DropDown, target dropdown               [-]
        %   v    - double [1] or char, ItemsData value to select             [-]
        %   name - char, cfg field name used in warnings                     [-]
        % Outputs:
        %   msg  - char, warning text ('' if selected)                       [-]

            msg = '';
            data = dd.ItemsData;
            if iscell(data)
                found = ischar(v) && any(strcmp(v, data));
            else
                found = isnumeric(v) && isscalar(v) && any(data == v);
            end
            if found
                dd.Value = v;
            else
                msg = sprintf('%s has an unsupported value (kept current selection)', name);
            end
        end

        function setDropDownItems(app, dd, items, data)
        %SETDROPDOWNITEMS Replace the Items and ItemsData of a dropdown safely.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   dd    - matlab.ui.control.DropDown, target dropdown              [-]
        %   items - cell [1xK] of char, display names                        [-]
        %   data  - 1xK numeric array or cell of char, ItemsData             [-]
        % Outputs:
        %   (none)

            dd.ItemsData = {};
            dd.Items = items;
            dd.ItemsData = data;
        end

        function updateNoiseEnable(app)
        %UPDATENOISEENABLE Enable the noise-std fields only when sensor noise is enabled.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none)

            st = onOff(app, app.NoiseCheckBox.Value);
            app.AttNoiseEditField.Enable  = st;
            app.GyroNoiseEditField.Enable = st;
        end

        function s = noteSuffix(app)
        %NOTESUFFIX Status-bar suffix listing the notes of the last writeCfgToWidgets.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   s   - char, ' (notes: ...)' or '' when there were none           [-]

            if isempty(app.WriteNotes)
                s = '';
            else
                s = [' (notes: ' app.WriteNotes ')'];
            end
        end

        function s = onOff(app, tf)
        %ONOFF Convert a logical to the char 'on' / 'off'.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        %   tf  - logical [1]                                                  [-]
        % Outputs:
        %   s   - char, 'on' if tf is true, else 'off'                       [-]

            if tf
                s = 'on';
            else
                s = 'off';
            end
        end

        function setStatus(app, msg)
        %SETSTATUS Show a message in the status bar.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        %   msg - char, message text                                         [-]
        % Outputs:
        %   (none)

            app.StatusLabel.Text = msg;
            app.StatusLabel.Tooltip = msg;
            drawnow limitrate;
        end

        function appendLog(app, msg)
        %APPENDLOG Append a time-stamped line to the run log.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        %   msg - char, message text                                         [-]
        % Outputs:
        %   (none)

            %% ===== Append the stamped line =====
            stamp = char(datetime('now', 'Format', 'HH:mm:ss'));
            v = app.LogTextArea.Value;
            if ischar(v)
                v = {v};
            end
            v = v(:);
            if numel(v) == 1 && isempty(v{1})
                v = {};
            end
            app.LogTextArea.Value = [v; {sprintf('[%s] %s', stamp, msg)}];

            %% ===== Scroll to the newest line =====
            try
                scroll(app.LogTextArea, 'bottom');
            catch
                % scroll() on text areas is not available in every release; ignore.
            end
        end

        function names = gainColumnNames(app, n)
        %GAINCOLUMNNAMES Column headers for a 1xn gain table.
        %
        % Inputs:
        %   app   - ADCS_ComparisonApp, this app instance                      [-]
        %   n     - double [1], number of elements                           [-]
        % Outputs:
        %   names - cell [1xn] of char, column headers                       [-]

            switch n
                case 1
                    names = {'value'};
                case 3
                    names = {'x', 'y', 'z'};
                case 6
                    names = {'Jxx', 'Jyy', 'Jzz', 'Jxy', 'Jxz', 'Jyz'};
                otherwise
                    names = arrayfun(@(k) sprintf('%d', k), 1:n, 'UniformOutput', false);
            end
        end

    end

    %% ===== Component creation =====
    methods (Access = private)

        function createComponents(app)
        %CREATECOMPONENTS Create the figure and every UI component (App Designer style).
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none) - components stored in the public properties

            %% ===== Figure and root grid =====
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [60 60 1400 860];
            app.UIFigure.Name = 'ADCS-MATLAB-App — LEO Chaser / ISS Rendezvous';

            app.MainGrid = uigridlayout(app.UIFigure, [2 1]);
            app.MainGrid.RowHeight = {'1x', 24};
            app.MainGrid.ColumnWidth = {'1x'};
            app.MainGrid.Padding = [6 4 6 6];
            app.MainGrid.RowSpacing = 4;

            %% ===== Tab group =====
            app.TabGroup = uitabgroup(app.MainGrid);
            app.TabGroup.Layout.Row = 1;
            app.TabGroup.Layout.Column = 1;
            app.ScenarioTab = uitab(app.TabGroup, 'Title', 'Scenario & Initial Conditions');
            app.GainsTab    = uitab(app.TabGroup, 'Title', 'Controller Gains');
            app.RunTab      = uitab(app.TabGroup, 'Title', 'Run');
            app.ResultsTab  = uitab(app.TabGroup, 'Title', 'Results');

            %% ===== Status bar =====
            app.StatusLabel = uilabel(app.MainGrid, 'Text', 'Starting ...');
            app.StatusLabel.Layout.Row = 2;
            app.StatusLabel.Layout.Column = 1;
            app.StatusLabel.FontColor = [0.2 0.2 0.2];
            app.StatusLabel.BackgroundColor = [0.93 0.93 0.93];

            %% ===== Tabs =====
            createScenarioTab(app);
            createGainsTab(app);
            createRunTab(app);
            createResultsTab(app);

            app.UIFigure.Visible = 'on';
        end

        function createScenarioTab(app)
        %CREATESCENARIOTAB Build tab 1: preset, attitude, reference, inertia, disturbance, actuator, simulation panels.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none)

            g = uigridlayout(app.ScenarioTab, [3 3]);
            g.RowHeight   = {70, '1.3x', '1x'};
            g.ColumnWidth = {'1x', '1x', '1.15x'};
            g.Padding     = [8 8 8 8];
            g.Scrollable  = 'on';
            app.ScenarioGrid = g;

            %% ===== Preset panel =====
            [app.PresetPanel, pg] = newPanel(app, g, 'Preset', 1, [1 3], [1 6]);
            pg.ColumnWidth = {'fit', 300, 'fit', 'fit', 'fit', '1x'};
            pg.RowHeight   = {22};
            addLabel(app, pg, 1, 1, 'Scenario preset', 'Predefined scenarios from listScenarios()');
            app.PresetDropDown = uidropdown(pg, 'Items', {'(loading)'});
            app.PresetDropDown.Layout.Row = 1;
            app.PresetDropDown.Layout.Column = 2;
            app.PresetDropDown.Tooltip = 'Scenario function from config/scenarios';
            app.LoadPresetButton = uibutton(pg, 'push', 'Text', 'Load preset');
            app.LoadPresetButton.Layout.Row = 1;
            app.LoadPresetButton.Layout.Column = 3;
            app.LoadPresetButton.Tooltip = 'cfg = feval(preset); all widgets are refreshed';
            app.LoadPresetButton.ButtonPushedFcn = createCallbackFcn(app, @LoadPresetButtonPushed, true);
            app.SaveConfigButton = uibutton(pg, 'push', 'Text', 'Save config…');
            app.SaveConfigButton.Layout.Row = 1;
            app.SaveConfigButton.Layout.Column = 4;
            app.SaveConfigButton.Tooltip = 'Save the current settings (saveConfig) to a .mat file';
            app.SaveConfigButton.ButtonPushedFcn = createCallbackFcn(app, @SaveConfigButtonPushed, true);
            app.LoadConfigButton = uibutton(pg, 'push', 'Text', 'Load config…');
            app.LoadConfigButton.Layout.Row = 1;
            app.LoadConfigButton.Layout.Column = 5;
            app.LoadConfigButton.Tooltip = 'Load settings from a .mat file (loadConfig)';
            app.LoadConfigButton.ButtonPushedFcn = createCallbackFcn(app, @LoadConfigButtonPushed, true);
            app.ConfigNameLabel = uilabel(pg, 'Text', 'Current: -');
            app.ConfigNameLabel.Layout.Row = 1;
            app.ConfigNameLabel.Layout.Column = 6;
            app.ConfigNameLabel.FontAngle = 'italic';

            %% ===== Attitude error panel =====
            [app.AttitudePanel, ag] = newPanel(app, g, 'Attitude error', 2, 1, [5 4]);
            ag.ColumnWidth = {'fit', '1x', '1x', '1x'};
            ag.RowHeight   = {22, 22, 40, 22, '1x'};
            addLabel(app, ag, 1, 1, 'Rotation axis ê [-]', 'Axis of the initial attitude error (x, y, z); normalised when read');
            flds = cell(1, 3);
            for k = 1:3
                flds{k} = uieditfield(ag, 'numeric');
                flds{k}.Layout.Row = 1;
                flds{k}.Layout.Column = k + 1;
                flds{k}.ValueDisplayFormat = '%.6g';
                flds{k}.Tooltip = sprintf('Axis component %s [-]', char('x' + k - 1));
            end
            app.AxisEditFields = [flds{:}];
            addLabel(app, ag, 2, 1, 'Error angle [deg]', 'Initial attitude error angle about the axis, 0..180 deg');
            app.AngleEditField = uieditfield(ag, 'numeric', 'Limits', [0 180]);
            app.AngleEditField.Layout.Row = 2;
            app.AngleEditField.Layout.Column = 2;
            app.AngleEditField.ValueDisplayFormat = '%.4g';
            app.AngleEditField.ValueChangedFcn = createCallbackFcn(app, @AngleEditFieldValueChanged, true);
            app.AngleSlider = uislider(ag, 'Limits', [0 180]);
            app.AngleSlider.Layout.Row = 3;
            app.AngleSlider.Layout.Column = [1 4];
            app.AngleSlider.MajorTicks = 0:30:180;
            app.AngleSlider.ValueChangingFcn = createCallbackFcn(app, @AngleSliderValueChanging, true);
            app.AngleSlider.ValueChangedFcn  = createCallbackFcn(app, @AngleSliderValueChanged, true);
            addLabel(app, ag, 4, 1, 'Rate error [deg/s]', 'Initial body-rate error w.r.t. the reference at t = 0 (x, y, z)');
            flds = cell(1, 3);
            for k = 1:3
                flds{k} = uieditfield(ag, 'numeric');
                flds{k}.Layout.Row = 4;
                flds{k}.Layout.Column = k + 1;
                flds{k}.ValueDisplayFormat = '%.6g';
                flds{k}.Tooltip = sprintf('Rate error component %s [deg/s]', char('x' + k - 1));
            end
            app.RateEditFields = [flds{:}];
            note = addLabel(app, ag, 5, [1 4], ...
                'The axis is normalised when read. Rate error = body rate minus reference rate at t = 0.', '');
            note.FontColor = [0.4 0.4 0.4];
            note.WordWrap = 'on';
            note.VerticalAlignment = 'top';

            %% ===== Reference / orbit panel =====
            [app.ReferencePanel, rg] = newPanel(app, g, 'Reference / orbit', 2, 2, [10 2]);
            rg.ColumnWidth = {'fit', '1x'};
            rg.RowHeight   = repmat({22}, 1, 10);
            addLabel(app, rg, 1, 1, 'Target mode', 'Reference attitude: inertial hold, LVLH hold, docking approach (line of sight to the ISS) or a persistently exciting attitude scan about LVLH hold');
            app.TargetModeDropDown = uidropdown(rg, 'Items', {'Inertial hold', 'LVLH hold', 'Docking approach', 'Attitude scan (PE)'}, ...
                'ItemsData', [0 1 2 3], 'Value', 2);
            app.TargetModeDropDown.Layout.Row = 1;
            app.TargetModeDropDown.Layout.Column = 2;
            app.AltitudeEditField    = addNumericRow(app, rg, 2, 'Altitude [km]', 'Circular orbit altitude', [0 Inf], 'off');
            app.InclinationEditField = addNumericRow(app, rg, 3, 'Inclination [deg]', 'Orbit inclination (magnetic-field model)', [0 180], 'on');
            app.R0EditField          = addNumericRow(app, rg, 4, 'R0 [m]', 'Initial chaser-to-ISS range (docking mode)', [0 Inf], 'off');
            app.RfEditField          = addNumericRow(app, rg, 5, 'Rf [m]', 'Final range of the exponential closing profile', [0 Inf], 'on');
            app.TcloseEditField      = addNumericRow(app, rg, 6, 'T_close [s]', 'Time constant of R(t) = Rf + (R0 - Rf) exp(-t/T_close)', [0 Inf], 'off');
            app.Yoff0EditField       = addNumericRow(app, rg, 7, 'y_off0 [m]', 'Initial cross-track offset in LVLH, shrinks as (R/R0)^2', [-Inf Inf], 'on');
            app.Zoff0EditField       = addNumericRow(app, rg, 8, 'z_off0 [m]', 'Initial radial offset in LVLH, shrinks as (R/R0)^2', [-Inf Inf], 'on');
            app.ScanAmpEditFields    = addTripletRow(app, rg, 9, 'Scan amp [deg]', 'Mode 3: amplitudes A_i of the scan rotation vector phi_i = A_i sin(w_i t)', [0 90]);
            app.ScanFreqEditFields   = addTripletRow(app, rg, 10, 'Scan freq [rad/s]', 'Mode 3: angular frequencies w_i (use incommensurate values for persistent excitation)', [0 1]);

            %% ===== Inertia panel =====
            [app.InertiaPanel, ig] = newPanel(app, g, 'Inertia', 2, 3, [6 4]);
            ig.ColumnWidth = {'fit', '1x', 'fit', 70};
            ig.RowHeight   = {18, 92, 40, 22, 18, 92};
            addLabel(app, ig, 1, [1 4], 'J_nom [kg·m²] (editing an element mirrors it to keep J symmetric)', ...
                'Nominal inertia used by the controllers');
            app.JnomTable = uitable(ig);
            app.JnomTable.Layout.Row = 2;
            app.JnomTable.Layout.Column = [1 4];
            app.JnomTable.ColumnName = {'x', 'y', 'z'};
            app.JnomTable.RowName = {'x', 'y', 'z'};
            app.JnomTable.Data = zeros(3);
            app.JnomTable.ColumnEditable = true(1, 3);
            app.JnomTable.ColumnFormat = {'numeric', 'numeric', 'numeric'};
            app.JnomTable.CellEditCallback = createCallbackFcn(app, @JnomTableCellEdit, true);
            addLabel(app, ig, 3, 1, 'Uncertainty [%]', 'Magnitude of the true-inertia perturbation, 0..50 %');
            app.UncSlider = uislider(ig, 'Limits', [0 50]);
            app.UncSlider.Layout.Row = 3;
            app.UncSlider.Layout.Column = [2 3];
            app.UncSlider.MajorTicks = 0:10:50;
            app.UncSlider.ValueChangingFcn = createCallbackFcn(app, @UncSliderValueChanging, true);
            app.UncSlider.ValueChangedFcn  = createCallbackFcn(app, @UncSliderValueChanged, true);
            app.UncEditField = uieditfield(ig, 'numeric', 'Limits', [0 50]);
            app.UncEditField.Layout.Row = 3;
            app.UncEditField.Layout.Column = 4;
            app.UncEditField.ValueDisplayFormat = '%.4g';
            app.UncEditField.ValueChangedFcn = createCallbackFcn(app, @UncEditFieldValueChanged, true);
            addLabel(app, ig, 4, 1, 'Mode', 'Random signed per-parameter (seeded, physically valid) or uniform scale (1 + pct/100)');
            app.UncModeDropDown = uidropdown(ig, 'Items', {'Random per-parameter', 'Uniform scale'}, ...
                'ItemsData', [1 2], 'Value', 1);
            app.UncModeDropDown.Layout.Row = 4;
            app.UncModeDropDown.Layout.Column = 2;
            app.UncModeDropDown.ValueChangedFcn = createCallbackFcn(app, @InertiaParamChanged, true);
            addLabel(app, ig, 4, 3, 'Seed', 'RNG seed of the random perturbation [-]');
            app.SeedEditField = uieditfield(ig, 'numeric', 'Limits', [0 Inf], 'RoundFractionalValues', 'on');
            app.SeedEditField.Layout.Row = 4;
            app.SeedEditField.Layout.Column = 4;
            app.SeedEditField.ValueDisplayFormat = '%.0f';
            app.SeedEditField.ValueChangedFcn = createCallbackFcn(app, @InertiaParamChanged, true);
            jl = uigridlayout(ig, [1 2]);
            jl.Layout.Row = 5;
            jl.Layout.Column = [1 4];
            jl.Padding = [0 0 0 0];
            jl.ColumnWidth = {'fit', '1x'};
            jl.RowHeight = {'1x'};
            addLabel(app, jl, 1, 1, 'J_true (derived) [kg·m²]', 'True plant inertia from buildSimParams(cfg) (read-only)');
            app.JtrueNoteLabel = uilabel(jl, 'Text', '');
            app.JtrueNoteLabel.Layout.Row = 1;
            app.JtrueNoteLabel.Layout.Column = 2;
            app.JtrueNoteLabel.FontAngle = 'italic';
            app.JtrueTable = uitable(ig);
            app.JtrueTable.Layout.Row = 6;
            app.JtrueTable.Layout.Column = [1 4];
            app.JtrueTable.ColumnName = {'x', 'y', 'z'};
            app.JtrueTable.RowName = {'x', 'y', 'z'};
            app.JtrueTable.Data = zeros(3);
            app.JtrueTable.ColumnEditable = false(1, 3);
            app.JtrueTable.ColumnFormat = {'numeric', 'numeric', 'numeric'};

            %% ===== Disturbances panel =====
            [app.DisturbancePanel, dg] = newPanel(app, g, 'Disturbances', 3, 1, [5 3]);
            dg.ColumnWidth = {'1x', 'fit', 90};
            dg.RowHeight   = repmat({22}, 1, 5);
            addLabel(app, dg, 1, 1, 'Source (enable)', 'dist_enable flags');
            addLabel(app, dg, 1, [2 3], 'Scale [-]', 'dist_scale multipliers on each torque');
            cbs = cell(1, 4);
            sfs = cell(1, 4);
            for k = 1:4
                cbs{k} = uicheckbox(dg, 'Text', app.DistNames{k});
                cbs{k}.Layout.Row = k + 1;
                cbs{k}.Layout.Column = 1;
                sfs{k} = uieditfield(dg, 'numeric', 'Limits', [0 Inf]);
                sfs{k}.Layout.Row = k + 1;
                sfs{k}.Layout.Column = 3;
                sfs{k}.ValueDisplayFormat = '%.4g';
                sfs{k}.Tooltip = sprintf('Magnitude multiplier of the %s torque [-]', app.DistNames{k});
                addLabel(app, dg, k + 1, 2, '×', '');
            end
            app.DistCheckBoxes = [cbs{:}];
            app.DistScaleEditFields = [sfs{:}];

            %% ===== Actuator & sensors panel =====
            [app.ActuatorPanel, xg] = newPanel(app, g, 'Actuator & sensors', 3, 2, [5 2]);
            xg.ColumnWidth = {'fit', '1x'};
            xg.RowHeight   = repmat({22}, 1, 5);
            app.TauMaxEditField = addNumericRow(app, xg, 1, 'Wheel torque max [N·m]', 'Per-wheel torque saturation', [0 Inf], 'off');
            app.HMaxEditField   = addNumericRow(app, xg, 2, 'Wheel momentum max [N·m·s]', 'Per-wheel momentum saturation', [0 Inf], 'off');
            app.NoiseCheckBox = uicheckbox(xg, 'Text', 'Sensor noise enabled');
            app.NoiseCheckBox.Layout.Row = 3;
            app.NoiseCheckBox.Layout.Column = [1 2];
            app.NoiseCheckBox.ValueChangedFcn = createCallbackFcn(app, @NoiseCheckBoxValueChanged, true);
            app.AttNoiseEditField  = addNumericRow(app, xg, 4, 'Attitude noise std [deg]', 'Star-tracker white-noise standard deviation', [0 Inf], 'on');
            app.GyroNoiseEditField = addNumericRow(app, xg, 5, 'Gyro noise std [deg/s]', 'Gyro white-noise standard deviation', [0 Inf], 'on');

            %% ===== Simulation panel =====
            [app.SimulationPanel, sg] = newPanel(app, g, 'Simulation', 3, 3, [3 2]);
            sg.ColumnWidth = {'fit', '1x'};
            sg.RowHeight   = repmat({22}, 1, 3);
            app.TfinalEditField       = addNumericRow(app, sg, 1, 't_final [s]', 'Simulation stop time (must exceed dt)', [0 Inf], 'off');
            app.DtEditField           = addNumericRow(app, sg, 2, 'dt [s]', 'Fixed integration step (ode4 / RK4)', [0 Inf], 'off');
            app.SettleThreshEditField = addNumericRow(app, sg, 3, 'Settle threshold [deg]', ...
                'Settling time = first t after which the attitude error stays below this value', [0 Inf], 'off');
        end

        function createGainsTab(app)
        %CREATEGAINSTAB Build tab 2: nested sub-tabs generated from gainMetadata().
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none) - app.GainWidgets filled

            %% ===== Container grid and nested tab group =====
            g = uigridlayout(app.GainsTab, [1 1]);
            g.Padding = [8 8 8 8];
            app.GainsTabGroup = uitabgroup(g);
            app.GainsTabGroup.Layout.Row = 1;
            app.GainsTabGroup.Layout.Column = 1;

            %% ===== One generated sub-tab per controller =====
            meta = gainMetadata();
            app.GainWidgets = struct();
            btns = cell(1, numel(app.GainCtrls));
            for c = 1:numel(app.GainCtrls)
                ctrl = app.GainCtrls{c};
                tab  = uitab(app.GainsTabGroup, 'Title', app.GainTabTitles{c});
                switch ctrl
                    case 'robust'
                        app.RobustGainsTab = tab;
                    case 'adaptive'
                        app.AdaptiveGainsTab = tab;
                    case 'baseline'
                        app.BaselineGainsTab = tab;
                end
                if isfield(meta, ctrl)
                    rows = meta.(ctrl);
                else
                    rows = cell(0, 5);
                end
                btns{c} = createGainSubTab(app, tab, ctrl, rows);
            end
            app.LoadDefaultsButtons = [btns{:}];
        end

        function btn = createGainSubTab(app, tab, ctrl, rows)
        %CREATEGAINSUBTAB Generate one gain sub-tab: a labelled 1xn uitable per metadata row.
        %
        % Inputs:
        %   app  - ADCS_ComparisonApp, this app instance                       [-]
        %   tab  - matlab.ui.container.Tab, parent sub-tab                   [-]
        %   ctrl - char, 'robust' | 'adaptive' | 'baseline'                  [-]
        %   rows - cell [Nx5], {field, label, units, n_elements, description} [mixed]
        % Outputs:
        %   btn  - matlab.ui.control.Button, the sub-tab's 'Load defaults' button [-]

            %% ===== Sub-tab grid =====
            N  = size(rows, 1);
            sg = uigridlayout(tab, [N + 1, 3]);
            sg.RowHeight   = [{34}, repmat({56}, 1, N)];
            sg.ColumnWidth = {280, 480, '1x'};
            sg.Padding     = [10 10 10 10];
            sg.RowSpacing  = 8;
            sg.Scrollable  = 'on';

            %% ===== Toolbar =====
            tb = uigridlayout(sg, [1 7]);
            tb.Layout.Row = 1;
            tb.Layout.Column = [1 3];
            tb.Padding = [0 0 0 0];
            tb.ColumnWidth = {'fit', 'fit', 70, 'fit', 70, 'fit', '1x'};
            tb.RowHeight = {'1x'};
            btn = uibutton(tb, 'push', 'Text', 'Load defaults');
            btn.Layout.Row = 1;
            btn.Layout.Column = 1;
            btn.UserData = ctrl;
            btn.Tooltip = sprintf('Replace these gains by %s_default(J_nom) using the current J_nom table', ctrl);
            btn.ButtonPushedFcn = createCallbackFcn(app, @LoadGainDefaultsButtonPushed, true);
            switch ctrl
                case 'robust'
                    app.AblationCheckBox = uicheckbox(tb, 'Text', ...
                        'Keep Adaptive Λ, K, η, φ identical to Robust (clean ablation)', 'Value', true);
                    app.AblationCheckBox.Layout.Row = 1;
                    app.AblationCheckBox.Layout.Column = [2 7];
                    app.AblationCheckBox.Tooltip = ['When ticked, the Adaptive SMC differs from the Robust SMC ' ...
                        'only by the online inertia estimate theta_hat'];
                    app.AblationCheckBox.ValueChangedFcn = createCallbackFcn(app, @AblationCheckBoxValueChanged, true);
                case 'adaptive'
                    nt = addLabel(app, tb, 1, [2 7], ...
                        'Λ, K, η, φ are copied from the Robust tab (read-only) while the clean-ablation box there is ticked.', '');
                    nt.FontColor = [0.4 0.4 0.4];
                case 'baseline'
                    addLabel(app, tb, 1, 2, 'ωn [rad/s]', 'Closed-loop natural frequency for the PD design');
                    app.WnEditField = uieditfield(tb, 'numeric', 'Limits', [0 Inf], 'Value', 0.03, ...
                        'LowerLimitInclusive', 'off');
                    app.WnEditField.Layout.Row = 1;
                    app.WnEditField.Layout.Column = 3;
                    app.WnEditField.ValueDisplayFormat = '%.4g';
                    addLabel(app, tb, 1, 4, 'ζ [-]', 'Damping ratio for the PD design');
                    app.ZetaEditField = uieditfield(tb, 'numeric', 'Limits', [0 Inf], 'Value', 0.9);
                    app.ZetaEditField.Layout.Row = 1;
                    app.ZetaEditField.Layout.Column = 5;
                    app.ZetaEditField.ValueDisplayFormat = '%.4g';
                    app.ComputePDButton = uibutton(tb, 'push', 'Text', 'Compute Kp/Kd from ωn, ζ');
                    app.ComputePDButton.Layout.Row = 1;
                    app.ComputePDButton.Layout.Column = 6;
                    app.ComputePDButton.Tooltip = 'Kp = 2 diag(J_nom) wn^2, Kd = 2 zeta wn diag(J_nom) (pdGainsFromBandwidth)';
                    app.ComputePDButton.ButtonPushedFcn = createCallbackFcn(app, @ComputePDButtonPushed, true);
            end

            %% ===== One row per gain field =====
            W = struct();
            for i = 1:N
                field = rows{i, 1};
                label = rows{i, 2};
                units = rows{i, 3};
                n     = double(rows{i, 4});
                desc  = rows{i, 5};
                if isempty(units)
                    units = '-';
                end
                lbl = addLabel(app, sg, i + 1, 1, sprintf('%s [%s]', label, units), desc);
                lbl.FontWeight = 'bold';
                tbl = uitable(sg);
                tbl.Layout.Row = i + 1;
                tbl.Layout.Column = 2;
                tbl.ColumnName = gainColumnNames(app, n);
                tbl.RowName = {};
                tbl.Data = zeros(1, n);
                tbl.ColumnEditable = true(1, n);
                tbl.ColumnFormat = repmat({'numeric'}, 1, n);
                tbl.UserData = struct('ctrl', ctrl, 'field', field, 'n', n);
                tbl.CellEditCallback = createCallbackFcn(app, @GainTableCellEdit, true);
                W.(field) = tbl;
                dl = addLabel(app, sg, i + 1, 3, desc, desc);
                dl.FontColor = [0.35 0.35 0.35];
                dl.WordWrap = 'on';
            end
            app.GainWidgets.(ctrl) = W;
        end

        function createRunTab(app)
        %CREATERUNTAB Build tab 3: run mode, controller, engine, Run button, Simulink status and log.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none)

            %% ===== Layout =====
            g = uigridlayout(app.RunTab, [1 2]);
            g.ColumnWidth = {340, '1x'};
            g.RowHeight   = {'1x'};
            g.Padding     = [10 10 10 10];
            lc = uigridlayout(g, [8 1]);
            lc.Layout.Row = 1;
            lc.Layout.Column = 1;
            lc.RowHeight = {90, 22, 22, 22, 22, 44, 44, '1x'};
            lc.ColumnWidth = {'1x'};
            lc.Padding = [0 0 0 0];

            %% ===== Run mode (radio buttons must be direct children of the button group) =====
            app.RunModeButtonGroup = uibuttongroup(lc, 'Title', 'Run mode');
            app.RunModeButtonGroup.Layout.Row = 1;
            app.RunModeButtonGroup.Layout.Column = 1;
            app.RunModeButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @RunModeSelectionChanged, true);
            app.SingleRadioButton = uiradiobutton(app.RunModeButtonGroup, 'Text', 'Single controller', ...
                'Position', [12 36 250 22]);
            app.CompareAllRadioButton = uiradiobutton(app.RunModeButtonGroup, 'Text', 'Compare all three', ...
                'Position', [12 10 250 22]);
            app.CompareAllRadioButton.Value = true;

            %% ===== Controller and engine =====
            addLabel(app, lc, 2, 1, 'Controller (single run)', 'Used only in Single controller mode');
            app.ControllerDropDown = uidropdown(lc, 'Items', {'(loading)'});
            app.ControllerDropDown.Layout.Row = 3;
            app.ControllerDropDown.Layout.Column = 1;
            app.ControllerDropDown.Enable = 'off';
            addLabel(app, lc, 4, 1, 'Engine', ['Auto = Simulink if available, otherwise the MATLAB reference ' ...
                'engine (RK4, same dt)']);
            app.EngineDropDown = uidropdown(lc, 'Items', {'Auto', 'Simulink', 'MATLAB reference'}, ...
                'ItemsData', {'auto', 'simulink', 'reference'}, 'Value', 'auto');
            app.EngineDropDown.Layout.Row = 5;
            app.EngineDropDown.Layout.Column = 1;

            %% ===== Run button and Simulink status =====
            app.RunButton = uibutton(lc, 'push', 'Text', 'Run');
            app.RunButton.Layout.Row = 6;
            app.RunButton.Layout.Column = 1;
            app.RunButton.FontWeight = 'bold';
            app.RunButton.FontSize = 16;
            app.RunButton.Tooltip = 'Read all settings, simulate, compute metrics and open the Results tab';
            app.RunButton.ButtonPushedFcn = createCallbackFcn(app, @RunButtonPushed, true);
            app.SimulinkLabel = uilabel(lc, 'Text', 'Simulink: checking ...');
            app.SimulinkLabel.Layout.Row = 7;
            app.SimulinkLabel.Layout.Column = 1;
            app.SimulinkLabel.WordWrap = 'on';

            %% ===== Log =====
            rc = uigridlayout(g, [2 1]);
            rc.Layout.Row = 1;
            rc.Layout.Column = 2;
            rc.RowHeight = {22, '1x'};
            rc.ColumnWidth = {'1x'};
            rc.Padding = [0 0 0 0];
            addLabel(app, rc, 1, 1, 'Run log', '');
            app.LogTextArea = uitextarea(rc, 'Editable', 'off', 'Value', {''});
            app.LogTextArea.Layout.Row = 2;
            app.LogTextArea.Layout.Column = 1;
        end

        function createResultsTab(app)
        %CREATERESULTSTAB Build tab 4: display controls, 2x3 result axes, metrics table and exports.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none)

            %% ===== Layout =====
            g = uigridlayout(app.ResultsTab, [3 1]);
            g.RowHeight   = {28, '1x', 170};
            g.ColumnWidth = {'1x'};
            g.Padding     = [8 8 8 8];
            g.RowSpacing  = 6;

            %% ===== Toolbar =====
            tb = uigridlayout(g, [1 7]);
            tb.Layout.Row = 1;
            tb.Layout.Column = 1;
            tb.Padding = [0 0 0 0];
            tb.ColumnWidth = {'fit', 190, 'fit', 190, '1x', 'fit', 'fit'};
            tb.RowHeight = {'1x'};
            addLabel(app, tb, 1, 1, 'Display', 'Overlay all stored results or show a single one');
            app.ResultsModeDropDown = uidropdown(tb, 'Items', {'Overlay comparison', 'Single run'}, ...
                'ItemsData', {'overlay', 'single'}, 'Value', 'overlay');
            app.ResultsModeDropDown.Layout.Row = 1;
            app.ResultsModeDropDown.Layout.Column = 2;
            app.ResultsModeDropDown.ValueChangedFcn = createCallbackFcn(app, @ResultsModeDropDownValueChanged, true);
            addLabel(app, tb, 1, 3, 'Controller', 'Result shown in Single run mode');
            app.ResultsCtrlDropDown = uidropdown(tb, 'Items', {'(no results)'}, 'ItemsData', 0, 'Value', 0);
            app.ResultsCtrlDropDown.Layout.Row = 1;
            app.ResultsCtrlDropDown.Layout.Column = 4;
            app.ResultsCtrlDropDown.Enable = 'off';
            app.ResultsCtrlDropDown.ValueChangedFcn = createCallbackFcn(app, @ResultsCtrlDropDownValueChanged, true);
            app.ExportFigureButton = uibutton(tb, 'push', 'Text', 'Export figure…');
            app.ExportFigureButton.Layout.Row = 1;
            app.ExportFigureButton.Layout.Column = 6;
            app.ExportFigureButton.ButtonPushedFcn = createCallbackFcn(app, @ExportFigureButtonPushed, true);
            app.ExportCSVButton = uibutton(tb, 'push', 'Text', 'Export metrics CSV…');
            app.ExportCSVButton.Layout.Row = 1;
            app.ExportCSVButton.Layout.Column = 7;
            app.ExportCSVButton.ButtonPushedFcn = createCallbackFcn(app, @ExportCSVButtonPushed, true);

            %% ===== 2x3 axes =====
            ag = uigridlayout(g, [2 3]);
            ag.Layout.Row = 2;
            ag.Layout.Column = 1;
            ag.Padding = [0 0 0 0];
            nQ = numel(app.PlotQuantities);
            axs = cell(1, nQ);
            for i = 1:nQ
                r = ceil(i / 3);
                c = i - 3 * (r - 1);
                axs{i} = uiaxes(ag);
                axs{i}.Layout.Row = r;
                axs{i}.Layout.Column = c;
                title(axs{i}, app.PlotTitles{i}, 'Interpreter', 'none');
                xlabel(axs{i}, 'Time [s]');
                grid(axs{i}, 'on');
            end
            app.ResultAxes = [axs{:}];

            %% ===== Metrics table =====
            app.MetricsTable = uitable(g);
            app.MetricsTable.Layout.Row = 3;
            app.MetricsTable.Layout.Column = 1;
            app.MetricsTable.Data = {};
            app.MetricsTable.RowName = {};
        end

        function [pnl, grd] = newPanel(app, parent, titleText, row, col, gridSize)
        %NEWPANEL Create a titled panel in a grid cell with an inner grid layout.
        %
        % Inputs:
        %   app       - ADCS_ComparisonApp, this app instance                  [-]
        %   parent    - matlab.ui.container.GridLayout, parent grid          [-]
        %   titleText - char, panel title                                    [-]
        %   row, col  - double [1] or [1x2], Layout.Row / Layout.Column      [-]
        %   gridSize  - double [1x2], [rows cols] of the inner grid          [-]
        % Outputs:
        %   pnl - matlab.ui.container.Panel, the panel                       [-]
        %   grd - matlab.ui.container.GridLayout, the inner grid             [-]

            pnl = uipanel(parent, 'Title', titleText);
            pnl.Layout.Row = row;
            pnl.Layout.Column = col;
            pnl.FontWeight = 'bold';
            grd = uigridlayout(pnl, gridSize);
            grd.Padding = [8 6 8 6];
            grd.RowSpacing = 6;
            grd.ColumnSpacing = 8;
            grd.Scrollable = 'on';
        end

        function lbl = addLabel(app, parent, row, col, txt, tip)
        %ADDLABEL Create a label in a grid cell.
        %
        % Inputs:
        %   app    - ADCS_ComparisonApp, this app instance                     [-]
        %   parent - matlab.ui.container.GridLayout, parent grid             [-]
        %   row    - double [1] or [1x2], Layout.Row                         [-]
        %   col    - double [1] or [1x2], Layout.Column                      [-]
        %   txt    - char, label text                                        [-]
        %   tip    - char, tooltip ('' for none)                             [-]
        % Outputs:
        %   lbl    - matlab.ui.control.Label, the label                      [-]

            lbl = uilabel(parent, 'Text', txt);
            lbl.Layout.Row = row;
            lbl.Layout.Column = col;
            if ~isempty(tip)
                lbl.Tooltip = tip;
            end
        end

        function fld = addNumericRow(app, parent, row, labelText, tip, limits, lowerInclusive)
        %ADDNUMERICROW Create a 'label | numeric edit field' row in a 2-column grid.
        %
        % Inputs:
        %   app            - ADCS_ComparisonApp, this app instance             [-]
        %   parent         - matlab.ui.container.GridLayout, parent grid     [-]
        %   row            - double [1], grid row                            [-]
        %   labelText      - char, label including units                     [-]
        %   tip            - char, tooltip for label and field               [-]
        %   limits         - double [1x2], allowed range of the value        [field units]
        %   lowerInclusive - char, 'on' | 'off', whether limits(1) is allowed [-]
        % Outputs:
        %   fld            - matlab.ui.control.NumericEditField, the field   [-]

            %% ===== Valid initial value (set BEFORE an open lower limit is applied) =====
            if strcmp(lowerInclusive, 'off')
                if isfinite(limits(2))
                    v0 = mean(limits);
                else
                    v0 = limits(1) + 1;
                end
            else
                v0 = min(max(0, limits(1)), limits(2));
            end

            %% ===== Label and field =====
            addLabel(app, parent, row, 1, labelText, tip);
            fld = uieditfield(parent, 'numeric', 'Limits', limits, 'Value', v0, ...
                'LowerLimitInclusive', lowerInclusive);
            fld.Layout.Row = row;
            fld.Layout.Column = 2;
            fld.ValueDisplayFormat = '%.6g';
            fld.Tooltip = tip;
        end

        function flds = addTripletRow(app, parent, row, labelText, tip, limits)
        %ADDTRIPLETROW Create a 'label | three numeric fields' row in a 2-column grid.
        %
        % Inputs:
        %   app       - ADCS_ComparisonApp, this app instance                  [-]
        %   parent    - matlab.ui.container.GridLayout, parent grid          [-]
        %   row       - double [1], grid row                                 [-]
        %   labelText - char, label including units                          [-]
        %   tip       - char, tooltip for the label and the fields           [-]
        %   limits    - double [1x2], allowed range of each value (inclusive) [field units]
        % Outputs:
        %   flds      - 1x3 matlab.ui.control.NumericEditField, the fields   [-]

            %% ===== Label and nested 1x3 grid =====
            addLabel(app, parent, row, 1, labelText, tip);
            sub = uigridlayout(parent, [1 3]);
            sub.Layout.Row = row;
            sub.Layout.Column = 2;
            sub.Padding = [0 0 0 0];
            sub.ColumnSpacing = 4;

            %% ===== Three numeric fields (x, y, z components) =====
            c = cell(1, 3);
            for k = 1:3
                c{k} = uieditfield(sub, 'numeric', 'Limits', limits, 'Value', limits(1));
                c{k}.Layout.Row = 1;
                c{k}.Layout.Column = k;
                c{k}.ValueDisplayFormat = '%.6g';
                c{k}.Tooltip = sprintf('%s, component %s', tip, char('x' + k - 1));
            end
            flds = [c{:}];
        end

    end

    %% ===== Static helpers =====
    methods (Static, Access = private)

        function ensurePaths(root)
        %ENSUREPATHS Run startup_ADCS from the project root if the library is not on the path.
        %
        % Inputs:
        %   root - char, absolute project root folder                        [-]
        % Outputs:
        %   (none) - MATLAB path updated (run in this function's own workspace
        %            so the startup script cannot touch caller or base variables)

            if exist('initDefaults', 'file') ~= 2
                startupFile = fullfile(root, 'startup_ADCS.m');
                if exist(startupFile, 'file') == 2
                    run(startupFile);
                end
            end
            if exist('initDefaults', 'file') ~= 2
                error('ADCS_ComparisonApp:path', ...
                    'ADCS library not found on the path. Run startup_ADCS.m in %s first.', root);
            end
        end

    end

    %% ===== App creation and deletion =====
    methods (Access = public)

        function app = ADCS_ComparisonApp()
        %ADCS_COMPARISONAPP Construct the app: set paths, create components, register, start up.
        %
        % Inputs:
        %   (none)
        % Outputs:
        %   app - ADCS_ComparisonApp, the running app instance (cleared if nargout == 0) [-]

            %% ===== Paths =====
            app.ProjectRoot = fileparts(fileparts(mfilename('fullpath')));
            ADCS_ComparisonApp.ensurePaths(app.ProjectRoot);

            %% ===== Components, registration, startup =====
            createComponents(app);
            registerApp(app, app.UIFigure);
            runStartupFcn(app, @startupFcn);

            if nargout == 0
                clear app
            end
        end

        function delete(app)
        %DELETE Delete the app and its figure.
        %
        % Inputs:
        %   app - ADCS_ComparisonApp, this app instance                        [-]
        % Outputs:
        %   (none)

            delete(app.UIFigure);
        end

    end
end
