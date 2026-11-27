function check_app_3d_view()
% Opt-in smoke test of the app's 3D View: run, play, scrub, switch controller, close without a leftover timer.
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)
%
% Not auto-discovered by run_all_tests (it opens the app window). MATLAB
% desktop only; skipped in GNU Octave and in -batch sessions.

%% ===== Guards =====
if exist('OCTAVE_VERSION', 'builtin') ~= 0 || batchStartupOptionUsed
    fprintf('check_app_3d_view: SKIP (needs the MATLAB desktop)\n');
    return;
end
assert(isempty(timerfind('Name', 'ADCS3DPlayback')), 'a playback timer is already running');

%% ===== Run all three controllers on a short reference-engine run =====
app = ADCS_MATLAB_App();
cleanup = onCleanup(@() closeApp(app));
app.TfinalEditField.Value = 30;                       % [s] short run
app.CompareAllRadioButton.Value = true;
app.EngineDropDown.Value = 'reference';
app.RunButton.ButtonPushedFcn(app.RunButton, []);
assert(strcmp(app.VisCtrlDropDown.Enable, 'on') && numel(app.VisCtrlDropDown.ItemsData) == 3, ...
       '3D controller selector not filled after the run');
assert(strcmp(app.VisPlayButton.Enable, 'on'), 'Play not enabled after the run');

%% ===== Play, pause, scrub, controller switch =====
app.VisSpeedDropDown.Value = 5;                      % [-] 5x
app.VisPlayButton.ButtonPushedFcn(app.VisPlayButton, []);
pause(1.0);
tm = timerfind('Name', 'ADCS3DPlayback');
assert(numel(tm) == 1 && strcmp(tm.Running, 'on'), 'playback timer not running');
assert(app.VisTimeSlider.Value > 0, 'playback did not advance');
app.VisPlayButton.ButtonPushedFcn(app.VisPlayButton, []);
assert(strcmp(tm.Running, 'off') && strcmp(app.VisPlayButton.Text, 'Play'), 'pause failed');
app.VisTimeSlider.Value = 12.34;                      % [s]
app.VisTimeSlider.ValueChangedFcn(app.VisTimeSlider, []);
assert(contains(app.VisTimeLabel.Text, '12.3'), 'scrub did not move to t = 12.3 s');
app.VisCtrlDropDown.Value = 2;
app.VisCtrlDropDown.ValueChangedFcn(app.VisCtrlDropDown, []);
app.VisCameraDropDown.Value = 'docking';
app.VisCameraDropDown.ValueChangedFcn(app.VisCameraDropDown, []);
app.VisChaserSizeDropDown.Value = 10;
app.VisChaserSizeDropDown.ValueChangedFcn(app.VisChaserSizeDropDown, []);
app.VisMagDropDown.Value = 10;
app.VisMagDropDown.ValueChangedFcn(app.VisMagDropDown, []);
drawnow;

%% ===== Close: no timer may survive =====
app.VisPlayButton.ButtonPushedFcn(app.VisPlayButton, []);
pause(0.5);
delete(app);
assert(isempty(timerfind('Name', 'ADCS3DPlayback')), 'playback timer survived delete(app)');
fprintf('check_app_3d_view: PASS\n');
end

%% ===== Local functions =====
function closeApp(app)
% Delete the app if it still exists (cleanup on failure).
%
% Inputs:
%   app - ADCS_MATLAB_App or deleted handle  [-]
% Outputs:
%   (none)

if isvalid(app)
    delete(app);
end
end
