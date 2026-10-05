# ADCS Comparison App (GUI)

`app/ADCS_MATLAB_App.m` is the graphical front end of the ADCS-MATLAB-App. With it you configure a LEO-chaser / ISS rendezvous scenario (3-axis rotational dynamics only) and run one controller or all three:

1. Robust SMC
2. Adaptive SMC (online inertia estimation)
3. PD Benchmark

You can then inspect the plots and the metrics side by side.

The app contains **no physics or metric code**. Every computation goes through the library functions of `docs/DESIGN_SPEC.md` §5: `initDefaults`, `listScenarios`, `gainMetadata`, `pdGainsFromBandwidth`, `buildSimParams`, `runSimulation`, `computeMetrics`, `evaluatePassFail`, `metricsTable`, `exportMetricsCSV`, `plotResultsOnAxes`, `saveConfig`, `loadConfig`, `controllerNames`, `isSimulinkAvailable`, and the gain defaults `robust_default` / `adaptive_default` / `baseline_default`.

## Launching

Requires MATLAB R2021a or newer. Simulink is optional: without it the MATLAB reference engine is used.

```matlab
>> run('<project root>/startup_ADCS.m')   % once per session (adds src/, config/, app/, ...)
>> app = ADCS_MATLAB_App();               % or: app = launchADCSApp();
>> delete(app)                             % closes the window (closing the window also works)
```

If the library is not on the path, both the constructor and `launchADCSApp` run `startup_ADCS.m` from the project root (resolved from the app file location) automatically. The script runs inside a function workspace, so the app **never writes to the base workspace**.

## Tabs

### 1. Scenario & Initial Conditions
| Panel | Contents |
|---|---|
| **Preset** | A dropdown filled from `listScenarios()` (shows display names; the value is the function name). **Load preset** runs `cfg = feval(fname)` and refreshes every widget. **Save config…** opens a save dialog that defaults to `config/saved/*.mat` and calls `saveConfig`. **Load config…** opens a file dialog and calls `loadConfig`, then refreshes every widget. The label shows `cfg.name`, with `cfg.description` as its tooltip. |
| **Attitude error** | Rotation axis x, y, z (normalised when read), angle in deg (a 0–180 slider linked to a numeric field), and initial rate error x, y, z in deg/s. |
| **Reference / orbit** | Target mode (Inertial hold = 0, LVLH hold = 1, Docking approach = 2), altitude km, inclination deg, R0 m, Rf m, T_close s, y_off0 m, z_off0 m. |
| **Inertia** | An editable 3×3 `J_nom` table: editing element (i,j) also writes (j,i), so the matrix stays symmetric. Uncertainty in % (a 0–50 slider plus a field), mode (Random per-parameter = 1, Uniform scale = 2) and seed. A read-only **J_true (derived)** table plus a note, refreshed through `buildSimParams(cfg)` whenever J_nom, uncertainty, mode or seed change. Any error from `buildSimParams` goes to the status bar. |
| **Disturbances** | Enable checkboxes and magnitude scales for Gravity gradient, Aerodynamic, SRP and Magnetic. |
| **Actuator & sensors** | Wheel torque max N·m, wheel momentum max N·m·s, a sensor-noise checkbox, attitude noise std deg and gyro noise std deg/s. The noise fields are greyed out while noise is off. |
| **Simulation** | t_final s, dt s, and the settling threshold in deg. |

### 2. Controller Gains
This tab has one sub-tab per controller: **Robust SMC**, **Adaptive SMC** and **PD Benchmark**. Each sub-tab is **generated from `gainMetadata()`**. Every metadata row `{field, label, units, n_elements, description}` becomes:

- a label `<label> [<units>]` whose tooltip is the description;
- a 1×n editable numeric `uitable`, stored as `GainWidgets.<ctrl>.<field>`;
- a grey copy of the description.

Non-finite entries are rejected and the previous value is restored.

- **Load defaults** (on every sub-tab) replaces that controller's gains with `robust_default` / `adaptive_default` / `baseline_default` evaluated at the *current* J_nom table.
- **Robust SMC** has the checkbox *Keep Adaptive Λ, K, η, φ identical to Robust (clean ablation)*, which is on by default. While it is ticked:
  - the adaptive tables for `lambda_diag`, `K_diag`, `eta` and `phi` mirror the robust ones and are read-only;
  - `readWidgetsToCfg` copies those four fields from `cfg.gains.robust` into `cfg.gains.adaptive`.

  When a preset or saved config is loaded whose adaptive values differ from the robust ones, the box is switched off automatically, so the loaded config is not altered silently. The status bar and log report this.
- **PD Benchmark** has ωn [rad/s] (default 0.03) and ζ [-] (default 0.9) fields. **Compute Kp/Kd from ωn, ζ** calls `pdGainsFromBandwidth(J_nom, wn, zeta)` and writes the results into the `Kp_diag` / `Kd_diag` tables.

J_nom edits do **not** recompute gains automatically. Press **Load defaults** or **Compute Kp/Kd** to do that.

### 3. Run
- The **Run mode** button group has two options: *Single controller*, which enables the controller dropdown (from `controllerNames()`, values 1–3), and *Compare all three* (the default).
- **Engine** can be *Auto* (`'auto'`), *Simulink* (`'simulink'`) or *MATLAB reference* (`'reference'`). *Auto* means Simulink if `isSimulinkAvailable()` returns true, otherwise the reference engine. A label shows the availability detected at startup.
- **Run** does the following:
  1. Calls `readWidgetsToCfg`.
  2. Validates the inputs: dt > 0, t_final > dt, angle in [0, 180], axis not all zero, J_nom positive definite, wheel limits > 0, all gains finite. If a check fails it shows a `uialert` and aborts.
  3. Disables the button and opens an indeterminate `uiprogressdlg`.
  4. Calls `results = runSimulation(cfg, ids, engine)`.
  5. For each result, calls `computeMetrics(res, cfg)` and `evaluatePassFail(M, cfg.pass)`.
  6. Stores `Results`, `Metrics`, `PassFlags` and the run's cfg snapshot, refreshes the Results tab and switches to it.

  Every error is caught: the progress dialog is closed, a `uialert` is shown, and the message plus its stack location is written to the log. The button is always re-enabled.
- **Run log** is a read-only, time-stamped text area.

### 4. Results
- The **Display** dropdown offers *Overlay comparison* (all stored results) or *Single run*. Single run enables the **Controller** selector, which lists the stored results.
- The 2×3 axes are titled *Attitude error [deg]*, *Rate error |ω_e| [deg/s]*, *Wheel torque |τ| [N·m]*, *Wheel momentum max|h| [N·m·s]*, *Quaternion norm deviation [-]* and *Adaptive θ̂/θ_true [-]*. Each is filled by `plotResultsOnAxes(ax, selected, quantity, cfg)` with quantities `att_err`, `w_e`, `tau_rw`, `h_w`, `qnorm` and `theta_hat`, after the legend is removed, `cla` is called and YScale is reset to linear. For `theta_hat`, only results with `ctrl_id == 2` are passed. If there are none, the text *Run the Adaptive controller to see inertia estimates* is shown.
- The **metrics table** is filled from `[header, rows] = metricsTable(Results, Metrics, PassFlags)`, with `ColumnName = header`.
- **Export figure…** calls `exportapp(UIFigure, file)` (png/pdf/jpg). If that fails, it falls back to `exportgraphics` on each axes, writing `<name>_<quantity>.<ext>`.
- **Export metrics CSV…** calls `exportMetricsCSV(file, header, rows)`.

### Status bar
The bottom line shows the latest message: loads, saves, validation, clamped values, J_true warnings and errors. Its tooltip holds the full text.

## Widget → cfg field mapping

This table mirrors `readWidgetsToCfg`, the **only** place that maps widgets to cfg, and its inverse `writeCfgToWidgets`. cfg fields that are not listed here are carried through unchanged from the loaded preset or config. These include the aero/SRP/magnetic constants, `noise_seed`, `gyro_bias_rw_degps_rts`, `h_w0_Nms`, `q_inertial_ref`, `shortest_path`, `gyro_comp`, `k_norm`, `ref_fd_h_s`, `metrics.ss_window_frac` and `pass.*`.

| Tab / panel | Widget (property) | cfg field | Units | Notes |
|---|---|---|---|---|
| Scenario / Attitude error | `AxisEditFields(1:3)` | `scenario.att_err_axis` [3x1] | [-] | normalised on read; all zeros is rejected at Run/Save |
| Scenario / Attitude error | `AngleEditField` (+ `AngleSlider`) | `scenario.att_err_angle_deg` | [deg] | limited to 0–180 |
| Scenario / Attitude error | `RateEditFields(1:3)` | `scenario.rate_err0_degps` [3x1] | [deg/s] | |
| Scenario / Reference | `TargetModeDropDown` | `scenario.target_mode` | [-] | 0 inertial / 1 LVLH / 2 docking / 3 attitude scan (PE) |
| Scenario / Reference | `AltitudeEditField` | `scenario.orbit_alt_km` | [km] | > 0 |
| Scenario / Reference | `InclinationEditField` | `scenario.orbit_incl_deg` | [deg] | 0–180 |
| Scenario / Reference | `R0EditField` | `scenario.R0_m` | [m] | > 0 |
| Scenario / Reference | `RfEditField` | `scenario.Rf_m` | [m] | ≥ 0 |
| Scenario / Reference | `TcloseEditField` | `scenario.T_close_s` | [s] | > 0 |
| Scenario / Reference | `Yoff0EditField` | `scenario.y_off0_m` | [m] | |
| Scenario / Reference | `Zoff0EditField` | `scenario.z_off0_m` | [m] | |
| Scenario / Reference | `ScanAmpEditFields(1:3)` | `scenario.scan_amp_deg` [3x1] | [deg] | mode 3 only; 0–90 |
| Scenario / Reference | `ScanFreqEditFields(1:3)` | `scenario.scan_freq_radps` [3x1] | [rad/s] | mode 3 only; 0–1 |
| Scenario / Inertia | `JnomTable` | `scenario.J_nom` [3x3] | [kg·m²] | symmetrised (J+Jᵀ)/2 on read |
| Scenario / Inertia | `UncEditField` (+ `UncSlider`) | `scenario.J_unc_pct` | [%] | 0–50 |
| Scenario / Inertia | `UncModeDropDown` | `scenario.J_unc_mode` | [-] | 1 / 2 |
| Scenario / Inertia | `SeedEditField` | `scenario.J_unc_seed` | [-] | integer ≥ 0 |
| Scenario / Disturbances | `DistCheckBoxes(1:4)` | `scenario.dist_enable` [1x4] | [-] | order GG, Aero, SRP, Mag |
| Scenario / Disturbances | `DistScaleEditFields(1:4)` | `scenario.dist_scale` [1x4] | [-] | ≥ 0 |
| Scenario / Actuator | `TauMaxEditField` | `scenario.wheel_tau_max_Nm` | [N·m] | > 0 |
| Scenario / Actuator | `HMaxEditField` | `scenario.wheel_h_max_Nms` | [N·m·s] | > 0 |
| Scenario / Actuator | `NoiseCheckBox` | `scenario.noise_enable` | [-] | 0 / 1 |
| Scenario / Actuator | `AttNoiseEditField` | `scenario.att_noise_std_deg` | [deg] | ≥ 0 |
| Scenario / Actuator | `GyroNoiseEditField` | `scenario.gyro_noise_std_degps` | [deg/s] | ≥ 0 |
| Scenario / Simulation | `TfinalEditField` | `scenario.t_final_s` | [s] | must be > dt |
| Scenario / Simulation | `DtEditField` | `scenario.dt_s` | [s] | > 0 |
| Scenario / Simulation | `SettleThreshEditField` | `metrics.settle_thresh_deg` | [deg] | > 0 |
| Gains / Robust SMC | `GainWidgets.robust.<field>` | `gains.robust.<field>` [nx1] | per `gainMetadata` | lambda_diag, K_diag, eta, phi |
| Gains / Adaptive SMC | `GainWidgets.adaptive.<field>` | `gains.adaptive.<field>` [nx1] | per `gainMetadata` | lambda_diag, K_diag, eta and phi are copied from robust while the ablation box is ticked; also Gamma_diag, theta_hat0_from_Jnom, theta_hat0, proj_frac and freeze_on_sat (label *Freeze on saturation*, adaptation anti-windup) |
| Gains / PD Benchmark | `GainWidgets.baseline.<field>` | `gains.baseline.<field>` [nx1] | per `gainMetadata` | Kp_diag, Kd_diag |

GUI-only widgets that do not map to cfg:

| Widget | Purpose |
|---|---|
| `AblationCheckBox` | Controls the robust→adaptive gain copy described above. |
| `WnEditField`, `ZetaEditField` | Inputs to `pdGainsFromBandwidth` only. |
| Run tab (mode, controller, engine) | Arguments to `runSimulation`, not stored in cfg. |

When `writeCfgToWidgets` meets a value outside a widget's limits (for example `J_unc_pct` > 50), it clamps the value and reports it in the status bar and the log.

## Why a programmatic AppBase class and not a binary `.mlapp`?

The file follows the structure of App Designer's generated code:

- `classdef ADCS_MATLAB_App < matlab.apps.AppBase`;
- public, typed component properties;
- private state properties (`Cfg`, `Results`, `Metrics`, `PassFlags`, `GainWidgets`);
- a private `createComponents(app)`;
- callbacks as private methods wired with `createCallbackFcn(app, @method, true)`;
- a constructor that calls `createComponents`, `registerApp(app, app.UIFigure)` and `runStartupFcn(app, @startupFcn)`;
- `delete(app)`, which deletes `UIFigure`.

It is kept as plain `.m` text rather than a `.mlapp` (a zipped binary container) for two reasons:

1. **It can be reviewed as text.** Diffs, code review and the project's commenting standard apply to every line.
2. **An `.mlapp` cannot be authored without MATLAB.** The file was written in an environment without MATLAB. A hand-built `.mlapp` could not have been verified at all.

Functionally the two are equivalent: the class uses only documented `uifigure`-family components.

### Converting to `.mlapp` (optional)
- **Keep it as is.** It behaves like an App Designer app, and this is the recommended option.
- **Recreate it in App Designer.** Create a new blank app and lay out the same components, using the property names above. Paste the bodies of the callback and helper methods into App Designer's code view: they only reference `app.<Component>` properties, so they transfer unchanged. The data-driven parts (the gain sub-tabs generated from `gainMetadata`, and the dropdown items from `listScenarios` / `controllerNames`) can stay programmatic: call `createGainsTab(app)` from the App Designer `startupFcn`.

## Components by layout
- Every component sits in a `uigridlayout` cell with `Layout.Row` / `Layout.Column` set, with two exceptions that MATLAB requires:
  - `uitab` objects, whose parent is their tab group;
  - the two radio buttons, which must be direct children of the `uibuttongroup` and therefore use a small `Position` inside the group.
- The figure is created at about 1400×860 px. Both the scenario grid and each gain sub-tab grid are scrollable.
