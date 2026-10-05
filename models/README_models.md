# models/ — Simulink harness

The Simulink model and its data dictionary are **generated** by a script. They are not stored in the repository, because binary `.slx`/`.sldd` files cannot be reviewed or diffed.

## Building the model

```matlab
startup_ADCS;             % adds src/, config/, models/ ... to the path
build_ADCS_model(true);   % true = overwrite an existing model
```

`build_ADCS_model(false)` (or `ensureModelBuilt()`) builds only if `ADCS_ComparisonHarness.slx` is missing. The app and `runSimulation(cfg, k, 'simulink')` call `ensureModelBuilt()` automatically, so a manual build is needed only after you change the builder or want a clean model.

## What gets generated

| File | Content |
|---|---|
| `ADCS_ComparisonHarness.slx` | Fixed-step `ode4` harness (`FixedStep = SC.dt`, `StopTime = SC.t_final`). Signal logging is on (`logsout`, Dataset) and the model returns a single `Simulink.SimulationOutput`. The top level has seven subsystems: `RefGenerator`, `Sensors`, `ControllerSelect`, `ReactionWheelActuator`, `DisturbanceTorques`, `RigidBodyDynamics3DOF`, `TrueErrors` (see `docs/DESIGN_SPEC.md` §7 and the ASCII diagram in the header of `build_ADCS_model.m`). |
| `ADCS_Params.sldd` | Data dictionary, section *Design Data*, with entries `SC`, `RobustGains`, `AdaptiveGains`, `BaselineGains`, `CONTROLLER_SELECT` (defaults from `initDefaults()` → `buildSimParams()`, controller 1). |

Every MATLAB Function block is a thin wrapper that calls one library function in `src/` (for example `referenceAttitude`, `controllerSelect` or `rigidBodyDerivatives`). All the numerics therefore live in `src/`, which the reference engine shares. The quaternion normalisation (`quatNormalizeState`) is its own block to avoid an algebraic loop. The builder sets `AlgebraicLoopMsg = error`, so a loop would be reported at once.

Every MATLAB Function port has a **fixed size and type** (`portSize()` in the builder). With inherited sizes, R2026a could analyse a block before its input sizes had propagated around the feedback loops. After grouping the blocks with `createSubsystem`, the builder **traces every wire of its wiring table `W`** across the subsystem boundaries and repairs any mis-wire (seen occasionally in R2026a). It then re-verifies all wires and scans for open ports. The build prints `N wires verified after grouping (M repaired)`, and its second output `info` returns the same counts. `tests/check_model_wiring_repair.m` (opt-in) exercises the repair path through the test hook `build_ADCS_model(true, true)`.

Logged signals (line names equal the `res` field names): `q, w, q_r, w_r, q_e, w_e, att_err_deg, s, tau_cmd, tau_rw, h_w, theta_hat, tau_d, tau_gg, tau_aero, tau_srp, tau_mag, qnorm, sat_flags`.

## How parameters are supplied

1. **Batch or app runs** (`runSimulinkBatch`): each run uses a `Simulink.SimulationInput` whose `setVariable` values for `SC`, `RobustGains`, `AdaptiveGains`, `BaselineGains` and `CONTROLLER_SELECT` override the dictionary. `StopTime`/`FixedStep` are set numerically. The dictionary is never modified, so parallel runs (`parsim`) do not interfere with each other. `parsim` is used only when there is more than one run, the Parallel Computing Toolbox is licensed and a parallel pool is **already open** (`gcp('nocreate')` is non-empty); `runSimulinkBatch` never starts a pool. Otherwise the runs are sequential (`sim`). If `parsim` throws, or any individual run reports an `ErrorMessage`, the batch is re-run sequentially with a warning (`runSimulinkBatch:parsimFailed` / `runSimulinkBatch:parsimRunError`).
2. **Interactive runs** (the Run button in Simulink): push a configuration into the dictionary first:
   ```matlab
   cfg = scn_nominal();
   pushParamsToDictionary(cfg, 2);   % 2 = Adaptive SMC
   open_system('ADCS_ComparisonHarness'); sim('ADCS_ComparisonHarness');
   ```

## Inspecting the generated model

```matlab
startup_ADCS; ensureModelBuilt(); open_system('ADCS_ComparisonHarness');
```
Double-click a subsystem, then a MATLAB Function block, to see the wrapper code. Use *Modeling → Model Data Editor → Parameters* or open `ADCS_Params.sldd` (Model Explorer) to see the parameter values. After a run, `out.logsout` can be browsed in the Simulation Data Inspector (`Simulink.sdi.view`).

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| *"Variable 'SC' is defined in both the base workspace and the data dictionary"* (or a similar message for another parameter name) | A variable with a parameter name exists in the base workspace and the model can read the base workspace. Run `clear SC RobustGains AdaptiveGains BaselineGains CONTROLLER_SELECT`, or turn off base-workspace access: `set_param('ADCS_ComparisonHarness','EnableAccessToBaseWorkspace','off')` (the builder tries to do this; older releases may not have the parameter). |
| MATLAB Function block code-generation errors such as *"Undefined function or variable 'referenceAttitude'"* | `src/` is not on the path when the model is compiled. Run `startup_ADCS` first. For `parsim`, `runSimulinkBatch` adds the project path on every worker through `SetupFcn`. |
| Code-generation errors *inside* a library function (e.g. size mismatch, a non-constant struct field, a cell array) | The function in `src/` is not codegen-compatible (see PROJECT_RULES R3). Fix the library function, not the wrapper. To reproduce outside Simulink, run `codegen` or the MATLAB Function block *Build* (Ctrl+B) on the model. |
| *"Cannot find data dictionary 'ADCS_Params.sldd'"* | `models/` is not on the path. Run `startup_ADCS` or `addpath models`, or rebuild with `build_ADCS_model(true)`. |
| *"Data dictionary ... is open with unsaved changes"* during a rebuild | The builder discards open dictionary changes (`Simulink.data.dictionary.closeAll('ADCS_Params.sldd','-discard')`). If this still fails, close the model and Model Explorer, then run `Simulink.data.dictionary.closeAll` by hand. |
| *"Logged signal 'x' not found"* from `extractSimulinkResults` | The model is out of date or was edited by hand. Rebuild with `build_ADCS_model(true)`. The error message lists the logged names that were found. |
| Algebraic-loop error | Someone merged the quaternion normalisation into the dynamics block or added a direct-feedthrough path around the integrators. Rebuild from the script. |
| *"Error due to multiple causes"* from `runSimulinkBatch` | Since v1.1.0 the message lists every nested cause (`collectCauses`). A "Port width mismatch" or "Incorrect dimensions" error in a MATLAB Function block usually means a new port without a `portSize` entry, or a mis-wire. Rebuild with `build_ADCS_model(true)` and check the "wires verified" line. |
| `build_ADCS_model:wiring` or `build_ADCS_model:openPorts` during a build | The post-grouping verification found a wire it could not repair, or an unconnected port. Report the printed wire. As a workaround, rebuild; the `createSubsystem` fault is non-deterministic. |
| Stale compiled code after changing `src/` | Clear the cache with `Simulink.data.dictionary.closeAll; bdclose all; rmdir('slprj','s')` (in the folder where it was created), then run again. |
