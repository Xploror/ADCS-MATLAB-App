# ADCS-MATLAB-App — LEO Chaser / ISS Rendezvous (v1.1.0)

An interactive MATLAB/Simulink environment for testing and comparing three 3-axis attitude controllers on a chaser spacecraft in a 400 km LEO orbit approaching the ISS:

| # | Controller | Idea |
|---|---|---|
| 1 | **Robust SMC** (non-adaptive) | Quaternion sliding-mode control with a boundary layer, fixed nominal inertia and a fixed conservative switching gain [Lo & Chen 1995] |
| 2 | **Adaptive SMC** (parametric) | Same sliding surface and gains, but the inertia used in the feedforward is estimated online (Slotine–Li regressor, box projection, freeze-on-saturation anti-windup) |
| 3 | **PD benchmark** | Quaternion PD feedback [Wie & Barba 1985] |

You can change the initial conditions, the reference mode, inertia uncertainty, disturbances, actuator limits, sensor noise and every controller gain from the GUI. You can run one controller or all three, overlay their time histories, and compare metrics (settling time, steady-state error, torque, momentum, saturation time, chattering, inertia-estimate error, pass/fail).

## Requirements
- MATLAB R2021a or newer, with Simulink for the Simulink engine. Parallel Computing Toolbox is optional; with it, runs are parallelised only when a pool is already open.
- Without Simulink, the tool automatically uses the built-in MATLAB **reference engine** (fixed-step RK4 running the same library code).
- The numerical library, tests and examples also run in **GNU Octave 8** on the reference engine. The GUI and the Simulink model are MATLAB-only.

## Quick start (MATLAB)
```matlab
cd ADCS_ComparisonTool
startup_ADCS                 % add paths
build_ADCS_model(true)       % generate models/ADCS_ComparisonHarness.slx + ADCS_Params.sldd (first time)
run_all_tests                % unit tests + Simulink-vs-reference cross-check
ADCS_ComparisonApp           % launch the GUI
```
Scripted use:
```matlab
cfg = scn_adaptive_excitation_scan();          % any scenario from listScenarios()
results = runSimulation(cfg, [1 2 3], 'auto'); % 'auto' | 'simulink' | 'reference'
M = arrayfun(@(r) computeMetrics(r, cfg), results);
```

## Scenarios (`config/scenarios/`)
| Scenario | Purpose |
|---|---|
| `scn_nominal` | 5° error, docking approach (line of sight to the ISS), GG and aero disturbances |
| `scn_inertia_uncertainty_pm25` | True inertia perturbed ±25 % (seeded, physically valid) |
| `scn_large_attitude_error` | 150° slew; momentum saturation |
| `scn_disturbance_stress` | All four disturbances at 4× magnitude |
| `scn_actuator_saturation_stress` | 120° slew with halved torque and momentum limits |
| `scn_sensor_noise` | Attitude and gyro noise plus gyro bias random walk |
| `scn_adaptive_excitation_scan` | Persistently exciting 3-axis scan with ±25 % inertia error: the case where adaptation pays off |

## Folder layout
```
README.md, CLAUDE.md, notes.md, PROJECT_RULES.md, startup_ADCS.m
app/        ADCS_ComparisonApp.m (App Designer-structured GUI), launchADCSApp.m, README_app.md
models/     build_ADCS_model.m (generates .slx/.sldd), pushParamsToDictionary.m, README_models.md
src/        utils, reference, dynamics, disturbances, controllers, sim, analysis, config_io
config/     initDefaults.m, gainMetadata.m, listScenarios.m, gains/, scenarios/, saved/
tests/      run_all_tests.m, test_*.m, run_all_scenarios_smoketest.m, check_model_wiring_repair.m (opt-in)
examples/   run_example_comparison.m
docs/       DESIGN_SPEC.md (interface contract), technical_report.pdf
```

## Verification status (read this)
- **MATLAB R2026a Update 5 with Simulink 26.1 (2026-09-23):**
  - `build_ADCS_model(true)` builds the harness and verifies all 43 wires after grouping;
  - `run_all_tests` gives **9/9 PASS**, including `test_simulink_vs_reference` (Simulink matches the reference engine to about 1e-11°);
  - all 7 scenarios × 3 controllers give identical verdicts on both engines: 20/21 pass, and the single fail (PD on the persistent-excitation scan) is a genuine limitation of PD.
- The numerical library and the reference engine are also verified in GNU Octave 8.4.
- **The GUI has not yet been exercised in MATLAB** (notes.md I9). Please report any GUI issue.
- `tests/check_model_wiring_repair.m` is opt-in. It rebuilds the model with a deliberately injected wiring fault to exercise the builder's self-repair (notes.md I12).

## Documentation
- `CLAUDE.md`: orientation for developers and agents, covering the summary, terminology, and inter- and intra-module dependencies.
- `docs/technical_report.pdf`: models, control-law derivations, architecture, verification results and discussion.
- `notes.md`: all caveats (including the critic's), deviations from the approved plan, research findings, future directions and the verified reference list.
- `docs/DESIGN_SPEC.md`: conventions, equations, struct fields and signatures.
