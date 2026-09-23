# notes.md — ADCS Controller Comparison Tool

| Field | Value |
|---|---|
| Project | Robust, Adaptive & Benchmark Attitude Controller Comparison Tool (LEO chaser → ISS, 3-axis attitude) |
| Version | v1.1.0 (first MATLAB/Simulink execution: Simulink generator fixes D12–D13, full MATLAB verification; guardrail rule R9 revised) |
| Last synced | 2026-09-23 |
| Phase | Implemented and verified in MATLAB R2026a Update 5 + Simulink 26.1 (both engines) and GNU Octave 8.4 (reference engine). GUI feature checks not yet executed (I9) |

> There is exactly one `notes.md` per project version. It is updated in place and never copied. Citations `[n]` refer to §6 References. Status legend: **OPEN** = not addressed; **MITIGATED** = reduced and documented; **RESOLVED** = fixed and verified.

---

## 1. Caveats

### 1.1 Critic agent: plan review (pre-approval)
| # | Caveat | Status | Resolution |
|---|---|---|---|
| C1 | The original wheel pair (0.20 N·m, 0.015 N·m·s) was physically inconsistent: momentum saturation in about 0.075 s | RESOLVED | 0.15 N·m / 15 N·m·s, described as an HR12-25-class wheel, derated [14]. The HR12 datasheet gives 0.1–0.23 N·m nominal torque and 12/25/50 N·m·s options |
| C2 | The gain-bus fields did not match the control laws | RESOLVED | Gain structs are defined in DESIGN_SPEC §4 to match the laws exactly |
| C3 | A scalar gain-adaptive SMC adapts only a disturbance bound, not the perturbed inertia | RESOLVED | Parametric, inertia-estimating adaptive SMC [4][5] |
| C4 | Boundary-layer SMC [1] is a baseline, not the 2020s frontier (finite-, fixed- and prescribed-time SMC [17]) | MITIGATED | Framed as an industrially relevant baseline. Terminal SMC is listed in §4. Note that [17] has a published comment [22] |
| C5 | Three separate controller models plus a Variant Subsystem were overbuilt | RESOLVED | One harness with a `ControllerSelect` subsystem |
| C6 | Aerodynamic torque magnitude depends on the CP–CG offset and on solar activity. [13] supports the offset dependence only; for GRACE it gives GG ~1e-5 N·m versus aero ~1e-7 N·m | MITIGATED | Density default 3e-12 kg/m³ (USSA-76 gives 2.8e-12 [23]); the solar-cycle range is about 5e-13 to 2e-11 kg/m³ [26]. A magnitude-scale slider covers stress tests. For this project's larger chaser geometry the model gives aero ~1.9e-4 N·m at 45° misalignment |
| C7 | No save or reload of configurations | RESOLVED | `saveConfig`/`loadConfig` and GUI Save/Load |

### 1.2 Critic agent: code review (post-implementation)
| # | Finding | Status | Resolution |
|---|---|---|---|
| R1 (MAJOR) | `parsim` could start a parallel pool (30–90 s) and did not fall back when a single run failed | RESOLVED | parsim is used only if a pool is already open. Per-run errors fall back to sequential `sim` |
| R2 (MAJOR) | `num2str(...,'%.10g')` truncated FixedStep/StopTime, which could cause a sample-time mismatch with `SC.dt` | RESOLVED | `sprintf('%.17g', ...)` |
| R3 | The GUI accepted φ ≤ 0, negative K/η/Γ, proj_frac ∉ (0,1), and t_final not a multiple of dt | RESOLVED | `validateCfg` checks added |
| R4 | A user θ̂(0) could lie outside the projection box, so the two engines started differently | RESOLVED | θ̂(0) is clipped into the box in `buildSimParams` |
| R5 | `config/saved/` was missing | RESOLVED | Folder and `.gitkeep` created; the GUI creates the folder if it is absent |
| R6 | Some functions lacked section breaks or config-variable comments (PROJECT_RULES R2) | RESOLVED | Added across src, models, app and tests |
| R7 | Items verified correct (do not "fix"): Stateflow `Script` idiom and parameter port ordering, `createSubsystem` usage (note: the call itself is correct, but in R2026a it occasionally mis-wires branched lines, so the builder now verifies the result; see I12), Integrator/Random Number parameter names, `setVariable` overriding data-dictionary entries, and the logged-data orientation [n×1×N] | — | Listed for future maintainers |

### 1.3 Literature-review agent
| # | Caveat | Status | Resolution |
|---|---|---|---|
| L1 | No public vehicle-specific inertia tensor exists for Cygnus, Dragon, Progress or HTV | MITIGATED | The default J_nom = [1200 100 −200; 100 2200 300; −200 300 3100] kg·m² is widely attributed to Wie, Weiss & Arapostathis [6] (see also [7]). **The attribution could not be confirmed against the primary text** because the full text was paywalled during verification. J_nom is fully editable in the GUI |
| L2 | Terminal/finite-time SMC was not selected | MITIGATED | See C4 |
| L3 | Parameter convergence needs persistent excitation (PE) [9] | RESOLVED (characterised) | Confirmed experimentally (§3.2). The new PE scenario `scn_adaptive_excitation_scan` demonstrates convergence |
| L4 | σ-modification versus projection | MITIGATED | Box projection plus freeze-on-saturation anti-windup (D9), in the spirit of [19] |

### 1.4 Project-planning agent
| # | Caveat | Status | Resolution |
|---|---|---|---|
| P1 | Sensors are ideal by default | MITIGATED | Toggleable noise model (`scn_sensor_noise`) |
| P2 | Wheel momentum is not desaturated | MITIGATED | Out of scope for minute-scale windows; momentum saturation is modelled, flagged and measured |
| P3 | No flexible-body dynamics and no 6DOF coupling | MITIGATED | Scope decision (plan §2) |

### 1.5 Implementation and verification (development agents, orchestrator)
| # | Caveat | Status | Resolution |
|---|---|---|---|
| I1 | MATLAB and Simulink were unavailable in the development environment. The `.slx`/`.sldd` generator, the Simulink batch runner and the GUI were only statically reviewed and mock-tested in Octave | **RESOLVED** (v1.1.0) for everything except the GUI | Executed through the MATLAB MCP connector (R2026a Update 5, Simulink 26.1) on 2026-09-23. The first run found three generator/runner defects (I11–I13), which were fixed (D12, D13). Afterwards the model builds, `run_all_tests` gives **9/9 PASS**, and `test_simulink_vs_reference` agrees to about 1e-11° (tolerance 0.02°). All 7 scenarios × 3 controllers give identical verdicts on both engines (§5.2). GUI checks were excluded by the user (I9) |
| I2 | Wheel gyroscopic coupling −ω×h_w was absent from the planned EOM | RESOLVED | Added to the plant. All controllers share a +ω×h_w compensation, so the comparison stays fair |
| I3 | The shortest-path error-quaternion sign switch can chatter near 180° errors | OPEN | Documented; hysteretic hybrid switching [18] is in §4 |
| I4 | **Adaptation windup under saturation**: in the 150° and 120° slews (500–650 s of saturation), the unfrozen law drove θ̂ onto the projection bounds (78–80 % error) and doubled the steady-state error | RESOLVED | Freeze-on-saturation (D9). θ̂ error 78 % → 1.4 % and 80 % → 0.3 %; steady-state error 0.102° → 0.046° and 0.095° → 0.046° [19][20] |
| I5 | **Adaptation gain versus excitation trade-off**: with Γ = 1e9–5e9 and no PE, θ̂ drifts 15–49 % from the truth in every non-PE scenario, even when J_true = J_nom, because it absorbs disturbance torques. Settling becomes faster (81 vs 119 s) | MITIGATED | Default Γ = 2e7 (conservative, no drift). The PE showcase scenario uses Γ = 5e9, where θ̂ converges to about 3 % and tracking error falls about 5× versus robust SMC (§3.2) |
| I6 | The spec's gyro-bias random walk was missing the 1/√dt factor for step-held noise | RESOLVED | `b_dot = σ/√dt · n` in both engines |
| I7 | The spec's torque-saturation flag `|τ_cmd| > τ_max` could never fire on a clamped command | RESOLVED | `≥ τ_max(1−1e-9)` |
| I8 | Random seeds differ between MATLAB (`rng`) and Octave (`randn('state')`), so J_true for ±25 % and the noise realisations differ across platforms | OPEN (confirmed in v1.1.0) | Confirmed in MATLAB. With the same seed 42, the PE-scan metrics differ between Octave and MATLAB (for example PD e_ss 1.138° vs 1.461° and θ̂ err 2.9 % vs 1.8 %) because J_true differs. **MATLAB R2026a values (§5) are authoritative**; the Octave values are kept only as a record. Future fix: a portable in-house generator (§4 item 9) |
| I9 | Plotting (`plotResultsOnAxes`, the example figure) could not be rendered in the sandbox because Octave's font renderer was broken | OPEN | The GUI and plotting checks were deliberately excluded from the v1.1.0 MATLAB validation at the user's request, so they are still unexecuted. Next step: open `ADCS_ComparisonApp` in the connector's single MATLAB window (PROJECT_RULES R8) and run the checklist in `app/README_app.md` |
| I10 | Solar-constant and geomagnetic defaults were outdated | RESOLVED | P_srp 4.54e-6 N/m² (TSI 1361 W/m² [25]); B0 2.97e-5 T (IGRF-14, epoch 2025 [24]) |
| I11 | **MATLAB Function block sizes (found in MATLAB).** With inherited (−1) port sizes, Simulink analysed `TrueErr` before its input sizes had propagated around the feedback loops and reported "Incorrect dimensions for matrix multiplication" in `trueErrors` | RESOLVED | D12: every MATLAB Function port now has a fixed size and type (`portSize()`), and the Random Number blocks emit [3×1] columns |
| I12 | **`createSubsystem` mis-wiring (found in MATLAB, non-deterministic).** After grouping, `TrueErr` input `w_r` (in one build) and `w` (in another) was connected to the `q_r` signal. The flat model was correct; the fault came from `Simulink.BlockDiagram.createSubsystem` | RESOLVED | D12: after grouping, every one of the 43 wires is traced across the subsystem boundaries and compared with the wiring table. Any mis-wire is repaired without leaving orphans, then all wires are re-verified and an open-port scan runs. Later clean builds: 43/43 verified, 0 repaired. The repair path is exercised by the opt-in `tests/check_model_wiring_repair.m`, which injects the fault (critic F1/F2) |
| I13 | Simulink compile errors surfaced only as "Error due to multiple causes", which hid the actual cause | RESOLVED | D13: `runSimulinkBatch` now reports every nested cause for thrown errors, per-run `ErrorDiagnostic`s and parsim failures, and keeps the original exception as its cause |
| I14 | The sensor-noise realisations differ between the two engines, even in MATLAB. Simulink uses Random Number blocks; the reference engine uses `randn` | ACCEPTED (by design) | The scenario is compared statistically. Worst-case attitude difference is 0.013°; e_ss is 0.050/0.051/0.101° (Simulink) vs 0.040/0.041/0.090° (reference); all runs pass on both engines. The deterministic scenarios agree to ≤ 2.3e-5° |
| I15 | Validation-process caveats (orchestrator): the MCP bridge has a fixed 60 s call limit; the MATLAB session was slow (the first `load_system` took 349 s); and the device commit bridge served a stale copy when a staged file name was reused | MITIGATED | Long work runs as in-session one-shot timer jobs with file-based status and logs (PROJECT_RULES R9). Every commit uses a unique staged name and is verified by checksum; a full local-versus-device checksum audit found 0 differences |

### 1.6 Critic agent: MATLAB-execution code review (v1.1.0)
This review covered the D12/D13 changes after the first MATLAB run. Verdict: *accept with minor fixes*. The critic confirmed every `portSize()` entry against the wrapped library functions, and confirmed the Simulink API usage.

| # | Finding | Status | Resolution |
|---|---|---|---|
| F1 (MAJOR) | The first repair version left an orphaned Inport and a visibly wrong top-level line, which the build's own checks could not see | RESOLVED | Repair now re-points the top-level line when the Inport is dedicated, uses a new Inport only when the Inport is shared, and terminates an output left open |
| F2 (MAJOR) | The repair path had never run and is non-deterministic in nature | RESOLVED | Test hook `build_ADCS_model(true, true)` and opt-in `tests/check_model_wiring_repair.m` (see §5.2) |
| F3 | The post-grouping check ignored subsystem boundary ports | RESOLVED | `checkNoOpenPorts` scans all ports at depth 1 and 2 |
| F4 | `delete_line` on a line handle could delete a whole branch subtree | RESOLVED | Uses the string form `delete_line(sys, 'src/p', 'dst/q')` |
| F5 | An empty `find_system` result gave a cryptic index error | RESOLVED | The trace returns "open", which is reported as a wiring error |
| F6 | Nested causes were missing for array runs (`ErrorMessage`) and parsim | RESOLVED | `collectCauses` is applied to `ErrorDiagnostic.Diagnostic` and to the parsim catch |
| F7 | `traceLeafSource` lacked section breaks (R2) | RESOLVED | Added |
| F8 | notes.md and CLAUDE.md had not been synced | RESOLVED | This v1.1.0 sync |
| F9, F10 (NIT) | An empty identifier printed as `[]`; the original exception was dropped | RESOLVED | Conditional prefix; `addCause` plus `throw` |

---

## 2. Deviations from the approved plan
| # | Plan | Implementation | Reason / impact |
|---|---|---|---|
| D1 | `.slx`, `.sldd` and `.mlapp` delivered as files | `models/build_ADCS_model.m` generates `.slx`/`.sldd`. The app is an App Designer-structured class (`matlab.apps.AppBase`) | Binary MATLAB containers cannot be authored or verified without MATLAB. Functionality is identical and the text sources are reviewable |
| D2 | Typed `Simulink.Bus` gain objects | Numeric struct parameters, with GUI fields generated from `gainMetadata()` | Same single source of truth, fewer failure points |
| D3 | Λ, K as general matrices | Diagonal vectors | Easier to use in the GUI |
| D4 | Controller 1 feedforward `ω×Jω + J[…]` | Regressor form `Y θ_nom` | Controller 2 differs only by θ̂ ← θ_nom (verified by `test_adaptive_ablation_identity` to 1e-12) |
| D5 | Lookup-table range profile | Analytic exponential closing profile | Smooth ω̇_r |
| D6 | `nominal.m` etc. | `scn_*.m` | `nominal` collides with a Statistics Toolbox class |
| D7 | — | Reference engine `simulateADCS_ref.m` (RK4) | Verification without Simulink, Simulink cross-check, and fallback when Simulink is unlicensed |
| D8 | Normalisation block after the integrator | Constraint-stabilised kinematics plus a separate normalisation block | Continuous states cannot be renormalised in place; keeping the block separate avoids an algebraic loop. Max ‖q‖ deviation 2.4e-10 |
| D9 | Projection-only adaptation | Adds freeze-on-saturation anti-windup (`AG.freeze_on_sat`, default 1) | Finding I4 |
| D10 | 5 test cases (+ optional noise) | Adds reference mode 3 (attitude scan) and the 7th scenario `scn_adaptive_excitation_scan` | Findings L3/I5: the plan's headline expectation, that θ̂ converges in the ±25 % docking case, does not hold without PE. This scenario shows when adaptation pays off |
| D11 (v1.0.1) | — | Added `CLAUDE.md`, this parameter-provenance section, and PROJECT_RULES R5–R7 | Standing-rules revision 2 (CLAUDE.md duty, parameter-default ladder, environment escalation). Documentation only; no code or results changed |
| D12 (v1.1.0) | Inherited MATLAB Function port sizes; grouping trusted to preserve wiring | `build_ADCS_model.m`: (a) fixed port sizes and types via `portSize()`; (b) column Random Number parameters with `VectorParams1D` off; (c) a wiring table `W` that is verified across boundaries after grouping, with orphan-free repair (re-pointing the top-level line, or a dedicated Inport when the Inport is shared); (d) an open-port scan; (e) a test hook `build_ADCS_model(true, true)` plus `tests/check_model_wiring_repair.m`. New second output `info` (nWires, nRepaired) | Findings I11 and I12 and critic findings F1–F5 and F7. Numerics unchanged |
| D13 (v1.1.0) | Flat Simulink error messages | `runSimulinkBatch.m`: `collectCauses` expands nested causes (thrown errors, per-run `ErrorDiagnostic`, parsim) and keeps the original exception | Finding I13; critic F6, F9 and F10 |
| D14 (v1.1.0) | PROJECT_RULES R9: wait 100 s, then 5 min, then exit | R9 revised by the user: check the status first, keep waiting while it is BUSY, and never proceed until it is non-busy. No fixed start wait and no time limit | User instruction of 2026-09-23. Process rule only |

---

## 2b. Parameter provenance (standing Rule 3 ladder: literature → wider search → test-tuned default)
| Parameter (default) | Source type | Evidence / range tested | Status |
|---|---|---|---|
| Orbit 400 km, i = 51.64° | Literature / public data | ISS orbit; n = 1.1314e-3 rad/s recomputed | VERIFIED |
| J_nom = [1200 100 −200; 100 2200 300; −200 300 3100] kg·m² | Literature [6][7] | Widely attributed to [6]; the primary text was not accessible | Attribution UNVERIFIED (L1); editable |
| Wheel τ_max 0.15 N·m, h_max 15 N·m·s | Datasheet [14] | HR12 nominal 0.1–0.23 N·m; 12/25/50 N·m·s options | VERIFIED (derated HR12-25 class) |
| ρ = 3e-12 kg/m³ | Wider search [23][26] | USSA-76 2.8e-12; solar-cycle range ≈ 5e-13–2e-11 | VERIFIED |
| P_srp = 4.54e-6 N/m² | Wider search [25] | TSI 1361 W/m² / c | VERIFIED |
| B0 = 2.97e-5 T | Wider search [24] | IGRF-14, epoch 2025 | VERIFIED |
| m_res = 0.5 A·m² per axis | Literature range (lit-review agent, [15]) | Typical 0.1–1 A·m² | Literature range, not vehicle-specific |
| Cd = 2.2 | Textbook value [15] | Standard free-molecular flat-plate value | Textbook |
| A_aero = A_srp = 10 m², r_cp offsets ~0.05–0.1 m, refl_q = 0.6 | Engineering default (no vehicle data found) | Gives aero ≈ 1.9e-4 N·m and SRP ≈ 8e-6 N·m at 45°. Sensitivity is covered by the `dist_scale` 4× stress scenario (all SMC runs pass) | Default, validated by the stress test |
| SMC Λ = 0.05 s⁻¹, K = diag(J_nom)/20, η = 2e-3 N·m, φ = 1e-3 rad/s | Test-tuned default ([1] gives the structure, not values) | Checked across all 7 scenarios: all SMC runs pass, torque within limits, chatter ≈ 1e-3 N·m/s | Validated default |
| Γ = 2e7 (default); 5e9 (PE scenario) | Test-tuned default | Swept 2e6, 2e7, 2e8, 1e9, 2e9 and 5e9. High Γ drifts without PE (I5) | Validated, scenario-dependent |
| proj_frac = 0.6 | Engineering default | With freeze on, bounds are never reached in the final runs (θ̂ err ≤ 13.9 %) | Validated default |
| PD ωn = 0.03 rad/s, ζ = 0.9 | Textbook rule [7] + test choice of ωn | ωn chosen so the nominal peak torque stays at about the limit (0.14 N·m) | Validated default |
| Scan 3°, 0.015/0.020/0.025 rad/s | Designed default | Torque within limits (peak 0.15 N·m, 5 s saturated); θ̂ converges to 2.9 % | Validated default |
| dt = 0.1 s, k_norm = 1, ref_fd_h = 0.5 s | Numerical defaults, test-validated | Norm deviation 2.4e-10; reference FD error < 1e-7 (unit test) | Validated |
| Sensor noise 0.005°, 0.001°/s, bias RW 1e-5 °/s/√s | Engineering default (typical star-tracker / tactical gyro grade) | Only exercised in `scn_sensor_noise` (all pass). No citation found | Default, open for a citation (see §4) |

---

## 3. Key research findings (MATLAB R2026a, Simulink and reference engines identical; full tables in docs/technical_report.pdf)

### 3.1 Controller ranking
In every scenario, robust and adaptive SMC reach about half the PD benchmark's steady-state error (0.030–0.164° versus 0.073–0.264°). On the PE scan the adaptive law reaches about 1/76 of the PD error (0.019° vs 1.461°), and about 1/8 of the robust SMC error. PD settles slightly earlier (110 s versus 119 s nominal) because of its larger proportional authority. Large slews are momentum-limited (h_max/J ≈ 0.005 rad/s) regardless of the controller. The per-scenario table is in §5 and in the technical report.

### 3.2 When does adaptation pay off?
- **Without PE** (docking approach, where the LVLH reference is nearly stationary), adaptive and robust SMC are practically identical. The high-gain sliding feedback already rejects the ±25 % inertia error, and the small inertial torques give the estimator little information [9].
- **With PE** (a 3° three-axis scan at 0.015/0.020/0.025 rad/s, ±25 % inertia error): with Γ = 5e9, the adaptive tracking error is 0.019° against 0.150° for robust SMC (about 8× lower), and θ̂ converges to 1.8 %. PD, having no feedforward, is about 1.5° off. (In Octave, with a different J_true draw (I8), the figures were 0.019 / 0.103 / 1.138° and 2.9 %. The conclusion is unchanged.)
- **The adaptation gain must match the excitation** (I5). High Γ without PE causes drift, so Γ is a scenario-level choice. The GUI exposes it.

---

## 4. Future development directions
1. Terminal, fixed-time or prescribed-time SMC as a fourth controller [17]; check the comment [22].
2. Hysteretic hybrid quaternion feedback to remove unwinding and 180° chattering [18].
3. Excitation-aware adaptation: a PE monitor, composite or concurrent-learning adaptation, or a dead-zone combined with σ-modification, so a single Γ works across scenarios [9].
4. Input-constrained adaptive laws that modify the update with the saturation error instead of freezing it [19][20].
5. Wheel momentum management (magnetorquers) and a 4-wheel pyramid with fault injection.
6. Coupled 6DOF relative motion (Clohessy–Wiltshire/Hill) with a translational controller [8].
7. An MEKF sensor-fusion front end [21] instead of direct noisy measurements.
8. Monte Carlo robustness envelopes over inertia, disturbances, ICs and solar activity [26].
9. A portable, platform-independent pseudo-random generator for J_true and the sensor noise, shared by both engines. This would remove I8 and I14 and make the Octave, MATLAB and Simulink results bit-comparable.
10. Build the Simulink hierarchy directly (blocks created inside their subsystems) rather than grouping afterwards with `createSubsystem`. This removes the I12 failure mode at its source.

---

## 5. Verification record

### 5.1 Unit tests
- **MATLAB R2026a Update 5 + Simulink 26.1 (2026-09-23): 9 PASS, 0 FAIL, 0 SKIP**, including the Simulink acceptance gate `test_simulink_vs_reference`. On nominal, 150 s, max |Δ att_err| is 1.7e-11 / 1.2e-11 / 2.7e-11° (robust / adaptive / PD), and the relative difference in RMS(τ_rw) is ≤ 3e-13 %.
- GNU Octave 8.4 (2026-09-22): 8 PASS, 0 FAIL, 1 SKIP (the Simulink test).
- Opt-in `tests/check_model_wiring_repair.m` (MATLAB, 2026-09-23): **PASSED**. The injected mis-wire `TrueErr.w_r ← q_r` was detected and repaired (43 wires verified, 1 repaired), and `test_simulink_vs_reference` passed on the repaired model. The clean rebuild then hit the genuine `createSubsystem` fault on its own (`QNorm.q → TrueErr.q`) and repaired it, which confirms I12 in practice. `run_all_tests` gave 9/9 PASS on that shipped model.

### 5.2 Scenario runs: 7 scenarios × 3 controllers on both engines (MATLAB R2026a)
**20/21 PASS on both engines, and all 21 verdicts agree.** The single FAIL is PD on the PE scan: without model feedforward it lags the moving reference by about 1.5°, which is a genuine limitation, not a bug. On the deterministic scenarios, Simulink matches the reference engine to ≤ 2.3e-10° (≤ 2.3e-5° in the momentum-saturated slews, from the limited-integrator vs post-step clamp). For sensor noise the RNG streams differ (I14). Wall-clock time per run in Simulink is about 1.3–2.3 s (400–2000 s simulated).

Legend: t_s = settling time [s] (0.5° band); e_ss = mean steady-state error [deg]; τ_pk = peak wheel torque [N·m]; h_pk = peak wheel momentum [N·m·s]; θ̂ err = final inertia-estimate error [%]; max|Δ| = max |att_err_Simulink − att_err_reference| [deg]. Controllers are listed Robust / Adaptive / PD in every row.

| Scenario | t_s | e_ss | τ_pk | h_pk | θ̂ err | max\|Δ\| | Pass |
|---|---|---|---|---|---|---|---|
| Nominal | 119 / 119 / 110 | 0.040 / 0.041 / 0.090 | 0.150 / 0.150 / 0.141 | 2.0 / 2.0 / 1.9 | – / 0.5 / – | ≤ 1.5e-10 | P / P / P |
| Inertia ±25 % | 119 / 118 / 111 | 0.030 / 0.031 / 0.073 | 0.150 / 0.150 / 0.141 | 2.2 / 2.2 / 2.0 | – / 13.8 / – | ≤ 2.3e-10 | P / P / P |
| Large error 150° | 591 / 589 / 536 | 0.046 / 0.046 / 0.085 | 0.150 ×3 | 15.0 ×3 | – / 1.4 / – | ≤ 2.3e-5 | P / P / P |
| Disturbance 4× | 119 / 118 / 111 | 0.163 / 0.164 / 0.264 | 0.150 / 0.150 / 0.141 | 2.6 ×3 | – / 0.5 / – | ≤ 3.7e-11 | P / P / P |
| Actuator saturation | 876 / 876 / 833 | 0.046 / 0.046 / 0.085 | 0.075 ×3 | 7.5 ×3 | – / 0.3 / – | ≤ 6.5e-6 | P / P / P |
| Sensor noise (Simulink) | 119 / 119 / 110 | 0.050 / 0.051 / 0.101 | 0.150 / 0.150 / 0.142 | 2.0 / 2.0 / 1.9 | – / 0.5 / – | 0.013 (RNG, I14) | P / P / P |
| PE scan ±25 % | 132 / 144 / – | 0.150 / **0.019** / 1.461 | 0.150 / 0.150 / 0.144 | 9.6 / 9.6 / 10.4 | – / 1.8 / – | ≤ 5.8e-10 | P / P / **F** |

Octave record (v1.0.x, reference engine, different J_true draw per I8): the values are identical except for the seeded scenarios. Inertia ±25 %: e_ss 0.042 / 0.042 / 0.089, θ̂ 13.9 %. PE scan: e_ss 0.103 / 0.019 / 1.138, θ̂ 2.9 %. Sensor noise: e_ss 0.049 / 0.050 / 0.099.

Anti-windup ablation (150° slew, adaptive): with the freeze off, e_ss = 0.102° and θ̂ err = 78.0 %; with the freeze on, e_ss = 0.046° and θ̂ err = 1.4 %. The Octave and MATLAB/Simulink results are identical here (Simulink, 2026-09-23: 0.1020° / 78.02 % versus 0.0462° / 1.38 %).

---

## 6. References
[1] S.-C. Lo and Y.-P. Chen, "Smooth sliding-mode control for spacecraft attitude tracking maneuvers," *J. Guidance, Control, and Dynamics*, vol. 18, no. 6, pp. 1345–1349, 1995. doi:10.2514/3.21551 — VERIFIED
[2] B. Wie and P. M. Barba, "Quaternion feedback for spacecraft large angle maneuvers," *J. Guidance, Control, and Dynamics*, vol. 8, no. 3, pp. 360–365, 1985. doi:10.2514/3.19988 — VERIFIED
[3] J. T.-Y. Wen and K. Kreutz-Delgado, "The attitude control problem," *IEEE Trans. Automatic Control*, vol. 36, no. 10, pp. 1148–1162, 1991. doi:10.1109/9.90228 — VERIFIED
[4] J.-J. E. Slotine and W. Li, "On the adaptive control of robot manipulators," *Int. J. Robotics Research*, vol. 6, no. 3, pp. 49–59, 1987. doi:10.1177/027836498700600303 — VERIFIED
[5] J.-J. E. Slotine and M. D. Di Benedetto, "Hamiltonian adaptive control of spacecraft," *IEEE Trans. Automatic Control*, vol. 35, no. 7, pp. 848–852, 1990. doi:10.1109/9.57028 — VERIFIED
[6] B. Wie, H. Weiss, and A. Arapostathis, "Quaternion feedback regulator for spacecraft eigenaxis rotations," *J. Guidance, Control, and Dynamics*, vol. 12, no. 3, pp. 375–380, 1989. doi:10.2514/3.20418 — VERIFIED (bibliographic); inertia-matrix attribution UNVERIFIED (L1)
[7] B. Wie, *Space Vehicle Dynamics and Control*, 2nd ed., AIAA Education Series. Reston, VA: AIAA, 2008. ISBN 978-1-56347-953-3. doi:10.2514/4.860119 — VERIFIED
[8] H. Schaub and J. L. Junkins, *Analytical Mechanics of Space Systems*, 4th ed., AIAA Education Series. Reston, VA: AIAA, 2018. ISBN 978-1-62410-521-0. doi:10.2514/4.105210 — VERIFIED
[9] P. A. Ioannou and J. Sun, *Robust Adaptive Control*. Upper Saddle River, NJ: PTR Prentice-Hall, 1996; reprinted Dover, 2012 (ISBN 978-0-486-49817-1) — VERIFIED
[10] S. M. Amrr, A. Chakravarty, M. M. Alam, A. A. Algethami, and M. Nabi, "Efficient event-based adaptive sliding mode control for spacecraft attitude stabilization," *J. Guidance, Control, and Dynamics*, vol. 45, no. 7, pp. 1328–1336, 2022. doi:10.2514/1.G006080 — VERIFIED
[11] B. Huo, M. Du, and Z. Yan, "Adaptive sliding mode attitude tracking control for rigid spacecraft considering the unwinding problem," *Mathematics*, vol. 11, no. 20, art. 4372, 2023. doi:10.3390/math11204372 — VERIFIED
[12] U. Javaid, Z. Zhen, Y. Xue, and S. Ijaz, "Robust adaptive attitude control of flexible spacecraft using a sliding mode disturbance observer," *Proc. IMechE, Part G: J. Aerospace Engineering*, vol. 236, no. 11, pp. 2235–2253, 2022. doi:10.1177/09544100211055318 — VERIFIED
[13] M. Tolstoj, "Analysis of Disturbance Torques on Satellites in Low-Earth Orbit Based Upon GRACE," Master's thesis, Dept. of Flight System Dynamics, RWTH Aachen University (carried out at DLR GSOC), 2017. https://elib.dlr.de/128377/ — VERIFIED
[14] Honeywell International Inc., "Constellation Series Reaction Wheels (HR12, HR14, HR16)," datasheet N61-0045-001-001, Dec. 2003 — VERIFIED
[15] J. R. Wertz (ed.), *Spacecraft Attitude Determination and Control*, Astrophysics and Space Science Library vol. 73. Dordrecht: D. Reidel, 1978. doi:10.1007/978-94-009-9907-7 — VERIFIED
[16] S. W. Shepperd, "Quaternion from rotation matrix," *J. Guidance and Control*, vol. 1, no. 3, pp. 223–224, 1978. doi:10.2514/3.55767b — VERIFIED
[17] R.-Q. Dong, A.-G. Wu, Y. Zhang, G.-R. Duan, and B. Li, "Anti-unwinding terminal sliding mode attitude tracking control for rigid spacecraft," *Automatica*, vol. 145, art. 110567, 2022. doi:10.1016/j.automatica.2022.110567 — VERIFIED
[18] C. G. Mayhew, R. G. Sanfelice, and A. R. Teel, "Quaternion-based hybrid control for robust global attitude tracking," *IEEE Trans. Automatic Control*, vol. 56, no. 11, pp. 2555–2566, 2011. doi:10.1109/TAC.2011.2108490 — VERIFIED
[19] S. P. Karason and A. M. Annaswamy, "Adaptive control in the presence of input constraints," *IEEE Trans. Automatic Control*, vol. 39, no. 11, pp. 2325–2330, 1994. doi:10.1109/9.333787 — VERIFIED
[20] J. D. Bošković, S.-M. Li, and R. K. Mehra, "Robust adaptive variable structure control of spacecraft under control input saturation," *J. Guidance, Control, and Dynamics*, vol. 24, no. 1, pp. 14–22, 2001. doi:10.2514/2.4704 — VERIFIED
[21] F. L. Markley and J. L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*. New York: Springer, 2014. doi:10.1007/978-1-4939-0802-8 — VERIFIED (DOI); series details unverified
[22] Y. Su, "Comments on 'Anti-unwinding terminal sliding mode attitude tracking control for rigid spacecraft'," *Automatica*, art. 111205, 2023. doi:10.1016/j.automatica.2023.111205 — VERIFIED
[23] NOAA, NASA, and USAF, *U.S. Standard Atmosphere, 1976*. Washington, DC: U.S. Government Printing Office, 1976 (400 km density 2.80e-12 kg/m³) — VERIFIED (tabulated value)
[24] IAGA Working Group V-MOD, *International Geomagnetic Reference Field, 14th generation (IGRF-14) coefficients*, NOAA NCEI, https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf14coeffs.txt (B0 = 29,733 nT at epoch 2025) — VERIFIED (computed from coefficients)
[25] G. Kopp and J. L. Lean, "A new, lower value of total solar irradiance: Evidence and climate significance," *Geophysical Research Letters*, vol. 38, L01706, 2011. doi:10.1029/2010GL045777 — VERIFIED
[26] O. Montenbruck and E. Gill, *Satellite Orbits: Models, Methods and Applications*. Berlin: Springer, 2000 (Harris–Priester density tables). doi:10.1007/978-3-642-58351-3 — cited via the reference agent's Harris–Priester and NRLMSISE-00 check; book DOI not independently re-verified
