# DESIGN_SPEC.md — Interface Contract (v1.0.0, rev. 2 after first verification)

This is the single source of truth for function names, signatures, struct fields, conventions and signal names. Every file must conform to it. If an implementation needs a change, the change is made here first and recorded in the change description and, if it exists, in the `.local/notes.md` under "Deviations".

Target platforms: MATLAB R2021a+ with Simulink (primary). The `src/` and `config/` layers also run on GNU Octave 8 through the reference engine.

---------------------------------------------------------------------
## 1. Conventions

### 1.1 Frames
- **N**: orbit-plane inertial frame. x̂_N points to the ascending node, ẑ_N is the orbit normal, and ŷ_N = ẑ_N × x̂_N. The circular orbit lies in the x–y plane with argument of latitude θ(t) = n·t (θ(0) = 0).
- **O**: LVLH frame, rows of C_ON:
  - ô1 = along-track (velocity) v̂_N = [−sinθ; cosθ; 0]
  - ô2 = −orbit normal = [0; 0; −1]
  - ô3 = nadir = −r̂_N, with r̂_N = [cosθ; sinθ; 0]
  - C_ON = [ô1ᵀ; ô2ᵀ; ô3ᵀ]
- **R**: reference (desired body) frame.
- **B**: spacecraft body frame. Three orthogonal reaction wheels are aligned with the B axes.

### 1.2 Quaternions (scalar-first, "attitude of B w.r.t. N")
- `q = [q0; q1; q2; q3]`, `qv = q(2:4)`
- `C_BN = qToDCM(q) = (q0² − qvᵀqv) I + 2 qv qvᵀ − 2 q0 [qv×]`, so that `v_B = C_BN v_N`
- Composition: `qMult(qa, qb) = [a0 b0 − av·bv ; a0 bv + b0 av − av × bv]` satisfies `qToDCM(qMult(qa,qb)) = qToDCM(qa) * qToDCM(qb)`. This must be verified by a unit test.
- Conjugate: `qConj(q) = [q0; −qv]` satisfies `qToDCM(qConj(q)) = qToDCM(q)ᵀ`.
- Kinematics: `q̇ = ½ B(q) ω`, where `B(q) = [ −qvᵀ ; q0 I3 + [qv×] ]` (4×3) and ω is the body rate w.r.t. N in B axes.
- Axis-angle: `qFromAxisAngle(e, φ) = [cos(φ/2); ê sin(φ/2)]`. It is the attitude of a frame rotated by +φ about ê: `C = cosφ I + (1−cosφ) ê êᵀ − sinφ [ê×]`.
- `skew3(v) = [v×] = [0 −v3 v2; v3 0 −v1; −v2 v1 0]`

### 1.3 Inertia parameter vector
- `θ = [Jxx; Jyy; Jzz; Jxy; Jxz; Jyz]`, where J = [Jxx Jxy Jxz; Jxy Jyy Jyz; Jxz Jyz Jzz]. Jxy etc. are the matrix entries, with their sign included.
- `inertiaLinearMap(v)` returns L(v) (3×6) such that `J v = L(v) θ`:
  `L(v) = [v1 0 0 v2 v3 0; 0 v2 0 v1 0 v3; 0 0 v3 0 v1 v2]`

---------------------------------------------------------------------
## 2. Control laws (shared error definitions)

Inputs are the measured attitude q and rate ω, plus the reference (q_r, ω_r, ω̇_r). Here ω_r is the R-frame rate w.r.t. N, in R axes.

```
q_e      = qMult(q, qConj(q_r))                 % C(q_e) = C_BR
if shortest_path && q_e(1) < 0, q_e = -q_e; end
C_e      = qToDCM(q_e)
w_dB     = C_e * w_r                             % reference rate in B axes
w_e      = w - w_dB
qev_dot  = 0.5*(q_e(1)*eye(3) + skew3(q_e(2:4))) * w_e
w_rv     = w_dB - Lambda*q_ev                    % virtual reference rate
w_rv_dot = -skew3(w_e)*w_dB + C_e*wdot_r - Lambda*qev_dot
s        = w_e + Lambda*q_ev   (= w - w_rv)
Y        = inertiaLinearMap(w_rv_dot) + skew3(w_rv)*inertiaLinearMap(w)     % 3x6
```

Identity: `Y θ = J ω̇_rv + ω_rv × (J ω)`. With plant `J ω̇ = −ω×(Jω + h_w) + τ_rw + τ_d` and the common compensation `+ω×h_w`, this gives `J ṡ = (Jω)×s + Y(θ̂−θ) − K s − η⊙sat(s/φ) + τ_d`. Since `sᵀ((Jω)×s) = 0`, V = ½sᵀJs + ½θ̃ᵀΓ⁻¹θ̃ is a valid Lyapunov function.

Here `Lambda = diag(lambda_diag)`, `K = diag(K_diag)`, and `sat(x) = min(max(x,−1),1)` is applied elementwise.

| # | Controller | Law | Notes |
|---|---|---|---|
| 1 | Robust SMC (non-adaptive) | `τ_c = Y θ_nom − K s − η ⊙ sat(s ./ φ)` | `θ_nom = inertiaToTheta(SC.J_nom)` is fixed |
| 2 | Adaptive SMC (parametric) | `τ_c = Y θ̂ − K s − η ⊙ sat(s ./ φ)` ; `θ̂̇ = paramProjection(θ̂, −Γ Yᵀ s, θ_min, θ_max)` | Γ = diag(Gamma_diag). K, η, φ and Λ default to the same values as controller 1 |
| 3 | PD benchmark (Wie & Barba) | `τ_c = −Kp ⊙ q_ev − Kd ⊙ ω_e` | Kp, Kd are diagonal vectors |

Common post-processing in `controllerSelect`:
`τ_cmd = τ_c + SC.gyro_comp * cross(ω, h_w)`. **Adaptation anti-windup (v1.0.0):** for controller 2 with `AG.freeze_on_sat = 1`, θ̂̇ = 0 whenever any |τ_cmd,i| > SC.tau_max or any |h_w,i| ≥ SC.h_max. After that, each axis is clamped to ±SC.tau_max. θ̂̇ is zeros(6,1) unless the controller is 2. s is zeros(3,1) for controller 3.

`paramProjection`: elementwise, the output is 0 where (θ̂_i ≥ θmax_i and raw_i > 0) or (θ̂_i ≤ θmin_i and raw_i < 0). Otherwise it equals raw_i.

**Controllers may read only**: `SC.J_nom`, `SC.theta_nom`, `SC.tau_max`, `SC.h_max`, `SC.gyro_comp`, `SC.shortest_path`, and their own gain struct. **Never `SC.J_true`.**

---------------------------------------------------------------------
## 3. Plant, actuator, sensors, disturbances, reference

### 3.1 Rigid body (true inertia)
```
q     = q_raw / norm(q_raw)
q̇_raw = 0.5*qKinMatrix(q)*w + SC.k_norm*(1 - q_raw'*q_raw)*q_raw     % constraint stabilisation
ẇ     = SC.J_true \ ( -cross(w, SC.J_true*w + h_w) + tau_rw + tau_d )
```

### 3.2 Reaction wheels (3 orthogonal, body-aligned)
```
tau_s = min(max(tau_cmd, -tau_max), tau_max)
h_w_dot = -tau_s
for i: if (h_w(i) >= h_max && h_w_dot(i) > 0) || (h_w(i) <= -h_max && h_w_dot(i) < 0), h_w_dot(i) = 0
tau_rw = -h_w_dot                                   % torque applied to the body
sat_flags = [abs(tau_cmd) >= tau_max*(1-1e-9) ; momentum-cutoff active]  (6x1 double 0/1; '>=' because tau_cmd arrives already clamped)
```

### 3.3 Sensors (off by default)
When `SC.noise_enable == 1`:
- `q_m = qNormalize(qMult(qFromRotVec(SC.sig_att*n_att), q))`
- `w_m = w + b + SC.sig_gyro*n_gyro`
- `b_dot = SC.sig_bias_rw/sqrt(SC.dt)*n_bias` (1/√dt scaling because the noise is held for one step)

Otherwise `q_m = q`, `w_m = w` and `b_dot = 0`. Here `n_*` are unit-variance normal 3-vectors, held for one step dt.

### 3.4 Disturbances (all in B axes, then multiplied by `SC.dist_enable(k)*SC.dist_scale(k)`, k = GG, Aero, SRP, Mag)
- **GG**: `τ = 3 n² r̂_B × (J_true r̂_B)`, with `r̂_B = C_BN r̂_N`.
- **Aero**: `F_B = −½ ρ Cd A V² v̂_B`, with `V = n R_orb` and `v̂_B = C_BN v̂_N`; `τ = r_cp_aero × F_B`. Constant area.
- **SRP**: `F_B = −P_srp (1+refl_q) A_srp ŝ_B`, with `ŝ_B = C_BN sun_N` (unit vector toward the Sun); `τ = r_cp_srp × F_B`. No eclipse and constant area.
- **Mag**: `B_N = B0 (R_E/R_orb)³ [3(m̂·r̂)r̂ − m̂]`, with `m̂ = −ẑ_E` and `ẑ_E = [0; sin i; cos i]` in N; `B_B = C_BN B_N`; `τ = m_res × B_B`. Tilted dipole and Earth rotation are ignored.

### 3.5 Reference attitude `referenceAttitude(t, SC)`
- **mode 0 (inertial hold)**: `q_r = SC.q_inertial_ref`, `ω_r = ω̇_r = 0`.
- **mode 1 (LVLH hold)** and **mode 2 (docking approach)**: `C_RN(t) = C_RO(t) * C_ON(t)`, where
  - mode 1: `û = [1;0;0]` (docking axis along +V-bar)
  - mode 2: the approach profile gives `R(t) = Rf + (R0 − Rf) exp(−t/T_close)`, `y(t) = y0 (R/R0)²`, `z(t) = z0 (R/R0)²`. The chaser position in O is `[−R; y; z]` relative to the ISS, and `û = [R; −y; −z]/‖·‖` (line of sight to the ISS).
  - `b1 = û`, `b2 = normalize(cross([0;0;1], b1))`, `b3 = cross(b1, b2)`, `C_RO = [b1ᵀ; b2ᵀ; b3ᵀ]`
  - `ω_r`: with `Ċ ≈ (C(t+h) − C(t−h))/(2h)` and `W = −Ċ C(t)ᵀ`, take `ω_r = [W(3,2) − W(2,3); W(1,3) − W(3,1); W(2,1) − W(1,2)]/2` (antisymmetric part).
  - `ω̇_r ≈ (ω_r(t+h) − ω_r(t−h))/(2h)`, with `h = SC.ref_fd_h`.
  - `q_r = dcmToQ(C_RN)` (Shepperd's method, q0 ≥ 0).
- **mode 3 (attitude scan, added in rev. 2)**: `C_RN = qToDCM(qFromRotVec(φ(t))) · C_RN,mode1`, with `φ_i = SC.scan_amp_i · sin(SC.scan_freq_i · t)`. Incommensurate frequencies give a persistently exciting reference for the inertia estimator. ω_r and ω̇_r use the same finite-difference scheme.

---------------------------------------------------------------------
## 4. Configuration structs

### 4.1 User-level config `cfg` (human units, may contain char fields)

`cfg = initDefaults()` returns:
```
cfg.format_version = 1
cfg.name  (char)          cfg.description (char)
cfg.scenario.att_err_axis        [3x1] [-]        default [1;1;1]/sqrt(3)
cfg.scenario.att_err_angle_deg   [1]   [deg]      default 5
cfg.scenario.rate_err0_degps     [3x1] [deg/s]    default [0;0;0]  (body rate error w.r.t. reference at t=0)
cfg.scenario.target_mode         [1]   0|1|2|3    default 2 (docking approach); 3 = attitude scan about LVLH hold
cfg.scenario.scan_amp_deg        [3x1] [deg]      default [3;3;3]          (mode 3)
cfg.scenario.scan_freq_radps     [3x1] [rad/s]    default [0.015;0.020;0.025] (mode 3)
cfg.scenario.q_inertial_ref      [4x1] [-]        default [1;0;0;0]
cfg.scenario.orbit_alt_km        [1]   [km]       default 400
cfg.scenario.orbit_incl_deg      [1]   [deg]      default 51.64
cfg.scenario.R0_m 500, Rf_m 10, T_close_s 600, y_off0_m 0, z_off0_m 100
cfg.scenario.J_nom               [3x3] [kg m^2]   default [1200 100 -200; 100 2200 300; -200 300 3100]  (Wie, Weiss & Arapostathis 1989)
cfg.scenario.J_unc_pct           [1]   [%]        default 0
cfg.scenario.J_unc_mode          [1]   1=random signed per-parameter (seeded, physically valid), 2=uniform scale (1+pct/100)
cfg.scenario.J_unc_seed          [1]   default 42
cfg.scenario.dist_enable         [1x4] GG Aero SRP Mag, default [1 1 0 0]
cfg.scenario.dist_scale          [1x4] default [1 1 1 1]
cfg.scenario.rho_kgm3 3e-12, Cd 2.2, A_aero_m2 10, r_cp_aero_m [0.10;0.05;0.0]
cfg.scenario.P_srp_Nm2 4.54e-6, refl_q 0.6, A_srp_m2 10, r_cp_srp_m [0.05;0.10;0.0], sun_dir_N [1;0;0]
cfg.scenario.B0_T 2.97e-5 (IGRF-14, 2025), m_res_Am2 [0.5;0.5;0.5]
cfg.scenario.wheel_tau_max_Nm 0.15, wheel_h_max_Nms 15, h_w0_Nms [0;0;0]
cfg.scenario.noise_enable 0, att_noise_std_deg 0.005, gyro_noise_std_degps 0.001, gyro_bias_rw_degps_rts 1e-5, noise_seed 7
cfg.scenario.t_final_s 400, dt_s 0.1
cfg.scenario.shortest_path 1, gyro_comp 1, k_norm 1.0, ref_fd_h_s 0.5
cfg.gains.robust   = robust_default(J_nom)
cfg.gains.adaptive = adaptive_default(J_nom)
cfg.gains.baseline = baseline_default(J_nom)
cfg.metrics.settle_thresh_deg 0.5, ss_window_frac 0.1
cfg.pass.ss_err_max_deg 0.5, settle_time_max_s Inf, qnorm_tol 1e-3
```

User gain structs (SI):
```
robust:   lambda_diag [3x1] [1/s], K_diag [3x1] [N m s], eta [3x1] [N m], phi [3x1] [rad/s]
adaptive: lambda_diag, K_diag, eta, phi (same meaning and defaults as robust),
          Gamma_diag [6x1] [kg m^2 s^2] (so that -Gamma*Y'*s has units kg m^2/s), theta_hat0_from_Jnom [1] (1 = start at J_nom),
          theta_hat0 [6x1] [kg m^2] (used only if theta_hat0_from_Jnom==0), proj_frac [1] [-] (bounds width, default 0.6),
          freeze_on_sat [1] [-] (1 = adaptation anti-windup, default 1)
baseline: Kp_diag [3x1] [N m], Kd_diag [3x1] [N m s]     (defaults from pdGainsFromBandwidth(J_nom, 0.03, 0.9))
```

### 4.2 Sim-level structs from `[SC, RG, AG, BG, info] = buildSimParams(cfg)` (numeric only, SI)

`SC` fields: `dt, t_final, mode, q_inertial_ref(4x1), mu (3.986004418e14), R_E (6378137), R_orb, n, incl, R0, Rf, T_close, y0, z0, ref_fd_h, scan_amp(3x1) [rad], scan_freq(3x1) [rad/s], J_nom(3x3), J_true(3x3), theta_nom(6x1), theta_true(6x1), q0(4x1), w0(3x1), h_w0(3x1), tau_max, h_max, dist_enable(1x4), dist_scale(1x4), rho, Cd, A_aero, r_cp_aero(3x1), P_srp, refl_q, A_srp, r_cp_srp(3x1), sun_N(3x1 unit), B0, m_res(3x1), noise_enable, sig_att [rad], sig_gyro [rad/s], sig_bias_rw [rad/s/sqrt(s)], noise_seed, shortest_path, gyro_comp, k_norm`

- `RG`: `lambda_diag, K_diag, eta, phi` (3x1 each)
- `AG`: `lambda_diag, K_diag, eta, phi, Gamma_diag(6x1), theta_hat0(6x1), theta_min(6x1), theta_max(6x1), freeze_on_sat`
- `BG`: `Kp_diag, Kd_diag`
- `info` (may contain char): `J_true_physical (logical)`, `q_r0`, and notes.

Derivations in `buildSimParams`:
- `n = sqrt(mu/R_orb^3)`, with `R_orb = R_E + 1e3*alt`.
- `q0 = qMult(qFromAxisAngle(axis, angle), q_r0)` and `w0 = qToDCM(dq)*w_r0 + deg2rad(rate_err0)`, where `[q_r0, w_r0] = referenceAttitude(0, SC)`.
- J_true, mode 1: a seeded uniform perturbation `θ_i(1+u_i p/100)` with `u_i ∈ [−1,1]` is redrawn until J is positive definite and its principal moments satisfy the triangle inequalities (at most 1000 tries). Mode 2: `(1+p/100) J_nom`.
- Projection bounds: diagonal `[ (1−f) θ_i, (1+f) θ_i ]`, products `θ_i ± (|θ_i| + f·mean(diag(J_nom)))`, with `f = proj_frac`.

Simulink workspace or data-dictionary variable names: **`SC`, `RobustGains` (=RG), `AdaptiveGains` (=AG), `BaselineGains` (=BG), `CONTROLLER_SELECT` (1 | 2 | 3)**.

---------------------------------------------------------------------
## 5. Function inventory (exact signatures)

`src/utils/`: `C = qToDCM(q)`; `q = dcmToQ(C)`; `q = qMult(qa, qb)`; `qc = qConj(q)`; `q = qFromAxisAngle(axis, angle_rad)`; `q = qFromRotVec(phi_rad)`; `qn = qNormalize(q)`; `S = skew3(v)`; `B = qKinMatrix(q)`; `ang_deg = qErrorAngleDeg(q_e)` (=2*acos(min(1,|q_e0|)) in deg); `y = satVec(x)`; `seedRNG(seed)` (MATLAB `rng`/Octave `randn('state')`, not codegen); `tf = isOctave()`

`src/reference/`: `[r_hat_N, v_hat_N, C_ON] = orbitGeometry(t, SC)`; `[R, y, z] = approachProfile(t, SC)`; `C_RN = referenceDCM(t, SC)`; `[q_r, w_r, wdot_r] = referenceAttitude(t, SC)`

`src/dynamics/`: `[qdot_raw, wdot] = rigidBodyDerivatives(q_raw, w, tau_rw, tau_d, h_w, SC)`; `[q, qnorm] = quatNormalizeState(q_raw)`; `[h_w_dot, tau_rw, sat_flags] = reactionWheelModel(tau_cmd, h_w, SC)`; `[q_m, w_m, b_dot] = sensorModel(q, w, b, n_att, n_gyro, n_bias, SC)`

`src/disturbances/`: `tau = gravityGradientTorque(q, r_hat_N, SC)`; `tau = aeroTorque(q, v_hat_N, SC)`; `tau = srpTorque(q, SC)`; `tau = magneticTorque(q, r_hat_N, SC)`; `[tau_d, tau_gg, tau_aero, tau_srp, tau_mag] = disturbanceTorques(t, q, SC)`

`src/controllers/`: `[q_e, w_e, w_dB, qev_dot, w_rv, w_rv_dot, s] = attitudeErrors(q, w, q_r, w_r, wdot_r, lambda_diag, shortest_path)`; `L = inertiaLinearMap(v)`; `Y = inertiaRegressor(w, w_rv, w_rv_dot)`; `theta = inertiaToTheta(J)`; `J = thetaToInertia(theta)`; `[tau_c, s] = robustControlLaw(q, w, q_r, w_r, wdot_r, SC, RG)`; `[tau_c, theta_hat_dot, s] = adaptiveControlLaw(q, w, q_r, w_r, wdot_r, theta_hat, SC, AG)`; `tau_c = baselineControlLaw(q, w, q_r, w_r, wdot_r, SC, BG)`; `thdot = paramProjection(theta_hat, thdot_raw, th_min, th_max)`; `[tau_cmd, theta_hat_dot, s] = controllerSelect(ctrl_id, q_m, w_m, q_r, w_r, wdot_r, h_w, theta_hat, SC, RG, AG, BG)`; `[q_e, w_e, att_err_deg] = trueErrors(q, w, q_r, w_r, SC)` (true-state errors for logging and metrics); `[Kp_diag, Kd_diag] = pdGainsFromBandwidth(J_nom, wn, zeta)` (`Kp = 2 diag(J) wn²`, `Kd = 2 ζ wn diag(J)`)

`src/sim/`: `[SC, RG, AG, BG, info] = buildSimParams(cfg)`; `res = simulateADCS_ref(SC, RG, AG, BG, ctrl_id)` (fixed-step RK4 with the same dt; noise sampled once per step; h_w and θ̂ clamped after each step); `results = runSimulation(cfg, ctrl_ids, engine)` (engine is `'auto'|'simulink'|'reference'`; returns a 1xK struct array; `'auto'` means Simulink if `isSimulinkAvailable()` else reference); `results = runSimulinkBatch(SC, RG, AG, BG, ctrl_ids)` (builds a `Simulink.SimulationInput` array, one element per controller, calls `setVariable` for `SC`, `RobustGains`, `AdaptiveGains`, `BaselineGains` and `CONTROLLER_SELECT`, sets StopTime/FixedStep numerically, and runs `parsim` if Parallel Computing Toolbox is licensed, otherwise `sim` on the array. Returns a 1xK `res` array via `extractSimulinkResults`); `res = extractSimulinkResults(simOut, SC, RG, AG, BG, ctrl_id)`; `tf = isSimulinkAvailable()`; `names = controllerNames()` → `{'Robust SMC','Adaptive SMC','PD Benchmark'}`; `ensureModelBuilt()` (builds the .slx via `build_ADCS_model` if missing)

`src/analysis/`: `M = computeMetrics(res, cfg)`; `[pass, reasons] = evaluatePassFail(M, passCfg)`; `[header, rows] = metricsTable(results, metrics, passFlags)`; `exportMetricsCSV(file, header, rows)`; `plotResultsOnAxes(ax, results, quantity, cfg)`, where `ax` is an axes/uiaxes handle and `quantity` is one of `'att_err','w_e','tau_rw','h_w','qnorm','theta_hat'`, which overlays all results in `results`.

`src/config_io/`: `saveConfig(file, cfg)`; `cfg = loadConfig(file)` (validates `format_version`, fills missing fields from `initDefaults`)

`config/`: `cfg = initDefaults()`; `g = robust_default(J_nom)`, `g = adaptive_default(J_nom)`, `g = baseline_default(J_nom)` (in `config/gains/`); `meta = gainMetadata()`, where `meta.robust`, `meta.adaptive` and `meta.baseline` are cell arrays Nx5 `{field, label, units, n_elements, description}`; `list = listScenarios()`, a Kx2 cell `{func_name, display_name}`; scenario functions in `config/scenarios/`: `scn_nominal`, `scn_inertia_uncertainty_pm25`, `scn_large_attitude_error`, `scn_disturbance_stress`, `scn_actuator_saturation_stress`, `scn_sensor_noise`, `scn_adaptive_excitation_scan` (rev. 2), each `cfg = scn_x()` built from `initDefaults()`.

`models/`: `mdl = build_ADCS_model(overwrite)` creates `models/ADCS_ComparisonHarness.slx` and `models/ADCS_Params.sldd`; `pushParamsToDictionary(cfg, ctrl_id)`

`app/`: `ADCS_MATLAB_App.m` (`classdef ... < matlab.apps.AppBase`)

Root: `startup_ADCS.m` (adds paths). `examples/run_example_comparison.m`

### 5.1 Metrics struct `M` (from `computeMetrics`)
`settle_time_s` (first t after which att_err ≤ thresh for all later t; NaN if never), `ss_err_deg` (mean att_err over the last `ss_window_frac` of the run), `final_err_deg`, `peak_err_deg`, `peak_torque_Nm` (max |tau_rw| over all axes), `effort_L1` (∫‖tau_rw‖dt [N m s]), `effort_L2` (∫‖tau_rw‖²dt [N² m² s]), `peak_h_Nms`, `sat_time_torque_s`, `sat_time_mom_s`, `chatter_TV` (Σ‖Δtau_cmd‖₁ / t_final [N m/s]), `qnorm_max_dev`, `theta_err_final_pct` (100·‖θ̂_end − θ_true‖/‖θ_true‖; NaN unless adaptive), `all_finite` (logical).

Pass criteria (`evaluatePassFail`) are: all_finite; ss_err_deg ≤ ss_err_max_deg; qnorm_max_dev ≤ qnorm_tol; settle_time_s ≤ settle_time_max_s (NaN counts as fail only if settle_time_max_s is finite).

---------------------------------------------------------------------
## 6. Result struct `res` (identical fields for both engines)

N samples, row-major time series:
`t [Nx1] s`, `q [Nx4]` (true, normalised), `w [Nx3]`, `q_r [Nx4]`, `w_r [Nx3]`, `q_e [Nx4]` (true), `w_e [Nx3]` (true), `att_err_deg [Nx1]`, `s [Nx3]`, `tau_cmd [Nx3]`, `tau_rw [Nx3]`, `h_w [Nx3]`, `theta_hat [Nx6]`, `tau_d, tau_gg, tau_aero, tau_srp, tau_mag [Nx3]`, `qnorm [Nx1]` (norm of the raw integrator state), `sat_flags [Nx6]`

Metadata: `ctrl_id`, `ctrl_name` (char), `engine` (`'simulink'|'reference'`), `SC`, `RG`, `AG`, `BG`, `wallclock_s`.

---------------------------------------------------------------------
## 7. Simulink model `ADCS_ComparisonHarness` (built by `build_ADCS_model.m`)

- Solver: fixed-step `ode4`, FixedStep = `SC.dt` (the App also sets it numerically), StopTime = `SC.t_final`. Signal logging is on with `logsout` as a Dataset. Single simulation output (`ReturnWorkspaceOutputs = on`).
- MATLAB Function blocks each call the library function of the same role. Parameters (`SC`, `RobustGains`, `AdaptiveGains`, `BaselineGains`, `CONTROLLER_SELECT`) are Stateflow data with Scope = `Parameter`.

| Subsystem (created with `Simulink.BlockDiagram.createSubsystem` after wiring) | Contents |
|---|---|
| `RefGenerator` | Clock → MATLAB Fcn `[q_r,w_r,wdot_r] = referenceAttitude(t,SC)` |
| `Sensors` | 3× Random Number blocks (Mean 0, Variance 1, SampleTime `SC.dt`, Seed `SC.noise_seed+[1 2 3]`, `+[4 5 6]`, `+[7 8 9]`) + MATLAB Fcn `sensorModel` + bias Integrator (IC `[0;0;0]`) |
| `ControllerSelect` | MATLAB Fcn `controllerSelect(CONTROLLER_SELECT, …)` + θ̂ Integrator (IC `AdaptiveGains.theta_hat0`, limits `AdaptiveGains.theta_min/max`) |
| `ReactionWheelActuator` | MATLAB Fcn `reactionWheelModel` + h_w Integrator (IC `SC.h_w0`, limits ±`SC.h_max`) |
| `DisturbanceTorques` | MATLAB Fcn `disturbanceTorques(t,q,SC)` |
| `RigidBodyDynamics3DOF` | MATLAB Fcn `rigidBodyDerivatives` + q_raw Integrator (IC `SC.q0`) + ω Integrator (IC `SC.w0`) + **separate** MATLAB Fcn `quatNormalizeState` |
| `TrueErrors` | MATLAB Fcn `trueErrors(q,w,q_r,w_r,SC)` |

**Algebraic-loop rule:** normalisation of q must be its own MATLAB Function block. If it shared a block with `rigidBodyDerivatives`, which has direct feedthrough from `tau_rw`, it would create a loop.

**Logged signal names** (line names, DataLogging on) must equal the `res` field names: `q, w, q_r, w_r, q_e, w_e, att_err_deg, s, tau_cmd, tau_rw, h_w, theta_hat, tau_d, tau_gg, tau_aero, tau_srp, tau_mag, qnorm, sat_flags`.

`extractSimulinkResults` squeezes 3-D logged data ([n×1×N] or [N×n]) into [N×n] row-major.
