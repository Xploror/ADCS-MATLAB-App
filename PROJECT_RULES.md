# PROJECT_RULES.md — ADCS Controller Comparison Tool

These rules are binding for every agent (human or AI) that develops this project.

## R1. Access scope
- Development agents may only read, write, and delete files inside this folder, `ADCS_ComparisonTool/`. Nothing outside it may be touched.
- Temporary files go in `ADCS_ComparisonTool/_tmp/`, which must be deleted before packaging.
- Development agents do no web research. Citations and literature come from `docs/technical_report.pdf`, `docs/DESIGN_SPEC.md` and, if it exists, `.local/notes.md`. Anything else is requested from the user.

## R2. Source-code commenting standard (mandatory for every .m file)
1. **Function header**: the H1 line directly under `function ...` gives the name in caps and a one-line purpose. It is followed by `Inputs:` and `Outputs:` blocks. Each argument lists its type/size, what it is, and its physical units in brackets (use `[-]` for unitless). Example:
   ```matlab
   function [tau, s] = exampleLaw(q, w, P)
   %EXAMPLELAW One-line purpose of the function.
   %
   % Inputs:
   %   q   - double [4x1], attitude quaternion N->B, scalar-first     [-]
   %   w   - double [3x1], body angular velocity w.r.t. N, body axes   [rad/s]
   %   P   - struct, gain set (fields documented in config/gains)      [mixed]
   % Outputs:
   %   tau - double [3x1], commanded control torque, body axes         [N*m]
   %   s   - double [3x1], sliding variable                            [rad/s]
   ```
2. **Section breaks**: every script and every function longer than about 15 lines is divided with `%% ===== Section name =====`.
3. **Input variables**: every user-settable or config variable carries an inline comment giving its meaning and units, e.g. `R0_m = 500; % [m] initial chaser-to-ISS range`.
4. **Class files** (the app): each method gets the same header, with a one-line purpose, Inputs and Outputs.

## R3. Conventions (see docs/DESIGN_SPEC.md for the full contract)
- Quaternions are scalar-first, `q = [q0; q1; q2; q3]`, and represent the attitude of frame B relative to frame N. `qToDCM(q)` returns C_BN, so `v_B = C_BN * v_N`.
- Units are SI internally (rad, s, m, kg, N*m). Degrees appear only in user-level config fields suffixed `_deg` or `_degps`.
- Every function called from a Simulink MATLAB Function block must be code-generation compatible: no cell arrays, no char or struct creation with dynamic fields, no `evalin`, fixed-size outputs, and only numeric struct fields.
- Code in `src/` and `config/` must run in both MATLAB (R2021a+) and GNU Octave 8, so the reference engine can be tested without MATLAB. Use neither `string` class literals ("...") nor `arguments` blocks.

## R5. CLAUDE.md (mandatory, maintained by development agents)
`CLAUDE.md` at the project root contains the project summary, terminology, inter-dependencies (between modules), intra-dependencies (within modules), invariants, a pointer to the `.local/notes.md`, and the CODE-SCAN section (file cross-check and `notes.md` creation under human approval). Update it whenever the structure or dependencies change, together with the `.local/notes.md` if it exists.

## R6. Undetermined parameter values
If a value cannot be taken from the reviewed literature, first search wider literature (research agents). Failing that, choose a default and validate it with test runs. Record the provenance in the parameter-provenance section of the `.local/notes.md` if it exists, otherwise in the comment next to the parameter and in the change description.

## R7. Environment gaps go to human-in-the-loop
If a required environment (e.g. MATLAB/Simulink, a toolbox, hardware) is unavailable, stop and flag it to the user. Suggest how to provide it, e.g. by connecting a MATLAB MCP server, and wait for the user's response. Continue with a workaround only if the user agrees, and log it as an open caveat.

## R8. HARD RULE: local MCP / MATLAB instance limits (never violate)
- **GUI enabled:** at most **ONE** MATLAB instance at a time.
- **Headless:** at most **THREE** instances at a time.
- **Why:** many simultaneous MATLAB windows push the user's machine into a prolonged suspended state.

In practice:
- Agents issue MATLAB MCP calls strictly one at a time, never in parallel.
- After a timeout, wait before retrying, and retry at most once. After two timeouts, stop and ask the user.
- Pool workers count toward the limit. `runSimulinkBatch` enforces this: no `parsim` while the desktop is running, and headless at most 2 workers.
- Work with **ONE MATLAB session** for the whole validation. Only the orchestrating agent talks to MATLAB; development subagents never call the MATLAB MCP.

## R9. HARD RULE: guardrail hook before every MATLAB MCP call
Run this sequence before **every** new MATLAB call:
1. **Check whether MATLAB is busy** without sending MATLAB a command. Read `_tmp/status.txt`: `BUSY <job> ...` means busy, and `IDLE ...` means free.
2. **While the status is busy, keep waiting** and re-check it periodically. There is no fixed initial wait and no time limit.
3. **Do not proceed further until the status is non-busy (`IDLE`).** No MATLAB call, no new job and no workaround may be started while a job is `BUSY`.
4. A call that times out counts as "busy": never resend it blindly. Wait for the status to become non-busy first.
5. **Session model:** the connector starts its own MATLAB window with the local agent attached. A window the user opens by hand has no agent and can't be used. Never ask the user to open MATLAB manually; the connector's single window is the session.
6. *(Revision note: this replaces the earlier timed hook of "wait 100 s, then 5 min, then exit". The user changed it on 2026-09-23.)*

Every call must return well within the connector's 60 s limit. Queue anything longer on a one-shot timer inside the same session with `_tmp/runJobAsync.m`. It writes `BUSY`/`IDLE` to `_tmp/status.txt` and its output to `_tmp/job_<name>.log`. Progress is monitored from those files only.

## R4. .local/notes.md (local-level, optional)
- `.local/notes.md` belongs to the user's own copy of the project and is git-excluded. It is not part of the shipped project, no build, test or code path depends on it, and it is never committed or packaged unless the user says so.
- If it is missing, it is created only by the CODE-SCAN procedure in `CLAUDE.md` section 7, under explicit human approval and with the `divine-knowledge` skill. Never create it silently, and never from a substitute skill.
- There is at most one notes file, always `.local/notes.md`; a `notes.md` anywhere else is not the knowledge base. It is updated in place, never copied.
- When it exists, it records caveats from every agent (always including the critic's), deviations from the approved plan, future directions, and a numbered References list cited inline as `[n]`.
