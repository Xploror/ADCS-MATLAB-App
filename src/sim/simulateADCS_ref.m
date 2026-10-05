function res = simulateADCS_ref(SC, RG, AG, BG, ctrl_id)
% Reference engine: fixed-step RK4 closed-loop attitude simulation.
%
% Inputs:
%   SC      - struct, sim constants from buildSimParams
%   RG      - struct, robust gains
%   AG      - struct, adaptive gains
%   BG      - struct, baseline gains
%   ctrl_id - double [1], 1 robust SMC, 2 adaptive SMC, 3 PD benchmark
% Outputs:
%   res     - struct, result struct: t, q, w, q_r, w_r, q_e, w_e, att_err_deg, s, tau_cmd,
%             tau_rw, h_w, theta_hat, tau_d, tau_gg, tau_aero, tau_srp,
%             tau_mag, qnorm, sat_flags; and metadata ctrl_id, ctrl_name,
%             engine, SC, RG, AG, BG, wallclock_s
%
% Method: classical RK4 with step SC.dt. The controller, sensors, wheels and 
% disturbances are evaluated at every RK4 stage. Sensor noise is sampled once 
% per step and held. After each step h_w and theta_hat are physically constraint. 
% Logged values at t_k are the block outputs evaluated at the state x_k.
% State x = [q_raw(4); w(3); h_w(3); theta_hat(6); b(3)] (19x1).

wall = tic;

%% ===== Time grid and preallocation =====
dt = SC.dt;
N  = floor(SC.t_final/dt + 0.5) + 1;
t  = (0:N-1)'*dt;
L = struct();
L.q = zeros(N,4);  L.w = zeros(N,3);  L.q_r = zeros(N,4);  L.w_r = zeros(N,3);
L.q_e = zeros(N,4); L.w_e = zeros(N,3); L.att_err_deg = zeros(N,1); L.s = zeros(N,3);
L.tau_cmd = zeros(N,3); L.tau_rw = zeros(N,3); L.h_w = zeros(N,3); L.theta_hat = zeros(N,6);
L.tau_d = zeros(N,3); L.tau_gg = zeros(N,3); L.tau_aero = zeros(N,3); L.tau_srp = zeros(N,3);
L.tau_mag = zeros(N,3); L.qnorm = zeros(N,1); L.sat_flags = zeros(N,6);

%% ===== Initial state setup =====
x = [SC.q0; SC.w0; SC.h_w0; AG.theta_hat0; zeros(3,1)]; % 19x1
if SC.noise_enable > 0.5
    seedRNG(SC.noise_seed);
end
noise = zeros(9,1);

%% ===== Reference at t_0 =====
refA = localRef(0, SC);

%% ===== Integration loop =====
for k = 1:N
    tk = t(k);
    if SC.noise_enable > 0.5
        noise = randn(9,1);                 % [n_att; n_gyro; n_bias], held over the step
    end

    % --- stage 1 (also the logged outputs at t_k) ---
    [k1, o] = localDeriv(tk, x, refA, noise, ctrl_id, SC, RG, AG, BG);
    L.q(k,:) = o.q.';            L.w(k,:) = x(5:7).';
    L.q_r(k,:) = refA.q_r.';     L.w_r(k,:) = refA.w_r.';
    L.q_e(k,:) = o.q_e.';        L.w_e(k,:) = o.w_e.';
    L.att_err_deg(k) = o.att_err_deg;
    L.s(k,:) = o.s.';            L.tau_cmd(k,:) = o.tau_cmd.';
    L.tau_rw(k,:) = o.tau_rw.';  L.h_w(k,:) = x(8:10).';
    L.theta_hat(k,:) = x(11:16).';
    L.tau_d(k,:) = o.tau_d.';    L.tau_gg(k,:) = o.tau_gg.';
    L.tau_aero(k,:) = o.tau_aero.'; L.tau_srp(k,:) = o.tau_srp.';
    L.tau_mag(k,:) = o.tau_mag.';   L.qnorm(k) = o.qnorm;
    L.sat_flags(k,:) = o.sat_flags.';
    if k == N
        break;
    end
    if any(~isfinite(x))
        % fill the remainder with NaN so that metrics flag the divergence
        fn = fieldnames(L);
        for j = 1:numel(fn)
            L.(fn{j})(k+1:N,:) = NaN;
        end
        break;
    end

    % --- stages 2-4 (the reference depends only on t: evaluate twice per step) ---
    refH = localRef(tk + 0.5*dt, SC);
    refB = localRef(tk + dt, SC);
    k2 = localDeriv(tk + 0.5*dt, x + 0.5*dt*k1, refH, noise, ctrl_id, SC, RG, AG, BG);
    k3 = localDeriv(tk + 0.5*dt, x + 0.5*dt*k2, refH, noise, ctrl_id, SC, RG, AG, BG);
    k4 = localDeriv(tk + dt,     x + dt*k3,     refB, noise, ctrl_id, SC, RG, AG, BG);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    % --- integrator limits ---
    x(8:10)  = min(max(x(8:10), -SC.h_max), SC.h_max);
    x(11:16) = min(max(x(11:16), AG.theta_min), AG.theta_max);
    refA = refB;
end

%% ===== Pack result =====
names = controllerNames();
res = struct();
res.t = t;
res.q = L.q;  res.w = L.w;  res.q_r = L.q_r;  res.w_r = L.w_r;
res.q_e = L.q_e;  res.w_e = L.w_e;  res.att_err_deg = L.att_err_deg;  res.s = L.s;
res.tau_cmd = L.tau_cmd;  res.tau_rw = L.tau_rw;  res.h_w = L.h_w;  res.theta_hat = L.theta_hat;
res.tau_d = L.tau_d;  res.tau_gg = L.tau_gg;  res.tau_aero = L.tau_aero;
res.tau_srp = L.tau_srp;  res.tau_mag = L.tau_mag;  res.qnorm = L.qnorm;
res.sat_flags = L.sat_flags;
res.ctrl_id = ctrl_id;
res.ctrl_name = names{min(max(round(ctrl_id), 1), 3)};
res.engine = 'reference';
res.SC = SC;  res.RG = RG;  res.AG = AG;  res.BG = BG;
res.wallclock_s = toc(wall);
end

%% ===== Local functions =====
function r = localRef(t, SC)
% Reference attitude packed in a struct.
%
% Inputs:
%   t  - double [1], time                                                 [s]
%   SC - struct, sim constants                                            [mixed SI]
% Outputs:
%   r  - struct, q_r [4x1] , w_r [3x1] [rad/s], wdot_r [3x1] [rad/s^2]

[q_r, w_r, wdot_r] = referenceAttitude(t, SC);
r = struct('q_r', q_r, 'w_r', w_r, 'wdot_r', wdot_r);
end

function [xdot, o] = localDeriv(t, x, ref, noise, ctrl_id, SC, RG, AG, BG)
% Closed-loop state derivative (same block chain as the Simulink harness).
%
% Inputs:
%   t       - double [1], time                                            [s]
%   x       - double [19x1], state [q_raw; w; h_w; theta_hat; b]          [mixed SI]
%   ref     - struct, reference q_r, w_r, wdot_r at t                     [mixed SI]
%   noise   - double [9x1], held unit normal samples [n_att; n_gyro; n_bias] 
%   ctrl_id - double [1], controller id                                   
%   SC, RG, AG, BG - structs, sim constants and gains                     [mixed SI]
% Outputs:
%   xdot    - double [19x1], state derivative                             [mixed SI]
%   o       - struct, block outputs for logging                           [mixed SI]

%% ===== Unpack the state =====
q_raw = x(1:4);  w = x(5:7);  h_w = x(8:10);  theta_hat = x(11:16);  b = x(17:19);

%% ===== Block chain: sensors, controller, wheels, disturbances, dynamics =====
[q, qnorm] = quatNormalizeState(q_raw);
[q_m, w_m, b_dot] = sensorModel(q, w, b, noise(1:3), noise(4:6), noise(7:9), SC);
[tau_cmd, th_dot, s] = controllerSelect(ctrl_id, q_m, w_m, ref.q_r, ref.w_r, ref.wdot_r, h_w, theta_hat, SC, RG, AG, BG);
[h_w_dot, tau_rw, sat_flags] = reactionWheelModel(tau_cmd, h_w, SC);
[tau_d, tau_gg, tau_aero, tau_srp, tau_mag] = disturbanceTorques(t, q, SC);
[qdot_raw, wdot] = rigidBodyDerivatives(q_raw, w, tau_rw, tau_d, h_w, SC);
xdot = [qdot_raw; wdot; h_w_dot; th_dot; b_dot];

%% ===== Logged block outputs (only when requested) =====
if nargout > 1
    [q_e, w_e, att_err_deg] = trueErrors(q, w, ref.q_r, ref.w_r, SC);
    o = struct('q', q, 'qnorm', qnorm, 'q_e', q_e, 'w_e', w_e, 'att_err_deg', att_err_deg, ...
               's', s, 'tau_cmd', tau_cmd, 'tau_rw', tau_rw, 'sat_flags', sat_flags, ...
               'tau_d', tau_d, 'tau_gg', tau_gg, 'tau_aero', tau_aero, ...
               'tau_srp', tau_srp, 'tau_mag', tau_mag);
end
end
