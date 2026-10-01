function [q_r, w_r, wdot_r] = referenceAttitude(t, SC)
% Reference quaternion, rate and rate derivative at time t.
%
% Inputs:
%   t  - double [1], simulation time in secs
%   SC - struct, sim constants; uses SC.mode, SC.q_inertial_ref,
%        SC.ref_fd_h (central-difference step) [s], orbit/approach fields
% Outputs:
%   q_r    - double [4x1], reference quaternion R w.r.t. N (q0 >= 0)
%   w_r    - double [3x1], R-frame rate w.r.t. N in R axes in rad/s
%   wdot_r - double [3x1], time derivative of w_r (R axes) in rad/s^2
%
% Rates are obtained by central differences of C_RN (docs/DESIGN_SPEC.md 3.5):
%   W = -Cdot*C',  w = antisymmetric part of W,  wdot = (w(t+h)-w(t-h))/(2h).

%% ===== Mode 0: inertial hold =====
if SC.mode < 0.5
    q_r = qNormalize(SC.q_inertial_ref);
    w_r = zeros(3,1);
    wdot_r = zeros(3,1);
    return;
end

%% ===== Modes 1 and 2: DCM samples on a 5-point stencil =====
h = SC.ref_fd_h;
Cm2 = referenceDCM(t - 2*h, SC);
Cm1 = referenceDCM(t - h,   SC);
C0  = referenceDCM(t,       SC);
Cp1 = referenceDCM(t + h,   SC);
Cp2 = referenceDCM(t + 2*h, SC);

%% ===== Rates and rate derivative =====
w_r  = localRate(Cm1, C0,  Cp1, h);
w_m  = localRate(Cm2, Cm1, C0,  h);
w_p  = localRate(C0,  Cp1, Cp2, h);
wdot_r = (w_p - w_m)/(2*h);
q_r = dcmToQ(C0);
end

function w = localRate(Cm, C, Cp, h)
% Angular rate from a central difference of a DCM history.
%
% Inputs:
%   Cm - double [3x3], DCM at t-h
%   C  - double [3x3], DCM at t
%   Cp - double [3x3], DCM at t+h
%   h  - double [1], time step in secs
% Outputs:
%   w  - double [3x1], rate such that Cdot = -[w x] C (frame axes)

Cdot = (Cp - Cm)/(2*h);
W = -Cdot*C';
w = 0.5*[W(3,2) - W(2,3); W(1,3) - W(3,1); W(2,1) - W(1,2)];
end
