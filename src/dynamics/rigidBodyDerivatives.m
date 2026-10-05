function [qdot_raw, wdot] = rigidBodyDerivatives(q_raw, w, tau_rw, tau_d, h_rw, SC)
% Rigid-body attitude kinematics and dynamics with reaction wheels.
%
% Inputs:
%   q_raw  - double [4x1], raw (integrator) quaternion state B w.r.t. N
%   w      - double [3x1], body rate w.r.t. N, B axes [rad/s]
%   tau_rw - double [3x1], wheel reaction torque applied to the body [N*m]
%   tau_d  - double [3x1], total external disturbance torque, B axes [N*m]
%   h_rw   - double [3x1], wheel angular momentum, B axes [N*m*s]
%   SC     - struct, uses SC.J_true [3x3] [kg*m^2], SC.k_norm [s^-1]
% Outputs:
%   qdot_raw - double [4x1], raw quaternion derivative [s^-1]
%   wdot     - double [3x1], body angular acceleration [rad/s^2]
%
% Model (docs/DESIGN_SPEC.md 3.1):
%   qdot_raw = 0.5*B(q)*w + k_norm*(1 - q_raw'q_raw)*q_raw,  q = q_raw/|q_raw|
%   J wdot   = -w x (J w + h_w) + tau_rw + tau_d

q = qNormalize(q_raw);
qdot_raw = 0.5*qKinMatrix(q)*w + SC.k_norm*(1 - q_raw.'*q_raw)*q_raw;
wdot = SC.J_true \ (-cross(w, SC.J_true*w + h_rw) + tau_rw + tau_d);
end
