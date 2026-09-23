function tau = srpTorque(q, SC)
%SRPTORQUE Solar radiation pressure torque (no eclipse, constant area), body axes.
%
% Inputs:
%   q   - double [4x1], attitude quaternion B w.r.t. N                   [-]
%   SC  - struct, uses SC.P_srp [N/m^2], SC.refl_q [-], SC.A_srp [m^2],
%         SC.sun_N [3x1] unit vector toward the Sun [-], SC.r_cp_srp [3x1] [m]
% Outputs:
%   tau - double [3x1], unscaled SRP torque r_cp x F_B                   [N*m]

s_B = qToDCM(q)*SC.sun_N;
F_B = -SC.P_srp*(1 + SC.refl_q)*SC.A_srp*s_B;       % [N] SRP force
tau = cross(SC.r_cp_srp, F_B);
end
