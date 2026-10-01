function tau = srpTorque(q, SC)
% Solar radiation pressure torque (no eclipse, constant area), body axes.
%
% Inputs:
%   q   - double [4x1], attitude quaternion B w.r.t. N
%   SC  - struct
% Outputs:
%   tau - double [3x1], unscaled SRP torque r_cp x F_B in Nm

s_B = qToDCM(q)*SC.sun_N;
F_B = -SC.P_srp*(1 + SC.refl_q)*SC.A_srp*s_B; % SRP force in Newtons
tau = cross(SC.r_cp_srp, F_B);
end
