function tau = aeroTorque(q, v_hat_N, SC)
% Aerodynamic drag torque (constant area, CP offset), body axes.
%
% Inputs:
%   q       - double [4x1], attitude quaternion B w.r.t. N
%   v_hat_N - double [3x1], unit orbital velocity direction in N
%   SC      - struct
% Outputs:
%   tau     - double [3x1], unscaled aerodynamic torque r_cp x F_B in N*m

V = SC.n*SC.R_orb; % orbital speed in m/s
v_B = qToDCM(q)*v_hat_N;
F_B = -0.5*SC.rho*SC.Cd*SC.A_aero*V^2*v_B; % drag force in N
tau = cross(SC.r_cp_aero, F_B);
end
