function tau = aeroTorque(q, v_hat_N, SC)
%AEROTORQUE Aerodynamic drag torque (constant area, CP offset), body axes.
%
% Inputs:
%   q       - double [4x1], attitude quaternion B w.r.t. N               [-]
%   v_hat_N - double [3x1], unit orbital velocity direction in N         [-]
%   SC      - struct, uses SC.rho [kg/m^3], SC.Cd [-], SC.A_aero [m^2],
%             SC.n [rad/s], SC.R_orb [m], SC.r_cp_aero [3x1] [m]
% Outputs:
%   tau     - double [3x1], unscaled aerodynamic torque r_cp x F_B       [N*m]

V = SC.n*SC.R_orb;                                   % [m/s] orbital speed
v_B = qToDCM(q)*v_hat_N;
F_B = -0.5*SC.rho*SC.Cd*SC.A_aero*V*V*v_B;          % [N] drag force
tau = cross(SC.r_cp_aero, F_B);
end
