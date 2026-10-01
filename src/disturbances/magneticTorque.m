function tau = magneticTorque(q, r_hat_N, SC)
% Residual-dipole torque in an aligned-dipole Earth field, body axes.
%
% Inputs:
%   q       - double [4x1], attitude quaternion B w.r.t. N
%   r_hat_N - double [3x1], unit radial direction in N
%   SC      - struct
% Outputs:
%   tau     - double [3x1], unscaled magnetic torque m_res x B_B in N*m
%
% B_N = B0 (R_E/R_orb)^3 [3 (m.r) r - m], m = -z_E, z_E = [0; sin i; cos i] in N.

m_hat = -[0; sin(SC.incl); cos(SC.incl)];
k = SC.B0*(SC.R_E/SC.R_orb)^3;
B_N = k*(3*(m_hat'*r_hat_N)*r_hat_N - m_hat); % magnetic force in Tesla
B_B = qToDCM(q)*B_N;
tau = cross(SC.m_res, B_B);
end
