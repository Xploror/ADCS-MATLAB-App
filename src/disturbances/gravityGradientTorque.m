function tau = gravityGradientTorque(q, r_hat_N, SC)
% Gravity-gradient torque on the true inertia, body axes.
%
% Inputs:
%   q       - double [4x1], attitude quaternion B w.r.t. N
%   r_hat_N - double [3x1], unit radial (zenith) direction in N
%   SC      - struct
% Outputs:
%   tau     - double [3x1], unscaled GG torque 3 n^2 r_B x (J r_B) in N*m

r_B = qToDCM(q)*r_hat_N;
tau = 3*SC.n^2*cross(r_B, SC.J_true*r_B);
end
