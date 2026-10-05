function [q_m, w_m, b_dot] = sensorModel(q, w, b, n_att, n_gyro, n_bias, SC)
% Star-tracker and gyro measurement model (ideal unless SC.noise_enable==1).
%
% Inputs:
%   q      - double [4x1], true attitude quaternion B w.r.t. N
%   w      - double [3x1], true body rate, B axes [rad/s]
%   b      - double [3x1], current gyro bias state [rad/s]
%   n_att  - double [3x1], unit-variance normal sample (held for dt)
%   n_gyro - double [3x1], unit-variance normal sample (held for dt)
%   n_bias - double [3x1], unit-variance normal sample (held for dt)
%   SC     - struct, uses SC.noise_enable [-], SC.sig_att [rad],
%            SC.sig_gyro [rad/s], SC.sig_bias_rw [rad/s/sqrt(s)], SC.dt [s]
% Outputs:
%   q_m   - double [4x1], measured attitude quaternion
%   w_m   - double [3x1], measured body rate [rad/s]
%   b_dot - double [3x1], gyro bias rate [rad/s^2]
%
% Bias random walk: n_bias is held constant over one step dt, so
% b_dot = sig_bias_rw/sqrt(dt)*n_bias gives the correct increment std
% sig_bias_rw*sqrt(dt) per step.

if SC.noise_enable > 0.5
    q_m = qNormalize(qMult(qFromRotVec(SC.sig_att*n_att), q));
    w_m = w + b + SC.sig_gyro*n_gyro;
    b_dot = (SC.sig_bias_rw/sqrt(SC.dt))*n_bias;
else
    q_m = q;
    w_m = w;
    b_dot = zeros(3,1);
end
end
