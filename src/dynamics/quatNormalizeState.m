function [q, qnorm] = quatNormalizeState(q_raw)
%QUATNORMALIZESTATE Normalised attitude output and norm of the raw integrator state.
%
% Inputs:
%   q_raw - double [4x1], raw integrator quaternion state                [-]
% Outputs:
%   q     - double [4x1], unit quaternion q_raw/|q_raw|                   [-]
%   qnorm - double [1], |q_raw| (health indicator, ideally 1)             [-]

qnorm = sqrt(q_raw.'*q_raw);
q = qNormalize(q_raw);
end
