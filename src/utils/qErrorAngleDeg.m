function ang_deg = qErrorAngleDeg(q_e)
% Principal rotation angle of an error quaternion, in degrees.
%
% Inputs:
%   q_e     - double [4x1], error quaternion, scalar-first (sign-independent)
% Outputs:
%   ang_deg - double [1], 2*acos(min(1,|q_e0|)), in [0, 180]

ang_deg = 2*acos(min(1, abs(q_e(1))))*180/pi;
end
