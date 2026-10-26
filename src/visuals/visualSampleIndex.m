function [m, t_play, ended] = visualSampleIndex(t, t_play, loop)
% Wrap or clamp a playback time and return the nearest stored sample.
%
% Inputs:
%   t      - double [Nx1], sample times of the run, increasing              [s]
%   t_play - double [1], requested playback time (may lie outside [t1, tN])  [s]
%   loop   - logical [1], true = wrap modulo the run length, false = clamp  [-]
% Outputs:
%   m      - double [1], index of the stored sample nearest to t_play       [-]
%   t_play - double [1], playback time after wrapping or clamping           [s]
%   ended  - logical [1], true when playback reached the end (clamp mode)   [-]
%
% The caller advances t_play by sigma*dt_wall each tick (sigma = speed
% factor), so any playback speed maps onto the stored samples without
% decimation.

%% ===== Wrap or clamp =====
t = t(:); % flatten the time series
N = numel(t);
t0 = t(1);
tf = t(N);
ended = false;
if loop && tf > t0
    t_play = t0 + mod(t_play - t0, tf - t0);
else
    if t_play >= tf
        ended = true;
    end
    t_play = max(t_play, t0); % because t_play < tf & t0 < tf
end

%% ===== Nearest sample (binary search) =====
lo = 1;
hi = N;
while hi - lo > 1
    mid = floor((lo + hi)/2);
    if t(mid) <= t_play
        lo = mid;
    else
        hi = mid;
    end
end
if abs(t(hi) - t_play) < abs(t_play - t(lo))
    m = hi;
else
    m = lo;
end
end
