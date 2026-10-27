function test_visual_sample_index()
% Checks of visualSampleIndex: nearest sample, ends, wrap and clamp (errors on failure).

%% ===== Uniform time grid =====
t = (0:0.1:400).';                                    % [s] default run, 4001 samples
N = numel(t);
[m, tp, ended] = visualSampleIndex(t, 0, false);
assert(m == 1 && tp == 0 && ~ended, 'start of run');
[m, tp, ended] = visualSampleIndex(t, 400, false);
assert(m == N && tp == 400 && ended, 'end of run');
[m, ~, ~] = visualSampleIndex(t, 123.44, false);
assert(m == 1235, 'nearest sample (round down)');
[m, ~, ~] = visualSampleIndex(t, 123.46, false);
assert(m == 1236, 'nearest sample (round up)');

%% ===== Clamp =====
[m, tp, ended] = visualSampleIndex(t, 512, false);
assert(m == N && tp == 400 && ended, 'clamp above the end');
[m, tp, ended] = visualSampleIndex(t, -5, false);
assert(m == 1 && tp == 0 && ~ended, 'clamp below the start');

%% ===== Wrap =====
[m, tp, ended] = visualSampleIndex(t, 410, true);
assert(abs(tp - 10) < 1e-9 && m == 101 && ~ended, 'wrap past the end');
[m, tp, ~] = visualSampleIndex(t, 1210.05, true);
assert(abs(tp - 10.05) < 1e-9 && (m == 101 || m == 102), 'wrap several periods');

%% ===== Monotone in playback time =====
tq = linspace(0, 400, 997);                           % [s] arbitrary playback times
idx = zeros(size(tq));
for k = 1:numel(tq)
    idx(k) = visualSampleIndex(t, tq(k), false);
    assert(abs(t(idx(k)) - tq(k)) <= 0.05 + 1e-12, 'not the nearest sample');
end
assert(all(diff(idx) >= 0), 'index not monotone');

%% ===== Non-uniform grid and a single sample =====
tn = [0; 0.1; 0.5; 2; 2.1];                           % [s]
assert(visualSampleIndex(tn, 1.2, false) == 3, 'non-uniform: nearest is 0.5 s');
assert(visualSampleIndex(tn, 1.3, false) == 4, 'non-uniform: nearest is 2 s');
[m, tp, ended] = visualSampleIndex(5, 7, true);
assert(m == 1 && tp == 5 && ended, 'single sample');
end
