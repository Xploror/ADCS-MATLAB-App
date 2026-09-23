function summary = run_all_tests()
%RUN_ALL_TESTS Run every tests/test_*.m function, print a PASS/FAIL/SKIP table and return a summary.
%
% Inputs:
%   (none)
% Outputs:
%   summary - struct with fields
%             names   - cell [1xK] of char, test function names          [-]
%             status  - cell [1xK] of char, 'PASS' | 'FAIL' | 'SKIP'     [-]
%             message - cell [1xK] of char, error message on failure     [-]
%             time_s  - double [1xK], wall-clock time per test           [s]
%             n_pass, n_fail, n_skip - double [1], counts                [-]
%
% Each test is a function file that throws an error on failure. The
% Simulink cross-check (test_simulink_vs_reference) is reported as SKIP
% when Simulink is not available (always the case in GNU Octave).
% The scenario smoke test is NOT run here (it takes minutes); run
% run_all_scenarios_smoketest separately.

%% ===== Discover tests =====
test_dir = fileparts(mfilename('fullpath'));          % [-] folder of this file
files = dir(fullfile(test_dir, 'test_*.m'));          % [-] test function files
names = sort({files.name});                           % [-] alphabetical order
K = numel(names);                                     % [-] number of tests

summary = struct();
summary.names   = cell(1, K);
summary.status  = cell(1, K);
summary.message = cell(1, K);
summary.time_s  = zeros(1, K);

%% ===== Run tests =====
for k = 1:K
    [~, fname] = fileparts(names{k});
    summary.names{k} = fname;
    summary.message{k} = '';
    if strcmp(fname, 'test_simulink_vs_reference') && ~isSimulinkAvailable()
        summary.status{k} = 'SKIP';
        summary.message{k} = 'Simulink not available';
        continue;
    end
    t0 = tic;
    try
        feval(fname);
        summary.status{k} = 'PASS';
    catch err
        summary.status{k} = 'FAIL';
        summary.message{k} = err.message;
    end
    summary.time_s(k) = toc(t0);
end

%% ===== Report =====
fprintf('\n%-40s %-6s %9s  %s\n', 'Test', 'Result', 'Time [s]', 'Message');
fprintf('%s\n', repmat('-', 1, 90));
for k = 1:K
    fprintf('%-40s %-6s %9.2f  %s\n', summary.names{k}, summary.status{k}, ...
            summary.time_s(k), summary.message{k});
end
summary.n_pass = sum(strcmp(summary.status, 'PASS'));
summary.n_fail = sum(strcmp(summary.status, 'FAIL'));
summary.n_skip = sum(strcmp(summary.status, 'SKIP'));
fprintf('%s\n', repmat('-', 1, 90));
fprintf('PASS %d   FAIL %d   SKIP %d   (of %d)\n\n', summary.n_pass, summary.n_fail, ...
        summary.n_skip, K);
end
