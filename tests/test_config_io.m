function test_config_io()
%TEST_CONFIG_IO Save/load round trip and default filling of missing fields (errors on failure).
%
% Inputs:
%   (none)
% Outputs:
%   (none; throws an error if any check fails)

%% ===== Temporary folder =====
root = fileparts(fileparts(mfilename('fullpath')));
tmpDir = fullfile(root, '_tmp');
if ~exist(tmpDir, 'dir')
    mkdir(tmpDir);
end
f1 = fullfile(tmpDir, 'test_config_io_1.mat');
f2 = fullfile(tmpDir, 'test_config_io_2.mat');

try
    %% ===== Round trip =====
    cfg = scn_disturbance_stress();
    cfg.gains.robust.K_diag = [1; 2; 3];                 % [N*m*s] non-default gain to check the round trip
    saveConfig(f1, cfg);
    c2 = loadConfig(f1);
    assert(isequal(c2, cfg), 'round trip changed the config');

    %% ===== Missing-field filling =====
    d = initDefaults();
    cfg.scenario = rmfield(cfg.scenario, 'k_norm');
    cfg.metrics = rmfield(cfg.metrics, 'ss_window_frac');
    cfg = rmfield(cfg, 'pass');
    saveConfig(f2, cfg);
    c3 = loadConfig(f2);
    assert(isfield(c3.scenario, 'k_norm') && c3.scenario.k_norm == d.scenario.k_norm, 'k_norm not filled');
    assert(c3.metrics.ss_window_frac == d.metrics.ss_window_frac, 'ss_window_frac not filled');
    assert(isequal(c3.pass, d.pass), 'pass struct not filled');
    assert(isequal(c3.scenario.dist_scale, [4 4 4 4]), 'existing field overwritten');
    assert(isequal(c3.gains.robust.K_diag, [1; 2; 3]), 'existing gain overwritten');
catch err
    localClean({f1, f2});
    rethrow(err);
end
localClean({f1, f2});
end

function localClean(files)
%LOCALCLEAN Delete temporary files if they exist.
%
% Inputs:
%   files - cell of char, file paths                                     [-]
% Outputs:
%   (none)

for k = 1:numel(files)
    if exist(files{k}, 'file')
        delete(files{k});
    end
end
end
