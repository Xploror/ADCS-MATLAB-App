% Adds the ADCS-MATLAB-App folders to the path and print usage.

adcs_root = fileparts(mfilename('fullpath'));   % absolute path of the project root

%% ===== Add folders to the path =====
addpath(genpath(fullfile(adcs_root, 'src')));
addpath(genpath(fullfile(adcs_root, 'config')));
adcs_extra = {'app', 'models', 'tests', 'examples'};   % non-recursive folders
for adcs_k = 1:numel(adcs_extra)
    adcs_dir = fullfile(adcs_root, adcs_extra{adcs_k});
    if exist(adcs_dir, 'dir')
        addpath(adcs_dir);
    end
end

%% ===== Usage banner =====
fprintf('ADCS-MATLAB-App: paths added (root: %s)\n', adcs_root);
fprintf('  cfg = scn_nominal();  results = runSimulation(cfg, [1 2 3], ''auto'');\n');
fprintf('  run_example_comparison          %% scripted comparison with plots\n');
fprintf('  run_all_tests                   %% unit tests (Simulink cross-check if available)\n');
fprintf('  run_all_scenarios_smoketest     %% all scenarios x all controllers\n');
fprintf('  ADCS_MATLAB_App                 %% GUI (MATLAB only)\n');
clear adcs_root adcs_extra adcs_k adcs_dir
