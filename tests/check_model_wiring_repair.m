function passed = check_model_wiring_repair()
%CHECK_MODEL_WIRING_REPAIR Opt-in check of the model builder's post-grouping wire verify-and-repair path.
%
% Inputs:
%   (none)
% Outputs:
%   passed - logical [1], true if all checks pass (failures raise errors)  [-]
%
% NOT auto-discovered by run_all_tests (its name does not start with
% "test_") because it rebuilds the Simulink model twice (minutes). Steps:
%   1. build_ADCS_model(true, true): the test hook re-routes TrueErr input
%      w_r to the q_r signal after grouping (the defect seen in R2026a, see
%      notes.md I12). The build must detect and repair exactly this wire and
%      leave no unconnected port (checkNoOpenPorts runs inside the build).
%   2. test_simulink_vs_reference must PASS on the repaired model.
%   3. build_ADCS_model(true) rebuilds the clean model for normal use.
% Prints SKIPPED and returns true when Simulink is not available.

%% ===== Skip without Simulink =====
passed = true;
if ~isSimulinkAvailable()
    fprintf('check_model_wiring_repair: SKIPPED (Simulink not available)\n');
    return;
end

%% ===== 1. Build with the injected fault =====
[~, info] = build_ADCS_model(true, true);
fprintf('check_model_wiring_repair: injected build verified %d wires, repaired %d\n', ...
        info.nWires, info.nRepaired);
assert(info.nRepaired >= 1, 'check_model_wiring_repair: the injected mis-wire was not repaired');

%% ===== 2. The repaired model must match the reference engine =====
test_simulink_vs_reference();

%% ===== 3. Restore the clean model =====
[~, info2] = build_ADCS_model(true);
fprintf('check_model_wiring_repair: clean rebuild verified %d wires, repaired %d\n', ...
        info2.nWires, info2.nRepaired);
fprintf('check_model_wiring_repair: PASSED\n');
end
