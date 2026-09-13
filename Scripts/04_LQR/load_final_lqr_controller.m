%% Load Final Tuned LQR Controller

scriptFolder = fileparts(mfilename('fullpath'));
projectFolder = fileparts(fileparts(scriptFolder));

controllerFile = fullfile( ...
    projectFolder, ...
    'Results', ...
    'LQR', ...
    'final_tuned_lqr_controller.mat');

if ~isfile(controllerFile)

    error( ...
        ['Final tuned LQR controller not found. ' ...
        'Run lqr_final_candidate_kerb_sweep first.']);

end

data = load(controllerFile);

K_lqr = data.K_lqr_final;

final_lqr_ratio = data.finalRatio;

fprintf('============================================\n');
fprintf('FINAL TUNED LQR CONTROLLER LOADED\n');
fprintf('============================================\n');
fprintf('K_lqr size = %d x %d\n', ...
    size(K_lqr,1), size(K_lqr,2));

fprintf('Final Q/R ratio = %.3f\n', ...
    final_lqr_ratio);

fprintf('============================================\n');
