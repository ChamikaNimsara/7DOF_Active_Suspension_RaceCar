%% Initial Skyhook Controller Baseline Summary

clc;

%% =========================================================
% Project folders
% ==========================================================

scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
projectFolder = fileparts(fileparts(scriptFolder));

passiveResultsFolder = fullfile(projectFolder, 'Results', 'Passive');
skyhookResultsFolder = fullfile(projectFolder, 'Results', 'Skyhook');
lqrResultsFolder = fullfile(projectFolder, 'Results', 'LQR');
robustnessResultsFolder = fullfile(projectFolder, 'Results', 'Robustness');
operatingResultsFolder = fullfile(projectFolder, 'Results', 'Operating_Envelope');
finalResultsFolder = fullfile(projectFolder, 'Results', 'Final');

resultsFolder = fullfile(projectFolder, 'Results', 'Skyhook');
plotsFolder = fullfile(projectFolder, 'Plots', 'Skyhook');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end


%% =========================================================
% Load Active vs Passive Results
% ==========================================================

comparisonFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_bump_metrics.mat');

if ~isfile(comparisonFile)
    error('active_vs_passive_bump_metrics.mat not found.');
end

data = load(comparisonFile);


%% =========================================================
% Load Actuator Metrics
% ==========================================================

forceFile = fullfile( ...
    resultsFolder, ...
    'initial_skyhook_force_metrics.mat');

if ~isfile(forceFile)
    error('initial_skyhook_force_metrics.mat not found.');
end

forces = load(forceFile);


%% =========================================================
% Controller Parameters
% ==========================================================

controller_name = 'Skyhook';
controller_gain = C_sky;
actuator_limit = F_act_max;


%% =========================================================
% Extract Performance Metrics
% ==========================================================

peak_heave_mm = data.active_peak_heave_mm;
peak_pitch_deg = data.active_peak_pitch_deg;

rms_heave_accel = ...
    data.active_rms_heave_accel;

rms_pitch_accel = ...
    data.active_rms_pitch_accel;

settling_time = ...
    data.active_settling_time;

heave_improvement = ...
    data.heave_improvement_pct;

pitch_improvement = ...
    data.pitch_improvement_pct;

heave_accel_improvement = ...
    data.heave_accel_improvement_pct;

pitch_accel_improvement = ...
    data.pitch_accel_improvement_pct;

settling_improvement = ...
    data.settling_improvement_pct;


%% =========================================================
% Actuator Effort
% ==========================================================

peak_force = max([ ...
    forces.peak_FL, ...
    forces.peak_FR, ...
    forces.peak_RL, ...
    forces.peak_RR]);

mean_rms_force = mean([ ...
    forces.rms_FL, ...
    forces.rms_FR, ...
    forces.rms_RL, ...
    forces.rms_RR]);

max_saturation = max([ ...
    forces.sat_FL, ...
    forces.sat_FR, ...
    forces.sat_RL, ...
    forces.sat_RR]);


%% =========================================================
% Display Summary
% ==========================================================

disp('====================================================')
disp('INITIAL SKYHOOK CONTROLLER BASELINE')
disp('====================================================')

fprintf('Controller gain          = %.1f N*s/m\n', ...
    controller_gain);

fprintf('Actuator limit           = %.0f N\n\n', ...
    actuator_limit);

fprintf('Peak heave               = %.3f mm\n', ...
    peak_heave_mm);

fprintf('Peak pitch               = %.4f deg\n', ...
    peak_pitch_deg);

fprintf('RMS heave acceleration   = %.3f m/s^2\n', ...
    rms_heave_accel);

fprintf('RMS pitch acceleration   = %.3f deg/s^2\n', ...
    rms_pitch_accel);

fprintf('Settling time            = %.3f s\n\n', ...
    settling_time);

fprintf('Peak actuator force      = %.2f N\n', ...
    peak_force);

fprintf('Mean RMS actuator force  = %.2f N\n', ...
    mean_rms_force);

fprintf('Maximum saturation       = %.3f %%\n\n', ...
    max_saturation);

fprintf('HEAVE IMPROVEMENT        = %.2f %%\n', ...
    heave_improvement);

fprintf('PITCH IMPROVEMENT        = %.2f %%\n', ...
    pitch_improvement);

fprintf('HEAVE ACCEL IMPROVEMENT  = %.2f %%\n', ...
    heave_accel_improvement);

fprintf('PITCH ACCEL IMPROVEMENT  = %.2f %%\n', ...
    pitch_accel_improvement);

fprintf('SETTLING IMPROVEMENT     = %.2f %%\n', ...
    settling_improvement);

disp('====================================================')


%% =========================================================
% Save Summary Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'Settling Time'
    'Peak Actuator Force'
    'Mean RMS Actuator Force'
    'Maximum Saturation'
    };

Value = [
    peak_heave_mm
    peak_pitch_deg
    rms_heave_accel
    rms_pitch_accel
    settling_time
    peak_force
    mean_rms_force
    max_saturation
    ];

Unit = {
    'mm'
    'deg'
    'm/s^2'
    'deg/s^2'
    's'
    'N'
    'N'
    '%'
    };

summaryTable = table( ...
    Metric, ...
    Value, ...
    Unit);


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'skyhook_baseline_summary.csv');

writetable(summaryTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'skyhook_baseline_summary.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, 'INITIAL SKYHOOK CONTROLLER BASELINE\n');
fprintf(fid, '===================================\n\n');

fprintf(fid, 'C_sky = %.1f N*s/m\n', controller_gain);
fprintf(fid, 'Actuator limit = %.0f N\n\n', actuator_limit);

fprintf(fid, 'Peak heave = %.3f mm\n', peak_heave_mm);
fprintf(fid, 'Peak pitch = %.4f deg\n', peak_pitch_deg);

fprintf(fid, ...
    'RMS heave acceleration = %.3f m/s^2\n', ...
    rms_heave_accel);

fprintf(fid, ...
    'RMS pitch acceleration = %.3f deg/s^2\n', ...
    rms_pitch_accel);

fprintf(fid, ...
    'Settling time = %.3f s\n\n', ...
    settling_time);

fprintf(fid, ...
    'Peak actuator force = %.2f N\n', ...
    peak_force);

fprintf(fid, ...
    'Mean RMS actuator force = %.2f N\n', ...
    mean_rms_force);

fprintf(fid, ...
    'Maximum saturation = %.3f %%\n\n', ...
    max_saturation);

fprintf(fid, ...
    'Peak heave improvement = %.2f %%\n', ...
    heave_improvement);

fprintf(fid, ...
    'Peak pitch improvement = %.2f %%\n', ...
    pitch_improvement);

fprintf(fid, ...
    'RMS heave acceleration improvement = %.2f %%\n', ...
    heave_accel_improvement);

fprintf(fid, ...
    'RMS pitch acceleration improvement = %.2f %%\n', ...
    pitch_accel_improvement);

fprintf(fid, ...
    'Settling time improvement = %.2f %%\n', ...
    settling_improvement);

fclose(fid);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'skyhook_baseline_summary.mat');

save(matFile, ...
    'summaryTable', ...
    'controller_name', ...
    'controller_gain', ...
    'actuator_limit', ...
    'peak_heave_mm', ...
    'peak_pitch_deg', ...
    'rms_heave_accel', ...
    'rms_pitch_accel', ...
    'settling_time', ...
    'peak_force', ...
    'mean_rms_force', ...
    'max_saturation', ...
    'heave_improvement', ...
    'pitch_improvement', ...
    'heave_accel_improvement', ...
    'pitch_accel_improvement', ...
    'settling_improvement');


%% =========================================================
% Improvement Plot
% ==========================================================

improvements = [ ...
    heave_improvement, ...
    pitch_improvement, ...
    heave_accel_improvement, ...
    pitch_accel_improvement, ...
    settling_improvement];

categories = categorical({ ...
    'Peak Heave', ...
    'Peak Pitch', ...
    'RMS Heave Accel', ...
    'RMS Pitch Accel', ...
    'Settling Time'});

fig = figure( ...
    'Name', ...
    'Skyhook Improvement Summary', ...
    'NumberTitle', ...
    'off');

bar(categories, improvements);

grid on

ylabel('Improvement vs Passive [%]')

title('Initial Skyhook Controller Improvement')


%% =========================================================
% Save Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'skyhook_improvement_summary.png');

figFile = fullfile( ...
    plotsFolder, ...
    'skyhook_improvement_summary.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', 300);

savefig(fig, figFile);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Skyhook controller baseline saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', pngFile);