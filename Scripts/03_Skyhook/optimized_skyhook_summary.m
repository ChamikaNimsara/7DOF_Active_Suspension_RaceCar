%% Optimized Skyhook Controller Summary
% Compares:
% Passive Suspension
% Initial Skyhook Controller
% Optimized Skyhook Controller

clc;

%% =========================================================
% Project Folders
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
% Load Passive Baseline
% ==========================================================

passiveFile = fullfile( ...
    passiveResultsFolder, ...
    'passive_baseline_summary.mat');

if ~isfile(passiveFile)
    error('passive_baseline_summary.mat not found.');
end

passive = load(passiveFile);


%% =========================================================
% Load Initial Skyhook Baseline
% ==========================================================

initialFile = fullfile( ...
    resultsFolder, ...
    'skyhook_baseline_summary.mat');

if ~isfile(initialFile)
    error('skyhook_baseline_summary.mat not found.');
end

initial = load(initialFile);


%% =========================================================
% Load Optimized Skyhook Result
% ==========================================================

optFile = fullfile( ...
    resultsFolder, ...
    'skyhook_weighted_optimization.mat');

if ~isfile(optFile)
    error('skyhook_weighted_optimization.mat not found.');
end

opt = load(optFile);


%% =========================================================
% Consolidated Metrics
% ==========================================================

Controller = {
    'Passive'
    'Initial Skyhook'
    'Optimized Skyhook'
    };

Gain_Nsm = [
    0
    initial.controller_gain
    opt.optimal_gain
    ];

PeakHeave_mm = [
    passive.peak_heave_mm
    initial.peak_heave_mm
    opt.optimal_peak_heave
    ];

PeakPitch_deg = [
    passive.peak_pitch_deg
    initial.peak_pitch_deg
    opt.optimal_peak_pitch
    ];

RMSHeaveAccel_mps2 = [
    passive.rms_heave_accel
    initial.rms_heave_accel
    opt.optimal_rms_heave_accel
    ];

RMSPitchAccel_degps2 = [
    passive.rms_pitch_accel
    initial.rms_pitch_accel
    opt.optimal_rms_pitch_accel
    ];

SettlingTime_s = [
    passive.settling_time
    initial.settling_time
    opt.optimal_settling_time
    ];

PeakActuatorForce_N = [
    0
    initial.peak_force
    opt.optimal_peak_force
    ];

summaryTable = table( ...
    Controller, ...
    Gain_Nsm, ...
    PeakHeave_mm, ...
    PeakPitch_deg, ...
    RMSHeaveAccel_mps2, ...
    RMSPitchAccel_degps2, ...
    SettlingTime_s, ...
    PeakActuatorForce_N);


%% =========================================================
% Display Summary
% ==========================================================

disp('====================================================')
disp('PASSIVE vs INITIAL vs OPTIMIZED SKYHOOK')
disp('====================================================')

disp(summaryTable)

disp(' ')

fprintf('Optimized Gain = %.0f N*s/m\n\n', ...
    opt.optimal_gain);

fprintf('Optimized improvement vs Passive:\n');

fprintf('Peak Heave        = %.2f %%\n', ...
    opt.heave_improvement);

fprintf('Peak Pitch        = %.2f %%\n', ...
    opt.pitch_improvement);

fprintf('RMS Heave Accel   = %.2f %%\n', ...
    opt.heave_accel_improvement);

fprintf('RMS Pitch Accel   = %.2f %%\n', ...
    opt.pitch_accel_improvement);

fprintf('Settling Time     = %.2f %%\n', ...
    opt.settling_improvement);

fprintf('Peak Actuator     = %.2f N\n', ...
    opt.optimal_peak_force);

disp('====================================================')


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'optimized_skyhook_summary.csv');

writetable(summaryTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'optimized_skyhook_summary.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'PASSIVE vs INITIAL vs OPTIMIZED SKYHOOK\n');

fprintf(fid, ...
    '=======================================\n\n');

for i = 1:height(summaryTable)

    fprintf(fid, ...
        '%s\n', ...
        summaryTable.Controller{i});

    fprintf(fid, ...
        'Gain = %.0f N*s/m\n', ...
        summaryTable.Gain_Nsm(i));

    fprintf(fid, ...
        'Peak Heave = %.4f mm\n', ...
        summaryTable.PeakHeave_mm(i));

    fprintf(fid, ...
        'Peak Pitch = %.5f deg\n', ...
        summaryTable.PeakPitch_deg(i));

    fprintf(fid, ...
        'RMS Heave Accel = %.5f m/s^2\n', ...
        summaryTable.RMSHeaveAccel_mps2(i));

    fprintf(fid, ...
        'RMS Pitch Accel = %.5f deg/s^2\n', ...
        summaryTable.RMSPitchAccel_degps2(i));

    fprintf(fid, ...
        'Settling Time = %.3f s\n', ...
        summaryTable.SettlingTime_s(i));

    fprintf(fid, ...
        'Peak Actuator Force = %.2f N\n\n', ...
        summaryTable.PeakActuatorForce_N(i));

end

fprintf(fid, ...
    'Optimized gain = %.0f N*s/m\n\n', ...
    opt.optimal_gain);

fprintf(fid, ...
    'Peak Heave Improvement = %.2f %%\n', ...
    opt.heave_improvement);

fprintf(fid, ...
    'Peak Pitch Improvement = %.2f %%\n', ...
    opt.pitch_improvement);

fprintf(fid, ...
    'RMS Heave Accel Improvement = %.2f %%\n', ...
    opt.heave_accel_improvement);

fprintf(fid, ...
    'RMS Pitch Accel Improvement = %.2f %%\n', ...
    opt.pitch_accel_improvement);

fprintf(fid, ...
    'Settling Improvement = %.2f %%\n', ...
    opt.settling_improvement);

fclose(fid);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'optimized_skyhook_summary.mat');

save(matFile, ...
    'summaryTable', ...
    'Controller', ...
    'Gain_Nsm', ...
    'PeakHeave_mm', ...
    'PeakPitch_deg', ...
    'RMSHeaveAccel_mps2', ...
    'RMSPitchAccel_degps2', ...
    'SettlingTime_s', ...
    'PeakActuatorForce_N');


%% =========================================================
% Normalized Comparison
% ==========================================================

% Passive is used as 100% reference.

normPeakHeave = ...
    100 * PeakHeave_mm ./ PeakHeave_mm(1);

normPeakPitch = ...
    100 * PeakPitch_deg ./ PeakPitch_deg(1);

normHeaveAccel = ...
    100 * RMSHeaveAccel_mps2 ./ RMSHeaveAccel_mps2(1);

normPitchAccel = ...
    100 * RMSPitchAccel_degps2 ./ RMSPitchAccel_degps2(1);

normSettling = ...
    100 * SettlingTime_s ./ SettlingTime_s(1);


%% =========================================================
% Comparison Plot
% ==========================================================

fig = figure( ...
    'Name', ...
    'Passive vs Skyhook Controllers', ...
    'NumberTitle', ...
    'off');

comparisonData = [ ...
    normPeakHeave, ...
    normPeakPitch, ...
    normHeaveAccel, ...
    normPitchAccel, ...
    normSettling];

bar(categorical(Controller), ...
    comparisonData);

grid on

ylabel('Metric [% of Passive]')

title('Passive vs Initial vs Optimized Skyhook')

legend( ...
    'Peak Heave', ...
    'Peak Pitch', ...
    'RMS Heave Accel', ...
    'RMS Pitch Accel', ...
    'Settling Time', ...
    'Location', ...
    'bestoutside');


%% =========================================================
% Save Comparison Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'passive_initial_optimized_skyhook_comparison.png');

figFile = fullfile( ...
    plotsFolder, ...
    'passive_initial_optimized_skyhook_comparison.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', ...
    300);

savefig(fig, figFile);


%% =========================================================
% Improvement Plot
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Optimized Skyhook Improvements', ...
    'NumberTitle', ...
    'off');

improvements = [ ...
    opt.heave_improvement, ...
    opt.pitch_improvement, ...
    opt.heave_accel_improvement, ...
    opt.pitch_accel_improvement, ...
    opt.settling_improvement];

categories = categorical({ ...
    'Peak Heave', ...
    'Peak Pitch', ...
    'RMS Heave Accel', ...
    'RMS Pitch Accel', ...
    'Settling Time'});

bar(categories, improvements);

grid on

ylabel('Improvement vs Passive [%]')

title('Optimized Skyhook Performance Improvement')


%% =========================================================
% Save Improvement Plot
% ==========================================================

improvementPNG = fullfile( ...
    plotsFolder, ...
    'optimized_skyhook_improvement.png');

improvementFIG = fullfile( ...
    plotsFolder, ...
    'optimized_skyhook_improvement.fig');

exportgraphics( ...
    fig2, ...
    improvementPNG, ...
    'Resolution', ...
    300);

savefig(fig2, improvementFIG);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Optimized Skyhook summary saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', pngFile);
fprintf('%s\n', improvementPNG);
