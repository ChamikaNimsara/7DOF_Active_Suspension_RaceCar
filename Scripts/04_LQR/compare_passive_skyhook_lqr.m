%% Passive vs Optimized Skyhook vs Initial LQR

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

resultsFolder = fullfile(projectFolder, 'Results', 'LQR');
plotsFolder = fullfile(projectFolder, 'Plots', 'LQR');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

%% =========================================================
% Load Results
% ==========================================================

passive = load(fullfile( ...
    passiveResultsFolder, ...
    'passive_baseline_summary.mat'));

skyhook = load(fullfile( ...
    skyhookResultsFolder, ...
    'skyhook_weighted_optimization.mat'));

lqr = load(fullfile( ...
    resultsFolder, ...
    'initial_lqr_bump_metrics.mat'));

%% =========================================================
% Build Comparison Table
% ==========================================================

Controller = {
    'Passive'
    'Optimized Skyhook'
    'Initial LQR'
    };

PeakHeave_mm = [
    passive.peak_heave_mm
    skyhook.optimal_peak_heave
    lqr.lqr_peak_heave_mm
    ];

PeakPitch_deg = [
    passive.peak_pitch_deg
    skyhook.optimal_peak_pitch
    lqr.lqr_peak_pitch_deg
    ];

RMSHeaveAccel_mps2 = [
    passive.rms_heave_accel
    skyhook.optimal_rms_heave_accel
    lqr.lqr_rms_heave_accel
    ];

RMSPitchAccel_degps2 = [
    passive.rms_pitch_accel
    skyhook.optimal_rms_pitch_accel
    lqr.lqr_rms_pitch_accel
    ];

SettlingTime_s = [
    passive.settling_time
    skyhook.optimal_settling_time
    lqr.lqr_settling_time
    ];

PeakActuatorForce_N = [
    0
    skyhook.optimal_peak_force
    lqr.lqr_peak_force
    ];

comparisonTable = table( ...
    Controller, ...
    PeakHeave_mm, ...
    PeakPitch_deg, ...
    RMSHeaveAccel_mps2, ...
    RMSPitchAccel_degps2, ...
    SettlingTime_s, ...
    PeakActuatorForce_N);

disp('====================================================')
disp('PASSIVE vs SKYHOOK vs INITIAL LQR')
disp('====================================================')
disp(comparisonTable)

%% =========================================================
% Relative Performance vs Passive
% ==========================================================

normHeave = ...
    100 * PeakHeave_mm / PeakHeave_mm(1);

normPitch = ...
    100 * PeakPitch_deg / PeakPitch_deg(1);

normHeaveAccel = ...
    100 * RMSHeaveAccel_mps2 / RMSHeaveAccel_mps2(1);

normPitchAccel = ...
    100 * RMSPitchAccel_degps2 / RMSPitchAccel_degps2(1);

normSettling = ...
    100 * SettlingTime_s / SettlingTime_s(1);

%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'passive_skyhook_initial_lqr_comparison.csv');

writetable(comparisonTable, csvFile);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'passive_skyhook_initial_lqr_comparison.mat');

save(matFile, ...
    'comparisonTable', ...
    'normHeave', ...
    'normPitch', ...
    'normHeaveAccel', ...
    'normPitchAccel', ...
    'normSettling');

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'passive_skyhook_initial_lqr_comparison.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'PASSIVE vs OPTIMIZED SKYHOOK vs INITIAL LQR\n');

fprintf(fid, ...
    '===========================================\n\n');

for i = 1:height(comparisonTable)

    fprintf(fid, '%s\n', ...
        comparisonTable.Controller{i});

    fprintf(fid, ...
        'Peak Heave = %.4f mm\n', ...
        comparisonTable.PeakHeave_mm(i));

    fprintf(fid, ...
        'Peak Pitch = %.5f deg\n', ...
        comparisonTable.PeakPitch_deg(i));

    fprintf(fid, ...
        'RMS Heave Accel = %.5f m/s^2\n', ...
        comparisonTable.RMSHeaveAccel_mps2(i));

    fprintf(fid, ...
        'RMS Pitch Accel = %.5f deg/s^2\n', ...
        comparisonTable.RMSPitchAccel_degps2(i));

    fprintf(fid, ...
        'Settling Time = %.3f s\n', ...
        comparisonTable.SettlingTime_s(i));

    fprintf(fid, ...
        'Peak Actuator Force = %.2f N\n\n', ...
        comparisonTable.PeakActuatorForce_N(i));

end

fclose(fid);

%% =========================================================
% Plot Performance Comparison
% ==========================================================

comparisonData = [ ...
    normHeave, ...
    normPitch, ...
    normHeaveAccel, ...
    normPitchAccel, ...
    normSettling];

fig1 = figure( ...
    'Name', ...
    'Passive vs Skyhook vs Initial LQR', ...
    'NumberTitle', ...
    'off');

bar(categorical(Controller), comparisonData);

grid on

ylabel('Metric [% of Passive]')

title('Passive vs Optimized Skyhook vs Initial LQR')

legend( ...
    'Peak Heave', ...
    'Peak Pitch', ...
    'RMS Heave Accel', ...
    'RMS Pitch Accel', ...
    'Settling Time', ...
    'Location', ...
    'bestoutside');

performancePNG = fullfile( ...
    plotsFolder, ...
    'passive_skyhook_initial_lqr_performance.png');

performanceFIG = fullfile( ...
    plotsFolder, ...
    'passive_skyhook_initial_lqr_performance.fig');

exportgraphics( ...
    fig1, ...
    performancePNG, ...
    'Resolution', 300);

savefig(fig1, performanceFIG);

%% =========================================================
% Plot Actuator Effort
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Controller Actuator Effort', ...
    'NumberTitle', ...
    'off');

bar(categorical(Controller), PeakActuatorForce_N);

hold on

yline(3000, '--', ...
    'Actuator Limit');

grid on

ylabel('Peak Actuator Force [N]')

title('Controller Peak Actuator Effort');

forcePNG = fullfile( ...
    plotsFolder, ...
    'skyhook_vs_lqr_actuator_effort.png');

forceFIG = fullfile( ...
    plotsFolder, ...
    'skyhook_vs_lqr_actuator_effort.fig');

exportgraphics( ...
    fig2, ...
    forcePNG, ...
    'Resolution', 300);

savefig(fig2, forceFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Controller comparison saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', performancePNG);
fprintf('%s\n', forcePNG);
