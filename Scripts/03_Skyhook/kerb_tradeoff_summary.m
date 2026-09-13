%% Kerb Test Trade-Off Summary

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
% Load Active vs Passive Kerb Results
% ==========================================================

dataFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_kerb_metrics.mat');

if ~isfile(dataFile)
    error('active_vs_passive_kerb_metrics.mat not found.');
end

data = load(dataFile);


%% =========================================================
% Extract Key Improvements
% ==========================================================

peakHeaveImprovement = data.heave_improvement;
peakPitchImprovement = data.pitch_improvement;
peakRollImprovement = data.roll_improvement;

heaveAccelImprovement = data.heave_accel_improvement;
pitchAccelImprovement = data.pitch_accel_improvement;
rollAccelImprovement = data.roll_accel_improvement;

rollSettlingImprovement = data.settling_improvement;

peakActuatorForce = data.active_peak_force;
meanRMSActuatorForce = data.active_mean_rms_force;
maxSaturation = data.active_max_saturation;


%% =========================================================
% Display Summary
% ==========================================================

disp('====================================================')
disp('KERB TEST CONTROL TRADE-OFF SUMMARY')
disp('====================================================')

fprintf('Peak heave reduction       = %.2f %%\n', ...
    peakHeaveImprovement);

fprintf('Peak pitch reduction       = %.2f %%\n', ...
    peakPitchImprovement);

fprintf('Peak roll reduction        = %.2f %%\n', ...
    peakRollImprovement);

fprintf('RMS heave accel reduction  = %.2f %%\n', ...
    heaveAccelImprovement);

fprintf('RMS pitch accel reduction  = %.2f %%\n', ...
    pitchAccelImprovement);

fprintf('RMS roll accel reduction   = %.2f %%\n', ...
    rollAccelImprovement);

fprintf('Roll settling improvement  = %.2f %%\n\n', ...
    rollSettlingImprovement);

fprintf('Peak actuator force        = %.2f N\n', ...
    peakActuatorForce);

fprintf('Mean RMS actuator force    = %.2f N\n', ...
    meanRMSActuatorForce);

fprintf('Maximum actuator saturation= %.3f %%\n', ...
    maxSaturation);

disp('====================================================')


%% =========================================================
% Save Summary Table
% ==========================================================

Metric = {
    'Peak Heave Reduction'
    'Peak Pitch Reduction'
    'Peak Roll Reduction'
    'RMS Heave Acceleration Reduction'
    'RMS Pitch Acceleration Reduction'
    'RMS Roll Acceleration Reduction'
    'Roll Settling Time Improvement'
    'Peak Actuator Force'
    'Mean RMS Actuator Force'
    'Maximum Saturation'
    };

Value = [
    peakHeaveImprovement
    peakPitchImprovement
    peakRollImprovement
    heaveAccelImprovement
    pitchAccelImprovement
    rollAccelImprovement
    rollSettlingImprovement
    peakActuatorForce
    meanRMSActuatorForce
    maxSaturation
    ];

Unit = {
    '%'
    '%'
    '%'
    '%'
    '%'
    '%'
    '%'
    'N'
    'N'
    '%'
    };

summaryTable = table(Metric, Value, Unit);


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'kerb_tradeoff_summary.csv');

writetable(summaryTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'kerb_tradeoff_summary.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, 'KERB TEST CONTROL TRADE-OFF SUMMARY\n');
fprintf(fid, '===================================\n\n');

for i = 1:height(summaryTable)

    fprintf(fid, ...
        '%s = %.4f %s\n', ...
        summaryTable.Metric{i}, ...
        summaryTable.Value(i), ...
        summaryTable.Unit{i});

end

fclose(fid);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'kerb_tradeoff_summary.mat');

save(matFile, ...
    'summaryTable', ...
    'peakHeaveImprovement', ...
    'peakPitchImprovement', ...
    'peakRollImprovement', ...
    'heaveAccelImprovement', ...
    'pitchAccelImprovement', ...
    'rollAccelImprovement', ...
    'rollSettlingImprovement', ...
    'peakActuatorForce', ...
    'meanRMSActuatorForce', ...
    'maxSaturation');


%% =========================================================
% Improvement Plot
% ==========================================================

improvementValues = [ ...
    peakHeaveImprovement, ...
    peakPitchImprovement, ...
    peakRollImprovement, ...
    heaveAccelImprovement, ...
    pitchAccelImprovement, ...
    rollAccelImprovement, ...
    rollSettlingImprovement];

categories = categorical({ ...
    'Peak Heave', ...
    'Peak Pitch', ...
    'Peak Roll', ...
    'RMS Heave Accel', ...
    'RMS Pitch Accel', ...
    'RMS Roll Accel', ...
    'Roll Settling'});

fig = figure( ...
    'Name', ...
    'Kerb Control Improvement Summary', ...
    'NumberTitle', ...
    'off');

bar(categories, improvementValues);

grid on

ylabel('Improvement vs Passive [%]')

title('Optimized Skyhook Performance on Asymmetric Kerb')


%% =========================================================
% Save Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'kerb_control_improvement_summary.png');

figFile = fullfile( ...
    plotsFolder, ...
    'kerb_control_improvement_summary.fig');

exportgraphics(fig, ...
    pngFile, ...
    'Resolution', 300);

savefig(fig, figFile);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Kerb trade-off summary saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', pngFile);