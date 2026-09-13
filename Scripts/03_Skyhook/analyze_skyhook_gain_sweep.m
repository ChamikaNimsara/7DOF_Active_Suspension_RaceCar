%% Analyze Skyhook Gain Sweep
% Identifies best gains for individual performance metrics.

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
% Load Gain Sweep Results
% ==========================================================

sweepFile = fullfile( ...
    resultsFolder, ...
    'skyhook_gain_sweep.mat');

if ~isfile(sweepFile)
    error('skyhook_gain_sweep.mat not found.');
end

data = load(sweepFile);

T = data.resultsTable;


%% =========================================================
% Find Best Gain for Each Metric
% ==========================================================

[bestHeave, idxHeave] = min(T.peakHeave);

[bestPitch, idxPitch] = min(T.peakPitch);

[bestHeaveAccel, idxHeaveAccel] = ...
    min(T.rmsHeaveAccel);

[bestPitchAccel, idxPitchAccel] = ...
    min(T.rmsPitchAccel);

[bestSettling, idxSettling] = ...
    min(T.settlingTime);

[lowestForce, idxForce] = ...
    min(T.peakForce);


gainBestHeave = T.Gain_Nsm(idxHeave);
gainBestPitch = T.Gain_Nsm(idxPitch);

gainBestHeaveAccel = ...
    T.Gain_Nsm(idxHeaveAccel);

gainBestPitchAccel = ...
    T.Gain_Nsm(idxPitchAccel);

gainBestSettling = ...
    T.Gain_Nsm(idxSettling);

gainLowestForce = ...
    T.Gain_Nsm(idxForce);


%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('SKYHOOK GAIN SWEEP - BEST INDIVIDUAL RESULTS')
disp('====================================================')

fprintf( ...
    'Best peak heave:\n');

fprintf( ...
    '  C_sky = %.0f N*s/m\n', ...
    gainBestHeave);

fprintf( ...
    '  Peak heave = %.4f mm\n\n', ...
    bestHeave);


fprintf( ...
    'Best peak pitch:\n');

fprintf( ...
    '  C_sky = %.0f N*s/m\n', ...
    gainBestPitch);

fprintf( ...
    '  Peak pitch = %.5f deg\n\n', ...
    bestPitch);


fprintf( ...
    'Best RMS heave acceleration:\n');

fprintf( ...
    '  C_sky = %.0f N*s/m\n', ...
    gainBestHeaveAccel);

fprintf( ...
    '  RMS heave accel = %.5f m/s^2\n\n', ...
    bestHeaveAccel);


fprintf( ...
    'Best RMS pitch acceleration:\n');

fprintf( ...
    '  C_sky = %.0f N*s/m\n', ...
    gainBestPitchAccel);

fprintf( ...
    '  RMS pitch accel = %.5f deg/s^2\n\n', ...
    bestPitchAccel);


fprintf( ...
    'Best settling time:\n');

fprintf( ...
    '  C_sky = %.0f N*s/m\n', ...
    gainBestSettling);

fprintf( ...
    '  Settling time = %.3f s\n\n', ...
    bestSettling);


fprintf( ...
    'Lowest actuator effort:\n');

fprintf( ...
    '  C_sky = %.0f N*s/m\n', ...
    gainLowestForce);

fprintf( ...
    '  Peak force = %.2f N\n', ...
    lowestForce);

disp('====================================================')


%% =========================================================
% Create Best-Metric Results Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'Settling Time'
    'Peak Actuator Force'
    };

BestGain_Nsm = [
    gainBestHeave
    gainBestPitch
    gainBestHeaveAccel
    gainBestPitchAccel
    gainBestSettling
    gainLowestForce
    ];

BestValue = [
    bestHeave
    bestPitch
    bestHeaveAccel
    bestPitchAccel
    bestSettling
    lowestForce
    ];

Unit = {
    'mm'
    'deg'
    'm/s^2'
    'deg/s^2'
    's'
    'N'
    };

bestMetricsTable = table( ...
    Metric, ...
    BestGain_Nsm, ...
    BestValue, ...
    Unit);


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'skyhook_best_individual_metrics.csv');

writetable(bestMetricsTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'skyhook_best_individual_metrics.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'SKYHOOK GAIN SWEEP - BEST INDIVIDUAL METRICS\n');

fprintf(fid, ...
    '============================================\n\n');

for i = 1:height(bestMetricsTable)

    fprintf(fid, ...
        '%s\n', ...
        bestMetricsTable.Metric{i});

    fprintf(fid, ...
        'Best Gain  = %.0f N*s/m\n', ...
        bestMetricsTable.BestGain_Nsm(i));

    fprintf(fid, ...
        'Best Value = %.6f %s\n\n', ...
        bestMetricsTable.BestValue(i), ...
        bestMetricsTable.Unit{i});

end

fclose(fid);


%% =========================================================
% Normalize Metrics for Trade-off Visualization
% ==========================================================

% Lower values are better for all of these metrics.

normHeave = ...
    T.peakHeave ./ max(T.peakHeave);

normPitch = ...
    T.peakPitch ./ max(T.peakPitch);

normHeaveAccel = ...
    T.rmsHeaveAccel ./ max(T.rmsHeaveAccel);

normPitchAccel = ...
    T.rmsPitchAccel ./ max(T.rmsPitchAccel);

normSettling = ...
    T.settlingTime ./ max(T.settlingTime);

normForce = ...
    T.peakForce ./ F_act_max;


%% =========================================================
% Trade-off Plot
% ==========================================================

fig = figure( ...
    'Name', ...
    'Skyhook Gain Trade-off Analysis', ...
    'NumberTitle', ...
    'off');

plot( ...
    T.Gain_Nsm, ...
    normHeave, ...
    '-o', ...
    'LineWidth', 1.4);

hold on

plot( ...
    T.Gain_Nsm, ...
    normPitch, ...
    '-o', ...
    'LineWidth', 1.4);

plot( ...
    T.Gain_Nsm, ...
    normHeaveAccel, ...
    '-o', ...
    'LineWidth', 1.4);

plot( ...
    T.Gain_Nsm, ...
    normPitchAccel, ...
    '-o', ...
    'LineWidth', 1.4);

plot( ...
    T.Gain_Nsm, ...
    normSettling, ...
    '-o', ...
    'LineWidth', 1.4);

plot( ...
    T.Gain_Nsm, ...
    normForce, ...
    '-o', ...
    'LineWidth', 1.4);

grid on

xlabel('C_{sky} [N s/m]')
ylabel('Normalized Metric')

title('Skyhook Gain Performance Trade-offs')

legend( ...
    'Peak Heave', ...
    'Peak Pitch', ...
    'RMS Heave Accel', ...
    'RMS Pitch Accel', ...
    'Settling Time', ...
    'Actuator Utilization', ...
    'Location', ...
    'best');


%% =========================================================
% Save Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_tradeoff_analysis.png');

figFile = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_tradeoff_analysis.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', ...
    300);

savefig(fig, figFile);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'skyhook_gain_tradeoff_analysis.mat');

save(matFile, ...
    'bestMetricsTable', ...
    'normHeave', ...
    'normPitch', ...
    'normHeaveAccel', ...
    'normPitchAccel', ...
    'normSettling', ...
    'normForce');


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Gain trade-off analysis saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', pngFile);