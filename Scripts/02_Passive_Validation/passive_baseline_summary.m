%% Passive Suspension Baseline Summary
% 7-DOF Active Suspension Race Car Project

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

resultsFolder = fullfile(projectFolder, 'Results', 'Passive');
plotsFolder = fullfile(projectFolder, 'Plots', 'Passive');

if ~exist(resultsFolder, 'dir')
    mkdir(resultsFolder);
end

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end


%% =========================================================
% Load Existing Results
% ==========================================================

bumpFile = fullfile( ...
    resultsFolder, ...
    'passive_bump_metrics.mat');

freqFile = fullfile( ...
    resultsFolder, ...
    'passive_frequency_results.mat');

if ~isfile(bumpFile)
    error('passive_bump_metrics.mat not found.');
end

if ~isfile(freqFile)
    error('passive_frequency_results.mat not found.');
end

bump = load(bumpFile);
freq = load(freqFile);


%% =========================================================
% Passive Baseline Metrics
% ==========================================================

peak_heave_mm = ...
    1000 * max(abs([ ...
    bump.peak_heave_pos, ...
    bump.peak_heave_neg]));

peak_pitch_deg = ...
    max(abs([ ...
    bump.peak_pitch_pos, ...
    bump.peak_pitch_neg]));

peak_heave_accel = ...
    max(abs([ ...
    bump.peak_heave_accel_pos, ...
    bump.peak_heave_accel_neg]));

peak_pitch_accel = ...
    max(abs([ ...
    bump.peak_pitch_accel_pos, ...
    bump.peak_pitch_accel_neg]));

rms_heave_accel = ...
    bump.rms_heave_accel;

rms_pitch_accel = ...
    bump.rms_pitch_accel;

settling_time = ...
    bump.settling_time_heave;

max_roll_deg = ...
    bump.max_abs_roll;

body_resonance_hz = ...
    freq.body_resonance_freq;

wheel_hop_hz = ...
    freq.wheel_hop_freq;

body_response_ratio = ...
    freq.body_peak_mag;

wheel_response_ratio = ...
    freq.wheel_peak_mag;


%% =========================================================
% Display Baseline
% ==========================================================

disp('====================================================')
disp('PASSIVE 7-DOF BASELINE SUMMARY')
disp('====================================================')

fprintf('Peak absolute heave        = %.3f mm\n', ...
    peak_heave_mm);

fprintf('Peak absolute pitch        = %.4f deg\n', ...
    peak_pitch_deg);

fprintf('Peak heave acceleration    = %.3f m/s^2\n', ...
    peak_heave_accel);

fprintf('RMS heave acceleration     = %.3f m/s^2\n', ...
    rms_heave_accel);

fprintf('Peak pitch acceleration    = %.3f deg/s^2\n', ...
    peak_pitch_accel);

fprintf('RMS pitch acceleration     = %.3f deg/s^2\n', ...
    rms_pitch_accel);

fprintf('Heave settling time        = %.3f s\n', ...
    settling_time);

fprintf('Maximum symmetric-test roll= %.8f deg\n', ...
    max_roll_deg);

fprintf('Body resonance             = %.3f Hz\n', ...
    body_resonance_hz);

fprintf('Wheel-hop region           = %.3f Hz\n', ...
    wheel_hop_hz);

fprintf('Body response ratio        = %.3f\n', ...
    body_response_ratio);

fprintf('Wheel response ratio       = %.3f\n', ...
    wheel_response_ratio);

disp('====================================================')


%% =========================================================
% Save Baseline CSV
% ==========================================================

Metric = {
    'Peak Absolute Heave'
    'Peak Absolute Pitch'
    'Peak Heave Acceleration'
    'RMS Heave Acceleration'
    'Peak Pitch Acceleration'
    'RMS Pitch Acceleration'
    'Heave Settling Time'
    'Maximum Roll - Symmetric Bump'
    'Body Resonance Frequency'
    'Wheel-Hop Frequency'
    'Body Response Ratio'
    'Wheel Response Ratio'
    };

Value = [
    peak_heave_mm
    peak_pitch_deg
    peak_heave_accel
    rms_heave_accel
    peak_pitch_accel
    rms_pitch_accel
    settling_time
    max_roll_deg
    body_resonance_hz
    wheel_hop_hz
    body_response_ratio
    wheel_response_ratio
    ];

Unit = {
    'mm'
    'deg'
    'm/s^2'
    'm/s^2'
    'deg/s^2'
    'deg/s^2'
    's'
    'deg'
    'Hz'
    'Hz'
    '-'
    '-'
    };

baselineTable = table(Metric, Value, Unit);

csvFile = fullfile( ...
    resultsFolder, ...
    'passive_baseline_summary.csv');

writetable(baselineTable, csvFile);


%% =========================================================
% Save Human-Readable TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'passive_baseline_summary.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'PASSIVE 7-DOF SUSPENSION BASELINE\n');

fprintf(fid, ...
    '=================================\n\n');

fprintf(fid, ...
    'Symmetric Bump Test\n');

fprintf(fid, ...
    '-------------------\n');

fprintf(fid, ...
    'Peak absolute heave       = %.3f mm\n', ...
    peak_heave_mm);

fprintf(fid, ...
    'Peak absolute pitch       = %.4f deg\n', ...
    peak_pitch_deg);

fprintf(fid, ...
    'Peak heave acceleration   = %.3f m/s^2\n', ...
    peak_heave_accel);

fprintf(fid, ...
    'RMS heave acceleration    = %.3f m/s^2\n', ...
    rms_heave_accel);

fprintf(fid, ...
    'Peak pitch acceleration   = %.3f deg/s^2\n', ...
    peak_pitch_accel);

fprintf(fid, ...
    'RMS pitch acceleration    = %.3f deg/s^2\n', ...
    rms_pitch_accel);

fprintf(fid, ...
    'Heave settling time       = %.3f s\n', ...
    settling_time);

fprintf(fid, ...
    'Maximum roll              = %.8f deg\n\n', ...
    max_roll_deg);

fprintf(fid, ...
    'Frequency Sweep\n');

fprintf(fid, ...
    '---------------\n');

fprintf(fid, ...
    'Body resonance            = %.3f Hz\n', ...
    body_resonance_hz);

fprintf(fid, ...
    'Wheel-hop frequency       = %.3f Hz\n', ...
    wheel_hop_hz);

fprintf(fid, ...
    'Body response ratio       = %.3f\n', ...
    body_response_ratio);

fprintf(fid, ...
    'Wheel response ratio      = %.3f\n', ...
    wheel_response_ratio);

fclose(fid);


%% =========================================================
% Save MATLAB Baseline
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'passive_baseline_summary.mat');

save(matFile, ...
    'peak_heave_mm', ...
    'peak_pitch_deg', ...
    'peak_heave_accel', ...
    'rms_heave_accel', ...
    'peak_pitch_accel', ...
    'rms_pitch_accel', ...
    'settling_time', ...
    'max_roll_deg', ...
    'body_resonance_hz', ...
    'wheel_hop_hz', ...
    'body_response_ratio', ...
    'wheel_response_ratio');


%% =========================================================
% Create Repository-Friendly Summary Plot
% ==========================================================

fig = figure( ...
    'Name', ...
    'Passive Baseline Summary', ...
    'NumberTitle', ...
    'off');

categories = categorical({ ...
    'Peak Heave [mm]', ...
    'RMS Heave Accel [m/s^2]', ...
    'Peak Pitch [deg]', ...
    'Settling Time [s]'});

values = [ ...
    peak_heave_mm, ...
    rms_heave_accel, ...
    peak_pitch_deg, ...
    settling_time];

bar(categories, values);

grid on

ylabel('Metric Value')

title('Passive Suspension Baseline Metrics')


plotPNG = fullfile( ...
    plotsFolder, ...
    'passive_baseline_summary.png');

plotFIG = fullfile( ...
    plotsFolder, ...
    'passive_baseline_summary.fig');

exportgraphics(fig, ...
    plotPNG, ...
    'Resolution', 300);

savefig(fig, plotFIG);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Passive baseline successfully consolidated.')

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', plotPNG);