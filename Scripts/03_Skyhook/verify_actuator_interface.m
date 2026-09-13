%% Verify Actuator Interface - Passive Mode Equivalence
% Confirms that active_mode = 0 reproduces the stored passive baseline.

clc;

%% =========================================================
% Locate Project Folders
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
% Load Stored Passive Baseline
% ==========================================================

baselineFile = fullfile( ...
    passiveResultsFolder, ...
    'passive_baseline_summary.mat');

if ~isfile(baselineFile)
    error('passive_baseline_summary.mat not found.');
end

baseline = load(baselineFile);


%% =========================================================
% Calculate Current Simulation Metrics
% ==========================================================

current_peak_heave_mm = ...
    1000 * max(abs(z_s_out.Data));

current_peak_pitch_deg = ...
    max(abs(rad2deg(theta_out.Data)));

current_rms_heave_accel = ...
    rms(z_s_ddot_out.Data);

current_rms_pitch_accel = ...
    rms(rad2deg(theta_ddot_out.Data));

current_max_roll_deg = ...
    max(abs(rad2deg(phi_out.Data)));


%% =========================================================
% Stored Baseline Values
% ==========================================================

baseline_peak_heave_mm = ...
    baseline.peak_heave_mm;

baseline_peak_pitch_deg = ...
    baseline.peak_pitch_deg;

baseline_rms_heave_accel = ...
    baseline.rms_heave_accel;

baseline_rms_pitch_accel = ...
    baseline.rms_pitch_accel;

baseline_max_roll_deg = ...
    baseline.max_roll_deg;


%% =========================================================
% Percentage Differences
% ==========================================================

heave_error_pct = ...
    100 * abs( ...
    current_peak_heave_mm - baseline_peak_heave_mm) ...
    / baseline_peak_heave_mm;

pitch_error_pct = ...
    100 * abs( ...
    current_peak_pitch_deg - baseline_peak_pitch_deg) ...
    / baseline_peak_pitch_deg;

heave_accel_error_pct = ...
    100 * abs( ...
    current_rms_heave_accel - baseline_rms_heave_accel) ...
    / baseline_rms_heave_accel;

pitch_accel_error_pct = ...
    100 * abs( ...
    current_rms_pitch_accel - baseline_rms_pitch_accel) ...
    / baseline_rms_pitch_accel;


%% =========================================================
% Pass / Fail Threshold
% ==========================================================

tolerance_pct = 0.1;

pass_heave = ...
    heave_error_pct <= tolerance_pct;

pass_pitch = ...
    pitch_error_pct <= tolerance_pct;

pass_heave_accel = ...
    heave_accel_error_pct <= tolerance_pct;

pass_pitch_accel = ...
    pitch_accel_error_pct <= tolerance_pct;

overall_pass = ...
    pass_heave && ...
    pass_pitch && ...
    pass_heave_accel && ...
    pass_pitch_accel;


%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('ACTUATOR INTERFACE PASSIVE-MODE VERIFICATION')
disp('====================================================')

fprintf( ...
    'Peak heave error       = %.6f %%\n', ...
    heave_error_pct);

fprintf( ...
    'Peak pitch error       = %.6f %%\n', ...
    pitch_error_pct);

fprintf( ...
    'RMS heave accel error  = %.6f %%\n', ...
    heave_accel_error_pct);

fprintf( ...
    'RMS pitch accel error  = %.6f %%\n', ...
    pitch_accel_error_pct);

fprintf( ...
    'Maximum current roll   = %.8f deg\n\n', ...
    current_max_roll_deg);

if overall_pass

    disp('RESULT: PASS')
    disp('Actuator architecture preserves passive behaviour.')

else

    disp('RESULT: FAIL')
    disp('Check actuator routing or suspension force signs.')

end

disp('====================================================')


%% =========================================================
% Create Results Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    };

Baseline = [
    baseline_peak_heave_mm
    baseline_peak_pitch_deg
    baseline_rms_heave_accel
    baseline_rms_pitch_accel
    ];

Current = [
    current_peak_heave_mm
    current_peak_pitch_deg
    current_rms_heave_accel
    current_rms_pitch_accel
    ];

ErrorPercent = [
    heave_error_pct
    pitch_error_pct
    heave_accel_error_pct
    pitch_accel_error_pct
    ];

Unit = {
    'mm'
    'deg'
    'm/s^2'
    'deg/s^2'
    };

verificationTable = table( ...
    Metric, ...
    Baseline, ...
    Current, ...
    ErrorPercent, ...
    Unit);


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'actuator_interface_verification.csv');

writetable(verificationTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'actuator_interface_verification.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'ACTUATOR INTERFACE PASSIVE-MODE VERIFICATION\n');

fprintf(fid, ...
    '============================================\n\n');

fprintf(fid, ...
    'Peak heave error       = %.6f %%\n', ...
    heave_error_pct);

fprintf(fid, ...
    'Peak pitch error       = %.6f %%\n', ...
    pitch_error_pct);

fprintf(fid, ...
    'RMS heave accel error  = %.6f %%\n', ...
    heave_accel_error_pct);

fprintf(fid, ...
    'RMS pitch accel error  = %.6f %%\n\n', ...
    pitch_accel_error_pct);

if overall_pass
    fprintf(fid, 'Verification Result: PASS\n');
else
    fprintf(fid, 'Verification Result: FAIL\n');
end

fclose(fid);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'actuator_interface_verification.mat');

save( ...
    matFile, ...
    'verificationTable', ...
    'overall_pass', ...
    'tolerance_pct');


%% =========================================================
% Comparison Plot
% Normalized so unlike units can be compared meaningfully.
% ==========================================================

normalized_current = ...
    100 * Current ./ Baseline;

fig = figure( ...
    'Name', ...
    'Actuator Interface Verification', ...
    'NumberTitle', ...
    'off');

bar(categorical(Metric), normalized_current);

yline(100, '--', ...
    'Stored Passive Baseline');

grid on

ylabel('Baseline Match [%]')

title( ...
    'Passive-Mode Verification After Actuator Integration')

ylim([95 105])


%% =========================================================
% Save Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'actuator_interface_passive_verification.png');

figFile = fullfile( ...
    plotsFolder, ...
    'actuator_interface_passive_verification.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', 300);

savefig(fig, figFile);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Verification results saved.')

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', pngFile);
