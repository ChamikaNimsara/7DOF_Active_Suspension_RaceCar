%% Verify Optimized Skyhook Controller
% Confirms that the selected C_sky = 6000 N*s/m
% reproduces the optimization result.

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
% Load Optimization Reference
% ==========================================================

optFile = fullfile( ...
    resultsFolder, ...
    'skyhook_weighted_optimization.mat');

if ~isfile(optFile)
    error('skyhook_weighted_optimization.mat not found.');
end

opt = load(optFile);


%% =========================================================
% Current Simulation Metrics
% ==========================================================

current_peak_heave = ...
    1000 * max(abs(z_s_out.Data));

current_peak_pitch = ...
    max(abs(rad2deg(theta_out.Data)));

current_rms_heave_accel = ...
    rms(z_s_ddot_out.Data);

current_rms_pitch_accel = ...
    rms(rad2deg(theta_ddot_out.Data));


%% =========================================================
% Settling Time
% ==========================================================

z_s = z_s_out.Data;
t = z_s_out.Time;

threshold = ...
    0.02 * max(abs(z_s));

current_settling_time = NaN;

for i = 1:length(z_s)

    if t(i) >= t_bump_front

        remaining = abs(z_s(i:end));

        if all(remaining <= threshold)

            current_settling_time = t(i);
            break;

        end

    end

end


%% =========================================================
% Actuator Force Metrics
% ==========================================================

FL = F_act_FL_out.Data;
FR = F_act_FR_out.Data;
RL = F_act_RL_out.Data;
RR = F_act_RR_out.Data;

current_peak_force = max([ ...
    max(abs(FL)), ...
    max(abs(FR)), ...
    max(abs(RL)), ...
    max(abs(RR))]);


%% =========================================================
% Compare Against Optimization Result
% ==========================================================

error_heave = ...
    100 * abs( ...
    current_peak_heave - opt.optimal_peak_heave) ...
    / opt.optimal_peak_heave;

error_pitch = ...
    100 * abs( ...
    current_peak_pitch - opt.optimal_peak_pitch) ...
    / opt.optimal_peak_pitch;

error_heave_accel = ...
    100 * abs( ...
    current_rms_heave_accel - opt.optimal_rms_heave_accel) ...
    / opt.optimal_rms_heave_accel;

error_pitch_accel = ...
    100 * abs( ...
    current_rms_pitch_accel - opt.optimal_rms_pitch_accel) ...
    / opt.optimal_rms_pitch_accel;

error_settling = ...
    100 * abs( ...
    current_settling_time - opt.optimal_settling_time) ...
    / opt.optimal_settling_time;

error_force = ...
    100 * abs( ...
    current_peak_force - opt.optimal_peak_force) ...
    / opt.optimal_peak_force;


%% =========================================================
% Pass / Fail
% ==========================================================

tolerance_pct = 0.1;

overall_pass = ...
    error_heave <= tolerance_pct && ...
    error_pitch <= tolerance_pct && ...
    error_heave_accel <= tolerance_pct && ...
    error_pitch_accel <= tolerance_pct && ...
    error_settling <= tolerance_pct && ...
    error_force <= tolerance_pct;


%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('OPTIMIZED SKYHOOK VERIFICATION')
disp('====================================================')

fprintf('Current C_sky            = %.0f N*s/m\n\n', ...
    C_sky);

fprintf('Peak heave error         = %.6f %%\n', ...
    error_heave);

fprintf('Peak pitch error         = %.6f %%\n', ...
    error_pitch);

fprintf('RMS heave accel error    = %.6f %%\n', ...
    error_heave_accel);

fprintf('RMS pitch accel error    = %.6f %%\n', ...
    error_pitch_accel);

fprintf('Settling time error      = %.6f %%\n', ...
    error_settling);

fprintf('Peak actuator force error= %.6f %%\n\n', ...
    error_force);

if overall_pass
    disp('RESULT: PASS')
    disp('Optimized Skyhook controller verified.')
else
    disp('RESULT: FAIL')
    disp('Check model configuration or gain value.')
end

disp('====================================================')


%% =========================================================
% Save Verification Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'Settling Time'
    'Peak Actuator Force'
    };

Expected = [
    opt.optimal_peak_heave
    opt.optimal_peak_pitch
    opt.optimal_rms_heave_accel
    opt.optimal_rms_pitch_accel
    opt.optimal_settling_time
    opt.optimal_peak_force
    ];

Current = [
    current_peak_heave
    current_peak_pitch
    current_rms_heave_accel
    current_rms_pitch_accel
    current_settling_time
    current_peak_force
    ];

ErrorPercent = [
    error_heave
    error_pitch
    error_heave_accel
    error_pitch_accel
    error_settling
    error_force
    ];

verificationTable = table( ...
    Metric, ...
    Expected, ...
    Current, ...
    ErrorPercent);


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'optimized_skyhook_verification.csv');

writetable(verificationTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'optimized_skyhook_verification.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'OPTIMIZED SKYHOOK VERIFICATION\n');

fprintf(fid, ...
    '==============================\n\n');

fprintf(fid, ...
    'C_sky = %.0f N*s/m\n\n', ...
    C_sky);

for i = 1:height(verificationTable)

    fprintf(fid, ...
        '%s\n', ...
        verificationTable.Metric{i});

    fprintf(fid, ...
        'Expected = %.6f\n', ...
        verificationTable.Expected(i));

    fprintf(fid, ...
        'Current  = %.6f\n', ...
        verificationTable.Current(i));

    fprintf(fid, ...
        'Error    = %.6f %%\n\n', ...
        verificationTable.ErrorPercent(i));

end

if overall_pass
    fprintf(fid, 'RESULT: PASS\n');
else
    fprintf(fid, 'RESULT: FAIL\n');
end

fclose(fid);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'optimized_skyhook_verification.mat');

save(matFile, ...
    'verificationTable', ...
    'overall_pass', ...
    'C_sky');


%% =========================================================
% Verification Plot
% ==========================================================

normalizedCurrent = ...
    100 * Current ./ Expected;

fig = figure( ...
    'Name', ...
    'Optimized Skyhook Verification', ...
    'NumberTitle', ...
    'off');

bar(categorical(Metric), normalizedCurrent);

yline(100, '--', ...
    'Optimization Reference');

grid on

ylabel('Match to Optimized Result [%]')

title('Optimized Skyhook Verification')

ylim([95 105])


%% =========================================================
% Save Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'optimized_skyhook_verification.png');

figFile = fullfile( ...
    plotsFolder, ...
    'optimized_skyhook_verification.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', ...
    300);

savefig(fig, figFile);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Optimized Skyhook verification saved.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', pngFile);