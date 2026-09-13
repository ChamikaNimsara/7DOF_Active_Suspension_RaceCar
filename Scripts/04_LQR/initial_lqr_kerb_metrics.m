%% Initial LQR Asymmetric Kerb Metrics

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
% Load Passive Kerb Baseline
% ==========================================================

passiveFile = fullfile( ...
    passiveResultsFolder, ...
    'passive_kerb_metrics.mat');

passive = load(passiveFile);

%% =========================================================
% Load Optimized Skyhook Kerb Results
% ==========================================================

skyhookFile = fullfile( ...
    skyhookResultsFolder, ...
    'active_vs_passive_kerb_metrics.mat');

skyhook = load(skyhookFile);

%% =========================================================
% Extract Current LQR Signals
% ==========================================================

t = z_s_out.Time;

z_s = z_s_out.Data;
theta = rad2deg(theta_out.Data);
phi = rad2deg(phi_out.Data);

z_s_ddot = z_s_ddot_out.Data;
theta_ddot = rad2deg(theta_ddot_out.Data);
phi_ddot = rad2deg(phi_ddot_out.Data);

FL = F_act_FL_out.Data;
FR = F_act_FR_out.Data;
RL = F_act_RL_out.Data;
RR = F_act_RR_out.Data;

%% =========================================================
% LQR Kerb Metrics
% ==========================================================

lqr_peak_heave_mm = ...
    1000 * max(abs(z_s));

lqr_peak_pitch_deg = ...
    max(abs(theta));

lqr_peak_roll_deg = ...
    max(abs(phi));

lqr_rms_heave_accel = ...
    rms(z_s_ddot);

lqr_rms_pitch_accel = ...
    rms(theta_ddot);

lqr_rms_roll_accel = ...
    rms(phi_ddot);

lqr_peak_roll_accel = ...
    max(abs(phi_ddot));

%% =========================================================
% Roll Settling Time
% ==========================================================

threshold = ...
    0.02 * max(abs(phi));

lqr_roll_settling_time = NaN;

for i = 1:length(phi)

    if t(i) >= t_kerb_front

        if all(abs(phi(i:end)) <= threshold)

            lqr_roll_settling_time = t(i);
            break;

        end

    end

end

%% =========================================================
% Actuator Metrics
% ==========================================================

lqr_peak_force = max([ ...
    max(abs(FL)), ...
    max(abs(FR)), ...
    max(abs(RL)), ...
    max(abs(RR))]);

lqr_mean_rms_force = mean([ ...
    rms(FL), ...
    rms(FR), ...
    rms(RL), ...
    rms(RR)]);

lqr_max_saturation = max([ ...
    100*mean(abs(FL) >= 0.999*F_act_max), ...
    100*mean(abs(FR) >= 0.999*F_act_max), ...
    100*mean(abs(RL) >= 0.999*F_act_max), ...
    100*mean(abs(RR) >= 0.999*F_act_max)]);

%% =========================================================
% Improvements vs Passive
% ==========================================================

heaveImprovement = ...
    100 * ...
    (passive.peak_heave_mm - lqr_peak_heave_mm) ...
    / passive.peak_heave_mm;

pitchImprovement = ...
    100 * ...
    (passive.peak_pitch_deg - lqr_peak_pitch_deg) ...
    / passive.peak_pitch_deg;

rollImprovement = ...
    100 * ...
    (passive.peak_roll_deg - lqr_peak_roll_deg) ...
    / passive.peak_roll_deg;

heaveAccelImprovement = ...
    100 * ...
    (passive.rms_heave_accel - lqr_rms_heave_accel) ...
    / passive.rms_heave_accel;

pitchAccelImprovement = ...
    100 * ...
    (passive.rms_pitch_accel - lqr_rms_pitch_accel) ...
    / passive.rms_pitch_accel;

rollAccelImprovement = ...
    100 * ...
    (passive.rms_roll_accel - lqr_rms_roll_accel) ...
    / passive.rms_roll_accel;

settlingImprovement = ...
    100 * ...
    (passive.roll_settling_time - lqr_roll_settling_time) ...
    / passive.roll_settling_time;

%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('INITIAL LQR ASYMMETRIC KERB RESULTS')
disp('====================================================')

fprintf('Peak heave              = %.4f mm\n', ...
    lqr_peak_heave_mm);

fprintf('Peak pitch              = %.5f deg\n', ...
    lqr_peak_pitch_deg);

fprintf('Peak roll               = %.5f deg\n', ...
    lqr_peak_roll_deg);

fprintf('RMS heave acceleration  = %.4f m/s^2\n', ...
    lqr_rms_heave_accel);

fprintf('RMS pitch acceleration  = %.4f deg/s^2\n', ...
    lqr_rms_pitch_accel);

fprintf('RMS roll acceleration   = %.4f deg/s^2\n', ...
    lqr_rms_roll_accel);

fprintf('Peak roll acceleration  = %.4f deg/s^2\n', ...
    lqr_peak_roll_accel);

fprintf('Roll settling time      = %.3f s\n\n', ...
    lqr_roll_settling_time);

fprintf('ACTUATOR EFFORT\n');

fprintf('Peak actuator force     = %.2f N\n', ...
    lqr_peak_force);

fprintf('Mean RMS actuator force = %.2f N\n', ...
    lqr_mean_rms_force);

fprintf('Maximum saturation      = %.3f %%\n\n', ...
    lqr_max_saturation);

fprintf('IMPROVEMENT VS PASSIVE KERB\n');

fprintf('Peak heave              = %.2f %%\n', ...
    heaveImprovement);

fprintf('Peak pitch              = %.2f %%\n', ...
    pitchImprovement);

fprintf('Peak roll               = %.2f %%\n', ...
    rollImprovement);

fprintf('RMS heave acceleration  = %.2f %%\n', ...
    heaveAccelImprovement);

fprintf('RMS pitch acceleration  = %.2f %%\n', ...
    pitchAccelImprovement);

fprintf('RMS roll acceleration   = %.2f %%\n', ...
    rollAccelImprovement);

fprintf('Roll settling time      = %.2f %%\n', ...
    settlingImprovement);

disp('====================================================')

%% =========================================================
% Comparison Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'Peak Roll'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'RMS Roll Acceleration'
    'Roll Settling Time'
    'Peak Actuator Force'
    };

Passive = [
    passive.peak_heave_mm
    passive.peak_pitch_deg
    passive.peak_roll_deg
    passive.rms_heave_accel
    passive.rms_pitch_accel
    passive.rms_roll_accel
    passive.roll_settling_time
    0
    ];

Skyhook = [
    skyhook.active_peak_heave_mm
    skyhook.active_peak_pitch_deg
    skyhook.active_peak_roll_deg
    skyhook.active_rms_heave_accel
    skyhook.active_rms_pitch_accel
    skyhook.active_rms_roll_accel
    skyhook.active_roll_settling_time
    skyhook.active_peak_force
    ];

InitialLQR = [
    lqr_peak_heave_mm
    lqr_peak_pitch_deg
    lqr_peak_roll_deg
    lqr_rms_heave_accel
    lqr_rms_pitch_accel
    lqr_rms_roll_accel
    lqr_roll_settling_time
    lqr_peak_force
    ];

Unit = {
    'mm'
    'deg'
    'deg'
    'm/s^2'
    'deg/s^2'
    'deg/s^2'
    's'
    'N'
    };

comparisonTable = table( ...
    Metric, ...
    Passive, ...
    Skyhook, ...
    InitialLQR, ...
    Unit);

%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_kerb_metrics.csv');

writetable(comparisonTable, csvFile);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_kerb_metrics.mat');

save(matFile, ...
    'comparisonTable', ...
    'lqr_peak_heave_mm', ...
    'lqr_peak_pitch_deg', ...
    'lqr_peak_roll_deg', ...
    'lqr_rms_heave_accel', ...
    'lqr_rms_pitch_accel', ...
    'lqr_rms_roll_accel', ...
    'lqr_peak_roll_accel', ...
    'lqr_roll_settling_time', ...
    'lqr_peak_force', ...
    'lqr_mean_rms_force', ...
    'lqr_max_saturation', ...
    'heaveImprovement', ...
    'pitchImprovement', ...
    'rollImprovement', ...
    'heaveAccelImprovement', ...
    'pitchAccelImprovement', ...
    'rollAccelImprovement', ...
    'settlingImprovement');

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_kerb_metrics.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'INITIAL LQR ASYMMETRIC KERB RESULTS\n');

fprintf(fid, ...
    '===================================\n\n');

for i = 1:height(comparisonTable)

    fprintf(fid, '%s\n', comparisonTable.Metric{i});

    fprintf(fid, ...
        'Passive = %.6f %s\n', ...
        comparisonTable.Passive(i), ...
        comparisonTable.Unit{i});

    fprintf(fid, ...
        'Skyhook = %.6f %s\n', ...
        comparisonTable.Skyhook(i), ...
        comparisonTable.Unit{i});

    fprintf(fid, ...
        'Initial LQR = %.6f %s\n\n', ...
        comparisonTable.InitialLQR(i), ...
        comparisonTable.Unit{i});

end

fclose(fid);

%% =========================================================
% Normalized Comparison Plot
% ==========================================================

normalizedPassive = ...
    100 * ones(7,1);

normalizedSkyhook = ...
    100 * Skyhook(1:7) ./ Passive(1:7);

normalizedLQR = ...
    100 * InitialLQR(1:7) ./ Passive(1:7);

fig1 = figure( ...
    'Name', ...
    'Kerb Controller Comparison', ...
    'NumberTitle', ...
    'off');

bar(categorical(Metric(1:7)), ...
    [normalizedPassive ...
     normalizedSkyhook ...
     normalizedLQR]);

grid on

ylabel('Metric [% of Passive]')

title('Passive vs Skyhook vs Initial LQR - Asymmetric Kerb')

legend( ...
    'Passive', ...
    'Optimized Skyhook', ...
    'Initial LQR', ...
    'Location', ...
    'best');

performancePNG = fullfile( ...
    plotsFolder, ...
    'passive_skyhook_initial_lqr_kerb_performance.png');

performanceFIG = fullfile( ...
    plotsFolder, ...
    'passive_skyhook_initial_lqr_kerb_performance.fig');

exportgraphics( ...
    fig1, ...
    performancePNG, ...
    'Resolution', 300);

savefig(fig1, performanceFIG);

%% =========================================================
% Actuator Effort Plot
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Kerb Actuator Effort', ...
    'NumberTitle', ...
    'off');

bar(categorical({ ...
    'Passive', ...
    'Optimized Skyhook', ...
    'Initial LQR'}), ...
    [0 ...
     skyhook.active_peak_force ...
     lqr_peak_force]);

hold on

yline(F_act_max, '--', ...
    'Actuator Limit');

grid on

ylabel('Peak Actuator Force [N]')

title('Kerb Test Peak Actuator Effort');

forcePNG = fullfile( ...
    plotsFolder, ...
    'skyhook_vs_lqr_kerb_actuator_effort.png');

forceFIG = fullfile( ...
    plotsFolder, ...
    'skyhook_vs_lqr_kerb_actuator_effort.fig');

exportgraphics( ...
    fig2, ...
    forcePNG, ...
    'Resolution', 300);

savefig(fig2, forceFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Initial LQR kerb metrics saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', performancePNG);
fprintf('%s\n', forcePNG);
