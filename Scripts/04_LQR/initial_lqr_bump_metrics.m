%% Initial LQR Symmetric-Bump Metrics

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
% Load Passive Baseline
% ==========================================================

passiveFile = fullfile( ...
    passiveResultsFolder, ...
    'passive_baseline_summary.mat');

passive = load(passiveFile);

%% =========================================================
% Load Optimized Skyhook
% ==========================================================

skyhookFile = fullfile( ...
    skyhookResultsFolder, ...
    'skyhook_weighted_optimization.mat');

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
% Vehicle Metrics
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

%% =========================================================
% Heave Settling Time
% ==========================================================

threshold = ...
    0.02 * max(abs(z_s));

lqr_settling_time = NaN;

for i = 1:length(z_s)

    if t(i) >= t_bump_front

        if all(abs(z_s(i:end)) <= threshold)

            lqr_settling_time = t(i);
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

heaveAccelImprovement = ...
    100 * ...
    (passive.rms_heave_accel - lqr_rms_heave_accel) ...
    / passive.rms_heave_accel;

pitchAccelImprovement = ...
    100 * ...
    (passive.rms_pitch_accel - lqr_rms_pitch_accel) ...
    / passive.rms_pitch_accel;

settlingImprovement = ...
    100 * ...
    (passive.settling_time - lqr_settling_time) ...
    / passive.settling_time;

%% =========================================================
% Display
% ==========================================================

disp('====================================================')
disp('INITIAL LQR SYMMETRIC-BUMP RESULTS')
disp('====================================================')

fprintf('Peak heave              = %.4f mm\n', ...
    lqr_peak_heave_mm);

fprintf('Peak pitch              = %.5f deg\n', ...
    lqr_peak_pitch_deg);

fprintf('Maximum roll            = %.10f deg\n', ...
    lqr_peak_roll_deg);

fprintf('RMS heave acceleration  = %.5f m/s^2\n', ...
    lqr_rms_heave_accel);

fprintf('RMS pitch acceleration  = %.5f deg/s^2\n', ...
    lqr_rms_pitch_accel);

fprintf('Settling time           = %.3f s\n\n', ...
    lqr_settling_time);

fprintf('Peak actuator force     = %.2f N\n', ...
    lqr_peak_force);

fprintf('Mean RMS actuator force = %.2f N\n', ...
    lqr_mean_rms_force);

fprintf('Maximum saturation      = %.3f %%\n\n', ...
    lqr_max_saturation);

fprintf('IMPROVEMENT VS PASSIVE\n');

fprintf('Peak heave              = %.2f %%\n', ...
    heaveImprovement);

fprintf('Peak pitch              = %.2f %%\n', ...
    pitchImprovement);

fprintf('RMS heave acceleration  = %.2f %%\n', ...
    heaveAccelImprovement);

fprintf('RMS pitch acceleration  = %.2f %%\n', ...
    pitchAccelImprovement);

fprintf('Settling time           = %.2f %%\n', ...
    settlingImprovement);

disp('====================================================')

%% =========================================================
% Comparison Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'Settling Time'
    'Peak Actuator Force'
    };

Passive = [
    passive.peak_heave_mm
    passive.peak_pitch_deg
    passive.rms_heave_accel
    passive.rms_pitch_accel
    passive.settling_time
    0
    ];

Skyhook = [
    skyhook.optimal_peak_heave
    skyhook.optimal_peak_pitch
    skyhook.optimal_rms_heave_accel
    skyhook.optimal_rms_pitch_accel
    skyhook.optimal_settling_time
    skyhook.optimal_peak_force
    ];

InitialLQR = [
    lqr_peak_heave_mm
    lqr_peak_pitch_deg
    lqr_rms_heave_accel
    lqr_rms_pitch_accel
    lqr_settling_time
    lqr_peak_force
    ];

Unit = {
    'mm'
    'deg'
    'm/s^2'
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
    'initial_lqr_bump_metrics.csv');

writetable(comparisonTable, csvFile);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_bump_metrics.mat');

save(matFile, ...
    'comparisonTable', ...
    'lqr_peak_heave_mm', ...
    'lqr_peak_pitch_deg', ...
    'lqr_peak_roll_deg', ...
    'lqr_rms_heave_accel', ...
    'lqr_rms_pitch_accel', ...
    'lqr_settling_time', ...
    'lqr_peak_force', ...
    'lqr_mean_rms_force', ...
    'lqr_max_saturation', ...
    'heaveImprovement', ...
    'pitchImprovement', ...
    'heaveAccelImprovement', ...
    'pitchAccelImprovement', ...
    'settlingImprovement');

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_bump_metrics.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'INITIAL LQR SYMMETRIC-BUMP RESULTS\n');

fprintf(fid, ...
    '==================================\n\n');

fprintf(fid, ...
    'Peak heave = %.6f mm\n', ...
    lqr_peak_heave_mm);

fprintf(fid, ...
    'Peak pitch = %.6f deg\n', ...
    lqr_peak_pitch_deg);

fprintf(fid, ...
    'Maximum roll = %.12f deg\n', ...
    lqr_peak_roll_deg);

fprintf(fid, ...
    'RMS heave acceleration = %.6f m/s^2\n', ...
    lqr_rms_heave_accel);

fprintf(fid, ...
    'RMS pitch acceleration = %.6f deg/s^2\n', ...
    lqr_rms_pitch_accel);

fprintf(fid, ...
    'Settling time = %.6f s\n', ...
    lqr_settling_time);

fprintf(fid, ...
    'Peak actuator force = %.6f N\n', ...
    lqr_peak_force);

fprintf(fid, ...
    'Mean RMS actuator force = %.6f N\n', ...
    lqr_mean_rms_force);

fprintf(fid, ...
    'Maximum saturation = %.6f %%\n', ...
    lqr_max_saturation);

fclose(fid);

%% =========================================================
% Plot Actuator Forces with Correct Title
% ==========================================================

fig = figure( ...
    'Name', ...
    'Initial LQR Actuator Forces', ...
    'NumberTitle', ...
    'off');

plot(F_act_FL_out.Time, FL, ...
    'LineWidth', 1.2);

hold on

plot(F_act_FR_out.Time, FR, ...
    'LineWidth', 1.2);

plot(F_act_RL_out.Time, RL, ...
    'LineWidth', 1.2);

plot(F_act_RR_out.Time, RR, ...
    'LineWidth', 1.2);

yline(F_act_max, '--', ...
    'HandleVisibility', 'off');

yline(-F_act_max, '--', ...
    'HandleVisibility', 'off');

grid on

xlabel('Time [s]')
ylabel('Actuator Force [N]')

title('Initial LQR Actuator Forces')

legend( ...
    'FL', ...
    'FR', ...
    'RL', ...
    'RR', ...
    'Location', ...
    'best');

forcePNG = fullfile( ...
    plotsFolder, ...
    'initial_lqr_actuator_forces.png');

forceFIG = fullfile( ...
    plotsFolder, ...
    'initial_lqr_actuator_forces.fig');

exportgraphics(fig, ...
    forcePNG, ...
    'Resolution', 300);

savefig(fig, forceFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Initial LQR bump metrics saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', forcePNG);
