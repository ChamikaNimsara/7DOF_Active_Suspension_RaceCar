%% Optimized Active Asymmetric Kerb Metrics

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
% Load Passive Kerb Baseline
% ==========================================================

passiveFile = fullfile( ...
    passiveResultsFolder, ...
    'passive_kerb_metrics.mat');

if ~isfile(passiveFile)
    error('passive_kerb_metrics.mat not found.');
end

passive = load(passiveFile);


%% =========================================================
% Extract Active Signals
% ==========================================================

t = z_s_out.Time;

z_s = z_s_out.Data;
z_s_ddot = z_s_ddot_out.Data;

theta_deg = rad2deg(theta_out.Data);
theta_ddot_deg = rad2deg(theta_ddot_out.Data);

phi_deg = rad2deg(phi_out.Data);
phi_ddot_deg = rad2deg(phi_ddot_out.Data);


%% =========================================================
% Active Vehicle Metrics
% ==========================================================

active_peak_heave_mm = ...
    1000 * max(abs(z_s));

active_peak_pitch_deg = ...
    max(abs(theta_deg));

active_peak_roll_deg = ...
    max(abs(phi_deg));

active_rms_heave_accel = ...
    rms(z_s_ddot);

active_rms_pitch_accel = ...
    rms(theta_ddot_deg);

active_rms_roll_accel = ...
    rms(phi_ddot_deg);

active_peak_roll_accel = ...
    max(abs(phi_ddot_deg));


%% =========================================================
% Roll Settling Time
% ==========================================================

roll_threshold = ...
    0.02 * max(abs(phi_deg));

active_roll_settling_time = NaN;

for i = 1:length(phi_deg)

    if t(i) >= t_kerb_front

        remaining = abs(phi_deg(i:end));

        if all(remaining <= roll_threshold)

            active_roll_settling_time = t(i);
            break;

        end

    end

end


%% =========================================================
% Actuator Metrics
% ==========================================================

FL = F_act_FL_out.Data;
FR = F_act_FR_out.Data;
RL = F_act_RL_out.Data;
RR = F_act_RR_out.Data;

active_peak_force = max([ ...
    max(abs(FL)), ...
    max(abs(FR)), ...
    max(abs(RL)), ...
    max(abs(RR))]);

active_mean_rms_force = mean([ ...
    rms(FL), ...
    rms(FR), ...
    rms(RL), ...
    rms(RR)]);

active_max_saturation = max([ ...
    100*mean(abs(FL) >= 0.999*F_act_max), ...
    100*mean(abs(FR) >= 0.999*F_act_max), ...
    100*mean(abs(RL) >= 0.999*F_act_max), ...
    100*mean(abs(RR) >= 0.999*F_act_max)]);


%% =========================================================
% Improvements vs Passive Kerb
% ==========================================================

heave_improvement = ...
    100 * ...
    (passive.peak_heave_mm - active_peak_heave_mm) ...
    / passive.peak_heave_mm;

pitch_improvement = ...
    100 * ...
    (passive.peak_pitch_deg - active_peak_pitch_deg) ...
    / passive.peak_pitch_deg;

roll_improvement = ...
    100 * ...
    (passive.peak_roll_deg - active_peak_roll_deg) ...
    / passive.peak_roll_deg;

heave_accel_improvement = ...
    100 * ...
    (passive.rms_heave_accel - active_rms_heave_accel) ...
    / passive.rms_heave_accel;

pitch_accel_improvement = ...
    100 * ...
    (passive.rms_pitch_accel - active_rms_pitch_accel) ...
    / passive.rms_pitch_accel;

roll_accel_improvement = ...
    100 * ...
    (passive.rms_roll_accel - active_rms_roll_accel) ...
    / passive.rms_roll_accel;

settling_improvement = ...
    100 * ...
    (passive.roll_settling_time - active_roll_settling_time) ...
    / passive.roll_settling_time;


%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('OPTIMIZED ACTIVE ASYMMETRIC KERB RESULTS')
disp('====================================================')

fprintf('Peak heave              = %.4f mm\n', ...
    active_peak_heave_mm);

fprintf('Peak pitch              = %.5f deg\n', ...
    active_peak_pitch_deg);

fprintf('Peak roll               = %.5f deg\n', ...
    active_peak_roll_deg);

fprintf('RMS heave acceleration  = %.4f m/s^2\n', ...
    active_rms_heave_accel);

fprintf('RMS pitch acceleration  = %.4f deg/s^2\n', ...
    active_rms_pitch_accel);

fprintf('RMS roll acceleration   = %.4f deg/s^2\n', ...
    active_rms_roll_accel);

fprintf('Peak roll acceleration  = %.4f deg/s^2\n', ...
    active_peak_roll_accel);

fprintf('Roll settling time      = %.3f s\n', ...
    active_roll_settling_time);

fprintf('\nACTUATOR EFFORT\n');

fprintf('Peak actuator force     = %.2f N\n', ...
    active_peak_force);

fprintf('Mean RMS actuator force = %.2f N\n', ...
    active_mean_rms_force);

fprintf('Maximum saturation      = %.3f %%\n', ...
    active_max_saturation);

fprintf('\nIMPROVEMENT VS PASSIVE KERB\n');

fprintf('Peak heave improvement  = %.2f %%\n', ...
    heave_improvement);

fprintf('Peak pitch improvement  = %.2f %%\n', ...
    pitch_improvement);

fprintf('Peak roll improvement   = %.2f %%\n', ...
    roll_improvement);

fprintf('Heave accel improvement = %.2f %%\n', ...
    heave_accel_improvement);

fprintf('Pitch accel improvement = %.2f %%\n', ...
    pitch_accel_improvement);

fprintf('Roll accel improvement  = %.2f %%\n', ...
    roll_accel_improvement);

fprintf('Roll settling improve   = %.2f %%\n', ...
    settling_improvement);

disp('====================================================')


%% =========================================================
% Save Comparison Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'Peak Roll'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'RMS Roll Acceleration'
    'Roll Settling Time'
    };

Passive = [
    passive.peak_heave_mm
    passive.peak_pitch_deg
    passive.peak_roll_deg
    passive.rms_heave_accel
    passive.rms_pitch_accel
    passive.rms_roll_accel
    passive.roll_settling_time
    ];

Active = [
    active_peak_heave_mm
    active_peak_pitch_deg
    active_peak_roll_deg
    active_rms_heave_accel
    active_rms_pitch_accel
    active_rms_roll_accel
    active_roll_settling_time
    ];

ImprovementPercent = [
    heave_improvement
    pitch_improvement
    roll_improvement
    heave_accel_improvement
    pitch_accel_improvement
    roll_accel_improvement
    settling_improvement
    ];

Unit = {
    'mm'
    'deg'
    'deg'
    'm/s^2'
    'deg/s^2'
    'deg/s^2'
    's'
    };

comparisonTable = table( ...
    Metric, ...
    Passive, ...
    Active, ...
    ImprovementPercent, ...
    Unit);


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_kerb_metrics.csv');

writetable(comparisonTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_kerb_metrics.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'OPTIMIZED ACTIVE vs PASSIVE KERB RESULTS\n');

fprintf(fid, ...
    '========================================\n\n');

for i = 1:height(comparisonTable)

    fprintf(fid, '%s\n', comparisonTable.Metric{i});

    fprintf(fid, ...
        'Passive = %.6f %s\n', ...
        comparisonTable.Passive(i), ...
        comparisonTable.Unit{i});

    fprintf(fid, ...
        'Active = %.6f %s\n', ...
        comparisonTable.Active(i), ...
        comparisonTable.Unit{i});

    fprintf(fid, ...
        'Improvement = %.2f %%\n\n', ...
        comparisonTable.ImprovementPercent(i));

end

fprintf(fid, ...
    'Peak actuator force = %.2f N\n', ...
    active_peak_force);

fprintf(fid, ...
    'Mean RMS actuator force = %.2f N\n', ...
    active_mean_rms_force);

fprintf(fid, ...
    'Maximum saturation = %.3f %%\n', ...
    active_max_saturation);

fclose(fid);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_kerb_metrics.mat');

save(matFile, ...
    'comparisonTable', ...
    'active_peak_heave_mm', ...
    'active_peak_pitch_deg', ...
    'active_peak_roll_deg', ...
    'active_rms_heave_accel', ...
    'active_rms_pitch_accel', ...
    'active_rms_roll_accel', ...
    'active_peak_roll_accel', ...
    'active_roll_settling_time', ...
    'active_peak_force', ...
    'active_mean_rms_force', ...
    'active_max_saturation', ...
    'heave_improvement', ...
    'pitch_improvement', ...
    'roll_improvement', ...
    'heave_accel_improvement', ...
    'pitch_accel_improvement', ...
    'roll_accel_improvement', ...
    'settling_improvement');


%% =========================================================
% Plot Active Body Motion
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'Optimized Active Kerb Body Motion', ...
    'NumberTitle', ...
    'off');

subplot(3,1,1)

plot(t, 1000*z_s, ...
    'LineWidth', 1.4);

grid on
xlabel('Time [s]')
ylabel('Heave [mm]')
title('Active Heave Response')


subplot(3,1,2)

plot(t, theta_deg, ...
    'LineWidth', 1.4);

grid on
xlabel('Time [s]')
ylabel('Pitch [deg]')
title('Active Pitch Response')


subplot(3,1,3)

plot(t, phi_deg, ...
    'LineWidth', 1.4);

grid on
xlabel('Time [s]')
ylabel('Roll [deg]')
title('Active Roll Response')

sgtitle('Optimized Skyhook Response to Left-Side Kerb');


motionPNG = fullfile( ...
    plotsFolder, ...
    'active_kerb_body_motion.png');

motionFIG = fullfile( ...
    plotsFolder, ...
    'active_kerb_body_motion.fig');

exportgraphics(fig1, motionPNG, ...
    'Resolution', 300);

savefig(fig1, motionFIG);


%% =========================================================
% Normalized Passive vs Active Comparison
% ==========================================================

normalizedPassive = ...
    100 * ones(size(Passive));

normalizedActive = ...
    100 * Active ./ Passive;

fig2 = figure( ...
    'Name', ...
    'Passive vs Active Kerb', ...
    'NumberTitle', ...
    'off');

bar(categorical(Metric), ...
    [normalizedPassive normalizedActive]);

grid on

ylabel('Metric [% of Passive]')

title('Passive vs Optimized Active Kerb Performance')

legend('Passive', 'Optimized Active', ...
    'Location', 'best');


comparisonPNG = fullfile( ...
    plotsFolder, ...
    'passive_vs_active_kerb_performance.png');

comparisonFIG = fullfile( ...
    plotsFolder, ...
    'passive_vs_active_kerb_performance.fig');

exportgraphics(fig2, ...
    comparisonPNG, ...
    'Resolution', 300);

savefig(fig2, comparisonFIG);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Active kerb results saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', motionPNG);
fprintf('%s\n', comparisonPNG);
