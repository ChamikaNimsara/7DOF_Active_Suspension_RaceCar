%% Active Bump Response Metrics
% Skyhook active suspension performance

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
% Load Passive Baseline
% ==========================================================

baselineFile = fullfile( ...
    passiveResultsFolder, ...
    'passive_baseline_summary.mat');

baseline = load(baselineFile);

%% =========================================================
% Active Response Metrics
% ==========================================================

z_s = z_s_out.Data;
theta_deg = rad2deg(theta_out.Data);
phi_deg = rad2deg(phi_out.Data);

z_s_ddot = z_s_ddot_out.Data;
theta_ddot_deg = rad2deg(theta_ddot_out.Data);

t = z_s_out.Time;

active_peak_heave_mm = ...
    1000 * max(abs(z_s));

active_peak_pitch_deg = ...
    max(abs(theta_deg));

active_rms_heave_accel = ...
    rms(z_s_ddot);

active_rms_pitch_accel = ...
    rms(theta_ddot_deg);

active_max_roll_deg = ...
    max(abs(phi_deg));

%% =========================================================
% Active Settling Time
% ==========================================================

peak_abs = max(abs(z_s));
threshold = 0.02 * peak_abs;

active_settling_time = NaN;

for i = 1:length(z_s)

    if t(i) >= t_bump_front

        remaining = abs(z_s(i:end));

        if all(remaining <= threshold)

            active_settling_time = t(i);
            break;

        end

    end

end

%% =========================================================
% Improvements Relative to Passive Baseline
% ==========================================================

heave_improvement_pct = ...
    100 * (baseline.peak_heave_mm - active_peak_heave_mm) ...
    / baseline.peak_heave_mm;

pitch_improvement_pct = ...
    100 * (baseline.peak_pitch_deg - active_peak_pitch_deg) ...
    / baseline.peak_pitch_deg;

heave_accel_improvement_pct = ...
    100 * (baseline.rms_heave_accel - active_rms_heave_accel) ...
    / baseline.rms_heave_accel;

pitch_accel_improvement_pct = ...
    100 * (baseline.rms_pitch_accel - active_rms_pitch_accel) ...
    / baseline.rms_pitch_accel;

settling_improvement_pct = ...
    100 * (baseline.settling_time - active_settling_time) ...
    / baseline.settling_time;

%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('ACTIVE SKYHOOK BUMP PERFORMANCE')
disp('====================================================')

fprintf('Peak heave              = %.3f mm\n', ...
    active_peak_heave_mm);

fprintf('Peak pitch              = %.4f deg\n', ...
    active_peak_pitch_deg);

fprintf('RMS heave acceleration  = %.3f m/s^2\n', ...
    active_rms_heave_accel);

fprintf('RMS pitch acceleration  = %.3f deg/s^2\n', ...
    active_rms_pitch_accel);

fprintf('Settling time           = %.3f s\n', ...
    active_settling_time);

fprintf('Maximum roll            = %.8f deg\n', ...
    active_max_roll_deg);

fprintf('\nIMPROVEMENT VS PASSIVE\n');

fprintf('Peak heave improvement  = %.2f %%\n', ...
    heave_improvement_pct);

fprintf('Peak pitch improvement  = %.2f %%\n', ...
    pitch_improvement_pct);

fprintf('RMS heave accel improve = %.2f %%\n', ...
    heave_accel_improvement_pct);

fprintf('RMS pitch accel improve = %.2f %%\n', ...
    pitch_accel_improvement_pct);

fprintf('Settling improvement    = %.2f %%\n', ...
    settling_improvement_pct);

disp('====================================================')

%% =========================================================
% Save Results Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'Settling Time'
    };

Passive = [
    baseline.peak_heave_mm
    baseline.peak_pitch_deg
    baseline.rms_heave_accel
    baseline.rms_pitch_accel
    baseline.settling_time
    ];

Active = [
    active_peak_heave_mm
    active_peak_pitch_deg
    active_rms_heave_accel
    active_rms_pitch_accel
    active_settling_time
    ];

ImprovementPercent = [
    heave_improvement_pct
    pitch_improvement_pct
    heave_accel_improvement_pct
    pitch_accel_improvement_pct
    settling_improvement_pct
    ];

Unit = {
    'mm'
    'deg'
    'm/s^2'
    'deg/s^2'
    's'
    };

comparisonTable = table( ...
    Metric, ...
    Passive, ...
    Active, ...
    ImprovementPercent, ...
    Unit);

csvFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_bump_metrics.csv');

writetable(comparisonTable, csvFile);

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_bump_metrics.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, 'ACTIVE SKYHOOK VS PASSIVE BUMP PERFORMANCE\n');
fprintf(fid, '==========================================\n\n');

for i = 1:height(comparisonTable)

    fprintf(fid, ...
        '%s\nPassive = %.6f %s\nActive  = %.6f %s\nImprovement = %.2f %%\n\n', ...
        comparisonTable.Metric{i}, ...
        comparisonTable.Passive(i), ...
        comparisonTable.Unit{i}, ...
        comparisonTable.Active(i), ...
        comparisonTable.Unit{i}, ...
        comparisonTable.ImprovementPercent(i));

end

fclose(fid);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'active_vs_passive_bump_metrics.mat');

save(matFile, ...
    'comparisonTable', ...
    'active_peak_heave_mm', ...
    'active_peak_pitch_deg', ...
    'active_rms_heave_accel', ...
    'active_rms_pitch_accel', ...
    'active_settling_time', ...
    'heave_improvement_pct', ...
    'pitch_improvement_pct', ...
    'heave_accel_improvement_pct', ...
    'pitch_accel_improvement_pct', ...
    'settling_improvement_pct');

%% =========================================================
% Comparison Plot
% ==========================================================

normalizedPassive = ...
    100 * ones(size(Passive));

normalizedActive = ...
    100 * Active ./ Passive;

fig = figure( ...
    'Name', ...
    'Passive vs Active Bump Performance', ...
    'NumberTitle', ...
    'off');

bar(categorical(Metric), ...
    [normalizedPassive normalizedActive]);

grid on

ylabel('Relative Metric [% of Passive]')

title('Passive vs Active Skyhook Suspension Performance')

legend( ...
    'Passive', ...
    'Active', ...
    'Location', ...
    'best');

%% =========================================================
% Save Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'passive_vs_active_bump_performance.png');

figFile = fullfile( ...
    plotsFolder, ...
    'passive_vs_active_bump_performance.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', ...
    300);

savefig(fig, figFile);

disp(' ')
disp('Active comparison results saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlot:\n');
fprintf('%s\n', pngFile);
