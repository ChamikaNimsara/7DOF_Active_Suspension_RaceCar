%% Passive Asymmetric Kerb Metrics

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

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end


%% =========================================================
% Extract Signals
% ==========================================================

t = z_s_out.Time;

z_s = z_s_out.Data;
z_s_ddot = z_s_ddot_out.Data;

theta_deg = rad2deg(theta_out.Data);
theta_ddot_deg = rad2deg(theta_ddot_out.Data);

phi_deg = rad2deg(phi_out.Data);
phi_ddot_deg = rad2deg(phi_ddot_out.Data);


%% =========================================================
% Calculate Metrics
% ==========================================================

peak_heave_mm = ...
    1000 * max(abs(z_s));

peak_pitch_deg = ...
    max(abs(theta_deg));

peak_roll_deg = ...
    max(abs(phi_deg));

rms_heave_accel = ...
    rms(z_s_ddot);

rms_pitch_accel = ...
    rms(theta_ddot_deg);

rms_roll_accel = ...
    rms(phi_ddot_deg);

peak_roll_accel = ...
    max(abs(phi_ddot_deg));


%% =========================================================
% Roll Settling Time
% ==========================================================

roll_threshold = ...
    0.02 * max(abs(phi_deg));

roll_settling_time = NaN;

for i = 1:length(phi_deg)

    if t(i) >= t_kerb_front

        remaining = abs(phi_deg(i:end));

        if all(remaining <= roll_threshold)

            roll_settling_time = t(i);
            break;

        end

    end

end


%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('PASSIVE ASYMMETRIC KERB RESULTS')
disp('====================================================')

fprintf('Peak heave              = %.4f mm\n', ...
    peak_heave_mm);

fprintf('Peak pitch              = %.5f deg\n', ...
    peak_pitch_deg);

fprintf('Peak roll               = %.5f deg\n', ...
    peak_roll_deg);

fprintf('RMS heave acceleration  = %.4f m/s^2\n', ...
    rms_heave_accel);

fprintf('RMS pitch acceleration  = %.4f deg/s^2\n', ...
    rms_pitch_accel);

fprintf('RMS roll acceleration   = %.4f deg/s^2\n', ...
    rms_roll_accel);

fprintf('Peak roll acceleration  = %.4f deg/s^2\n', ...
    peak_roll_accel);

fprintf('Roll settling time      = %.3f s\n', ...
    roll_settling_time);

disp('====================================================')


%% =========================================================
% Save Results Table
% ==========================================================

Metric = {
    'Peak Heave'
    'Peak Pitch'
    'Peak Roll'
    'RMS Heave Acceleration'
    'RMS Pitch Acceleration'
    'RMS Roll Acceleration'
    'Peak Roll Acceleration'
    'Roll Settling Time'
    };

Value = [
    peak_heave_mm
    peak_pitch_deg
    peak_roll_deg
    rms_heave_accel
    rms_pitch_accel
    rms_roll_accel
    peak_roll_accel
    roll_settling_time
    ];

Unit = {
    'mm'
    'deg'
    'deg'
    'm/s^2'
    'deg/s^2'
    'deg/s^2'
    'deg/s^2'
    's'
    };

resultsTable = table(Metric, Value, Unit);


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'passive_kerb_metrics.csv');

writetable(resultsTable, csvFile);


%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'passive_kerb_metrics.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'PASSIVE ASYMMETRIC KERB RESULTS\n');

fprintf(fid, ...
    '===============================\n\n');

for i = 1:height(resultsTable)

    fprintf(fid, ...
        '%s = %.6f %s\n', ...
        resultsTable.Metric{i}, ...
        resultsTable.Value(i), ...
        resultsTable.Unit{i});

end

fclose(fid);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'passive_kerb_metrics.mat');

save(matFile, ...
    'peak_heave_mm', ...
    'peak_pitch_deg', ...
    'peak_roll_deg', ...
    'rms_heave_accel', ...
    'rms_pitch_accel', ...
    'rms_roll_accel', ...
    'peak_roll_accel', ...
    'roll_settling_time', ...
    'resultsTable');


%% =========================================================
% Plot Body Motion
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'Passive Kerb Body Motion', ...
    'NumberTitle', ...
    'off');


subplot(3,1,1)

plot(t, 1000*z_s, ...
    'LineWidth', 1.4);

grid on

xlabel('Time [s]')
ylabel('Heave [mm]')

title('Passive Heave Response')


subplot(3,1,2)

plot(t, theta_deg, ...
    'LineWidth', 1.4);

grid on

xlabel('Time [s]')
ylabel('Pitch [deg]')

title('Passive Pitch Response')


subplot(3,1,3)

plot(t, phi_deg, ...
    'LineWidth', 1.4);

grid on

xlabel('Time [s]')
ylabel('Roll [deg]')

title('Passive Roll Response')


sgtitle('Passive 7-DOF Response to Left-Side Kerb');


%% =========================================================
% Save Body Motion Plot
% ==========================================================

motionPNG = fullfile( ...
    plotsFolder, ...
    'passive_kerb_body_motion.png');

motionFIG = fullfile( ...
    plotsFolder, ...
    'passive_kerb_body_motion.fig');

exportgraphics(fig1, ...
    motionPNG, ...
    'Resolution', 300);

savefig(fig1, motionFIG);


%% =========================================================
% Plot Body Accelerations
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Passive Kerb Accelerations', ...
    'NumberTitle', ...
    'off');


subplot(3,1,1)

plot(t, z_s_ddot, ...
    'LineWidth', 1.4);

grid on

xlabel('Time [s]')
ylabel('Heave Accel [m/s^2]')

title('Passive Heave Acceleration')


subplot(3,1,2)

plot(t, theta_ddot_deg, ...
    'LineWidth', 1.4);

grid on

xlabel('Time [s]')
ylabel('Pitch Accel [deg/s^2]')

title('Passive Pitch Acceleration')


subplot(3,1,3)

plot(t, phi_ddot_deg, ...
    'LineWidth', 1.4);

grid on

xlabel('Time [s]')
ylabel('Roll Accel [deg/s^2]')

title('Passive Roll Acceleration')


sgtitle('Passive Kerb Acceleration Response');


%% =========================================================
% Save Acceleration Plot
% ==========================================================

accelPNG = fullfile( ...
    plotsFolder, ...
    'passive_kerb_acceleration.png');

accelFIG = fullfile( ...
    plotsFolder, ...
    'passive_kerb_acceleration.fig');

exportgraphics(fig2, ...
    accelPNG, ...
    'Resolution', 300);

savefig(fig2, accelFIG);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Passive kerb results saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', motionPNG);
fprintf('%s\n', accelPNG);