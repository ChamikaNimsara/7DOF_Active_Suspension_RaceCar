%% Passive Bump Response Metrics
% Quantitative validation for passive 7-DOF model

clc;

%% =========================================================
% Locate Results Folder
% ==========================================================

scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
projectFolder = fileparts(fileparts(scriptFolder));
resultsFolder = fullfile(projectFolder, 'Results', 'Passive');

fprintf('Saving results to:\n%s\n\n', resultsFolder);


%% =========================================================
% Heave Displacement
% ==========================================================

z_s = z_s_out.Data;
t_z = z_s_out.Time;

peak_heave_pos = max(z_s);
peak_heave_neg = min(z_s);

[~, idx_heave_pos] = max(z_s);
[~, idx_heave_neg] = min(z_s);

time_peak_heave_pos = t_z(idx_heave_pos);
time_peak_heave_neg = t_z(idx_heave_neg);


%% =========================================================
% Heave Acceleration
% ==========================================================

z_s_ddot = z_s_ddot_out.Data;

peak_heave_accel_pos = max(z_s_ddot);
peak_heave_accel_neg = min(z_s_ddot);

rms_heave_accel = rms(z_s_ddot);


%% =========================================================
% Pitch Angle
% ==========================================================

theta_deg = rad2deg(theta_out.Data);
t_theta = theta_out.Time;

peak_pitch_pos = max(theta_deg);
peak_pitch_neg = min(theta_deg);

[~, idx_pitch_pos] = max(theta_deg);
[~, idx_pitch_neg] = min(theta_deg);

time_peak_pitch_pos = t_theta(idx_pitch_pos);
time_peak_pitch_neg = t_theta(idx_pitch_neg);


%% =========================================================
% Pitch Acceleration
% ==========================================================

theta_ddot_deg = rad2deg(theta_ddot_out.Data);

peak_pitch_accel_pos = max(theta_ddot_deg);
peak_pitch_accel_neg = min(theta_ddot_deg);

rms_pitch_accel = rms(theta_ddot_deg);


%% =========================================================
% Roll Validation
% ==========================================================

phi_deg = rad2deg(phi_out.Data);
phi_ddot_deg = rad2deg(phi_ddot_out.Data);

max_abs_roll = max(abs(phi_deg));
max_abs_roll_accel = max(abs(phi_ddot_deg));


%% =========================================================
% Settling Time Estimate for Heave
% ==========================================================

% Threshold = 2% of maximum absolute heave response

heave_peak_abs = max(abs(z_s));
heave_threshold = 0.02 * heave_peak_abs;

settling_time_heave = NaN;

for i = 1:length(z_s)

    if t_z(i) >= t_bump_front

        remaining_signal = abs(z_s(i:end));

        if all(remaining_signal <= heave_threshold)

            settling_time_heave = t_z(i);
            break;

        end

    end

end


%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('PASSIVE 7-DOF SYMMETRIC BUMP METRICS')
disp('====================================================')

fprintf('\nHEAVE DISPLACEMENT\n')
fprintf('Peak positive heave       = %.6f m\n', peak_heave_pos);
fprintf('Peak negative heave       = %.6f m\n', peak_heave_neg);
fprintf('Time of positive peak     = %.3f s\n', time_peak_heave_pos);
fprintf('Time of negative peak     = %.3f s\n', time_peak_heave_neg);

fprintf('\nHEAVE ACCELERATION\n')
fprintf('Peak positive accel       = %.3f m/s^2\n', peak_heave_accel_pos);
fprintf('Peak negative accel       = %.3f m/s^2\n', peak_heave_accel_neg);
fprintf('RMS heave acceleration    = %.3f m/s^2\n', rms_heave_accel);

fprintf('\nPITCH ANGLE\n')
fprintf('Peak positive pitch       = %.4f deg\n', peak_pitch_pos);
fprintf('Peak negative pitch       = %.4f deg\n', peak_pitch_neg);
fprintf('Time of positive peak     = %.3f s\n', time_peak_pitch_pos);
fprintf('Time of negative peak     = %.3f s\n', time_peak_pitch_neg);

fprintf('\nPITCH ACCELERATION\n')
fprintf('Peak positive pitch acc   = %.3f deg/s^2\n', peak_pitch_accel_pos);
fprintf('Peak negative pitch acc   = %.3f deg/s^2\n', peak_pitch_accel_neg);
fprintf('RMS pitch acceleration    = %.3f deg/s^2\n', rms_pitch_accel);

fprintf('\nROLL VALIDATION\n')
fprintf('Maximum absolute roll     = %.8f deg\n', max_abs_roll);
fprintf('Maximum roll acceleration = %.8f deg/s^2\n', max_abs_roll_accel);

fprintf('\nSETTLING\n')

if isnan(settling_time_heave)
    fprintf('Heave settling time       = Not reached\n');
else
    fprintf('Heave settling time       = %.3f s\n', settling_time_heave);
end

disp('====================================================')


%% =========================================================
% Save Human-Readable TXT File
% ==========================================================

txtFile = fullfile(resultsFolder, ...
    'passive_bump_metrics.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, 'PASSIVE 7-DOF SYMMETRIC BUMP METRICS\n');
fprintf(fid, '=====================================\n\n');

fprintf(fid, 'HEAVE DISPLACEMENT\n');
fprintf(fid, 'Peak positive heave       = %.6f m\n', peak_heave_pos);
fprintf(fid, 'Peak negative heave       = %.6f m\n', peak_heave_neg);
fprintf(fid, 'Time of positive peak     = %.3f s\n', time_peak_heave_pos);
fprintf(fid, 'Time of negative peak     = %.3f s\n\n', time_peak_heave_neg);

fprintf(fid, 'HEAVE ACCELERATION\n');
fprintf(fid, 'Peak positive accel       = %.3f m/s^2\n', peak_heave_accel_pos);
fprintf(fid, 'Peak negative accel       = %.3f m/s^2\n', peak_heave_accel_neg);
fprintf(fid, 'RMS heave acceleration    = %.3f m/s^2\n\n', rms_heave_accel);

fprintf(fid, 'PITCH ANGLE\n');
fprintf(fid, 'Peak positive pitch       = %.4f deg\n', peak_pitch_pos);
fprintf(fid, 'Peak negative pitch       = %.4f deg\n', peak_pitch_neg);
fprintf(fid, 'Time of positive peak     = %.3f s\n', time_peak_pitch_pos);
fprintf(fid, 'Time of negative peak     = %.3f s\n\n', time_peak_pitch_neg);

fprintf(fid, 'PITCH ACCELERATION\n');
fprintf(fid, 'Peak positive pitch acc   = %.3f deg/s^2\n', peak_pitch_accel_pos);
fprintf(fid, 'Peak negative pitch acc   = %.3f deg/s^2\n', peak_pitch_accel_neg);
fprintf(fid, 'RMS pitch acceleration    = %.3f deg/s^2\n\n', rms_pitch_accel);

fprintf(fid, 'ROLL VALIDATION\n');
fprintf(fid, 'Maximum absolute roll     = %.8f deg\n', max_abs_roll);
fprintf(fid, 'Maximum roll acceleration = %.8f deg/s^2\n\n', ...
    max_abs_roll_accel);

fprintf(fid, 'SETTLING\n');

if isnan(settling_time_heave)
    fprintf(fid, 'Heave settling time       = Not reached\n');
else
    fprintf(fid, 'Heave settling time       = %.3f s\n', ...
        settling_time_heave);
end

fclose(fid);


%% =========================================================
% Save Metrics as CSV
% ==========================================================

Metric = {
    'Peak Positive Heave'
    'Peak Negative Heave'
    'Time Positive Heave Peak'
    'Time Negative Heave Peak'
    'Peak Positive Heave Acceleration'
    'Peak Negative Heave Acceleration'
    'RMS Heave Acceleration'
    'Peak Positive Pitch'
    'Peak Negative Pitch'
    'Time Positive Pitch Peak'
    'Time Negative Pitch Peak'
    'Peak Positive Pitch Acceleration'
    'Peak Negative Pitch Acceleration'
    'RMS Pitch Acceleration'
    'Maximum Absolute Roll'
    'Maximum Absolute Roll Acceleration'
    'Heave Settling Time'
    };

Value = [
    peak_heave_pos
    peak_heave_neg
    time_peak_heave_pos
    time_peak_heave_neg
    peak_heave_accel_pos
    peak_heave_accel_neg
    rms_heave_accel
    peak_pitch_pos
    peak_pitch_neg
    time_peak_pitch_pos
    time_peak_pitch_neg
    peak_pitch_accel_pos
    peak_pitch_accel_neg
    rms_pitch_accel
    max_abs_roll
    max_abs_roll_accel
    settling_time_heave
    ];

Unit = {
    'm'
    'm'
    's'
    's'
    'm/s^2'
    'm/s^2'
    'm/s^2'
    'deg'
    'deg'
    's'
    's'
    'deg/s^2'
    'deg/s^2'
    'deg/s^2'
    'deg'
    'deg/s^2'
    's'
    };

resultsTable = table(Metric, Value, Unit);

csvFile = fullfile(resultsFolder, ...
    'passive_bump_metrics.csv');

writetable(resultsTable, csvFile);


%% =========================================================
% Save MATLAB Data
% ==========================================================

matFile = fullfile(resultsFolder, ...
    'passive_bump_metrics.mat');

save(matFile, ...
    'peak_heave_pos', ...
    'peak_heave_neg', ...
    'time_peak_heave_pos', ...
    'time_peak_heave_neg', ...
    'peak_heave_accel_pos', ...
    'peak_heave_accel_neg', ...
    'rms_heave_accel', ...
    'peak_pitch_pos', ...
    'peak_pitch_neg', ...
    'time_peak_pitch_pos', ...
    'time_peak_pitch_neg', ...
    'peak_pitch_accel_pos', ...
    'peak_pitch_accel_neg', ...
    'rms_pitch_accel', ...
    'max_abs_roll', ...
    'max_abs_roll_accel', ...
    'settling_time_heave');


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Results successfully saved:')
fprintf('TXT : %s\n', txtFile);
fprintf('CSV : %s\n', csvFile);
fprintf('MAT : %s\n', matFile);
