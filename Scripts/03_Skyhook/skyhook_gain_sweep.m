%% Skyhook Controller Gain Sweep
% 7-DOF Active Suspension Race Car Project
%
% Runs the symmetric bump test over multiple Skyhook gains
% and stores performance + actuator-effort metrics.

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
% Load Passive Baseline
% ==========================================================

baselineFile = fullfile( ...
    passiveResultsFolder, ...
    'passive_baseline_summary.mat');

if ~isfile(baselineFile)
    error('passive_baseline_summary.mat not found.');
end

baseline = load(baselineFile);


%% =========================================================
% Model Configuration
% ==========================================================

modelName = 'RaceCar_7DOF_ActiveSuspension';

if ~bdIsLoaded(modelName)
    load_system(fullfile( ...
        projectFolder, ...
        'Model', ...
        [modelName '.slx']));
end

active_mode = 1;

F_act_max = 3000;


%% =========================================================
% Candidate Skyhook Gains
% ==========================================================

gainValues = [ ...
    500 ...
    1000 ...
    1500 ...
    2000 ...
    2500 ...
    3000 ...
    4000 ...
    5000 ...
    6000 ...
    8000];


nGains = length(gainValues);


%% =========================================================
% Preallocate Results
% ==========================================================

peakHeave = zeros(nGains,1);
peakPitch = zeros(nGains,1);

rmsHeaveAccel = zeros(nGains,1);
rmsPitchAccel = zeros(nGains,1);

settlingTime = zeros(nGains,1);

peakForce = zeros(nGains,1);
meanRMSForce = zeros(nGains,1);

maxSaturation = zeros(nGains,1);


%% =========================================================
% Run Gain Sweep
% ==========================================================

disp('====================================================')
disp('SKYHOOK GAIN SWEEP')
disp('====================================================')


for k = 1:nGains

    %% Set gain

    C_sky = gainValues(k);

    fprintf( ...
        '\nRunning C_sky = %.0f N*s/m ...\n', ...
        C_sky);


    %% Push variables to base workspace

    assignin('base', ...
        'C_sky', ...
        C_sky);

    assignin('base', ...
        'active_mode', ...
        active_mode);

    assignin('base', ...
        'F_act_max', ...
        F_act_max);


    %% Run Simulink model

    sim(modelName);


    %% -----------------------------------------------------
    % Vehicle response metrics
    % ------------------------------------------------------

    z_s = z_s_out.Data;

    theta_deg = ...
        rad2deg(theta_out.Data);

    z_s_ddot = ...
        z_s_ddot_out.Data;

    theta_ddot_deg = ...
        rad2deg(theta_ddot_out.Data);

    t = z_s_out.Time;


    peakHeave(k) = ...
        1000 * max(abs(z_s));

    peakPitch(k) = ...
        max(abs(theta_deg));

    rmsHeaveAccel(k) = ...
        rms(z_s_ddot);

    rmsPitchAccel(k) = ...
        rms(theta_ddot_deg);


    %% -----------------------------------------------------
    % Settling time
    % ------------------------------------------------------

    threshold = ...
        0.02 * max(abs(z_s));

    settlingTime(k) = NaN;


    for i = 1:length(z_s)

        if t(i) >= t_bump_front

            remaining = ...
                abs(z_s(i:end));

            if all(remaining <= threshold)

                settlingTime(k) = t(i);

                break;

            end

        end

    end


    %% -----------------------------------------------------
    % Actuator forces
    % ------------------------------------------------------

    FL = F_act_FL_out.Data;
    FR = F_act_FR_out.Data;
    RL = F_act_RL_out.Data;
    RR = F_act_RR_out.Data;


    peakForce(k) = max([ ...
        max(abs(FL)), ...
        max(abs(FR)), ...
        max(abs(RL)), ...
        max(abs(RR))]);


    meanRMSForce(k) = mean([ ...
        rms(FL), ...
        rms(FR), ...
        rms(RL), ...
        rms(RR)]);


    satFL = ...
        100 * mean(abs(FL) >= 0.999*F_act_max);

    satFR = ...
        100 * mean(abs(FR) >= 0.999*F_act_max);

    satRL = ...
        100 * mean(abs(RL) >= 0.999*F_act_max);

    satRR = ...
        100 * mean(abs(RR) >= 0.999*F_act_max);


    maxSaturation(k) = ...
        max([satFL satFR satRL satRR]);


    fprintf( ...
        'Completed. Peak force = %.1f N\n', ...
        peakForce(k));

end


%% =========================================================
% Calculate Improvements vs Passive
% ==========================================================

heaveImprovement = ...
    100 * ...
    (baseline.peak_heave_mm - peakHeave) ...
    ./ baseline.peak_heave_mm;


pitchImprovement = ...
    100 * ...
    (baseline.peak_pitch_deg - peakPitch) ...
    ./ baseline.peak_pitch_deg;


heaveAccelImprovement = ...
    100 * ...
    (baseline.rms_heave_accel - rmsHeaveAccel) ...
    ./ baseline.rms_heave_accel;


pitchAccelImprovement = ...
    100 * ...
    (baseline.rms_pitch_accel - rmsPitchAccel) ...
    ./ baseline.rms_pitch_accel;


settlingImprovement = ...
    100 * ...
    (baseline.settling_time - settlingTime) ...
    ./ baseline.settling_time;


%% =========================================================
% Results Table
% ==========================================================

Gain_Nsm = gainValues(:);

resultsTable = table( ...
    Gain_Nsm, ...
    peakHeave, ...
    peakPitch, ...
    rmsHeaveAccel, ...
    rmsPitchAccel, ...
    settlingTime, ...
    peakForce, ...
    meanRMSForce, ...
    maxSaturation, ...
    heaveImprovement, ...
    pitchImprovement, ...
    heaveAccelImprovement, ...
    pitchAccelImprovement, ...
    settlingImprovement);


%% =========================================================
% Display Table
% ==========================================================

disp(' ')
disp(resultsTable)


%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'skyhook_gain_sweep.csv');

writetable( ...
    resultsTable, ...
    csvFile);


%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'skyhook_gain_sweep.mat');

save( ...
    matFile, ...
    'resultsTable', ...
    'gainValues');


%% =========================================================
% Save Human-Readable TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'skyhook_gain_sweep.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'SKYHOOK CONTROLLER GAIN SWEEP\n');

fprintf(fid, ...
    '=============================\n\n');


for k = 1:nGains

    fprintf(fid, ...
        'C_sky = %.0f N*s/m\n', ...
        gainValues(k));

    fprintf(fid, ...
        'Peak Heave = %.3f mm\n', ...
        peakHeave(k));

    fprintf(fid, ...
        'Peak Pitch = %.4f deg\n', ...
        peakPitch(k));

    fprintf(fid, ...
        'RMS Heave Accel = %.4f m/s^2\n', ...
        rmsHeaveAccel(k));

    fprintf(fid, ...
        'RMS Pitch Accel = %.4f deg/s^2\n', ...
        rmsPitchAccel(k));

    fprintf(fid, ...
        'Settling Time = %.3f s\n', ...
        settlingTime(k));

    fprintf(fid, ...
        'Peak Force = %.2f N\n', ...
        peakForce(k));

    fprintf(fid, ...
        'Max Saturation = %.3f %%\n\n', ...
        maxSaturation(k));

end

fclose(fid);


%% =========================================================
% Plot 1 - Body Motion
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'Skyhook Gain Sweep - Body Motion', ...
    'NumberTitle', ...
    'off');


subplot(2,1,1)

plot( ...
    gainValues, ...
    peakHeave, ...
    '-o', ...
    'LineWidth', ...
    1.5);

grid on

xlabel('C_{sky} [N s/m]')
ylabel('Peak Heave [mm]')

title('Peak Heave vs Skyhook Gain')


subplot(2,1,2)

plot( ...
    gainValues, ...
    peakPitch, ...
    '-o', ...
    'LineWidth', ...
    1.5);

grid on

xlabel('C_{sky} [N s/m]')
ylabel('Peak Pitch [deg]')

title('Peak Pitch vs Skyhook Gain')


sgtitle('Skyhook Gain Sweep - Body Motion');


motionPNG = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_sweep_body_motion.png');

motionFIG = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_sweep_body_motion.fig');

exportgraphics( ...
    fig1, ...
    motionPNG, ...
    'Resolution', ...
    300);

savefig(fig1, motionFIG);


%% =========================================================
% Plot 2 - Body Acceleration
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Skyhook Gain Sweep - Acceleration', ...
    'NumberTitle', ...
    'off');


subplot(2,1,1)

plot( ...
    gainValues, ...
    rmsHeaveAccel, ...
    '-o', ...
    'LineWidth', ...
    1.5);

grid on

xlabel('C_{sky} [N s/m]')
ylabel('RMS Heave Accel [m/s^2]')

title('RMS Heave Acceleration vs Skyhook Gain')


subplot(2,1,2)

plot( ...
    gainValues, ...
    rmsPitchAccel, ...
    '-o', ...
    'LineWidth', ...
    1.5);

grid on

xlabel('C_{sky} [N s/m]')
ylabel('RMS Pitch Accel [deg/s^2]')

title('RMS Pitch Acceleration vs Skyhook Gain')


sgtitle('Skyhook Gain Sweep - Ride Response');


accelPNG = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_sweep_acceleration.png');

accelFIG = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_sweep_acceleration.fig');

exportgraphics( ...
    fig2, ...
    accelPNG, ...
    'Resolution', ...
    300);

savefig(fig2, accelFIG);


%% =========================================================
% Plot 3 - Actuator Effort
% ==========================================================

fig3 = figure( ...
    'Name', ...
    'Skyhook Gain Sweep - Actuator Effort', ...
    'NumberTitle', ...
    'off');


plot( ...
    gainValues, ...
    peakForce, ...
    '-o', ...
    'LineWidth', ...
    1.5);

hold on

yline( ...
    F_act_max, ...
    '--', ...
    'Actuator Limit');

grid on

xlabel('C_{sky} [N s/m]')
ylabel('Peak Actuator Force [N]')

title('Actuator Effort vs Skyhook Gain')


forcePNG = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_sweep_actuator_force.png');

forceFIG = fullfile( ...
    plotsFolder, ...
    'skyhook_gain_sweep_actuator_force.fig');

exportgraphics( ...
    fig3, ...
    forcePNG, ...
    'Resolution', ...
    300);

savefig(fig3, forceFIG);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Skyhook gain sweep completed successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', motionPNG);
fprintf('%s\n', accelPNG);
fprintf('%s\n', forcePNG);
