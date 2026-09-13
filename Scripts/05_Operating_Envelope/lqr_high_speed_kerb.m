%% Final Tuned LQR - High-Speed Asymmetric Kerb
% Compares 72 km/h and 144 km/h kerb behaviour

clear;
clc;

%% =========================================================
% Vehicle Parameters
% ==========================================================

run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), ...
    'Scripts', '01_Parameters', 'vehicle_parameters.m'));

%% =========================================================
% Project Paths
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

resultsFolder = fullfile(projectFolder, 'Results', 'Operating_Envelope');
plotsFolder = fullfile(projectFolder, 'Plots', 'Operating_Envelope');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

%% =========================================================
% Load Final Tuned LQR
% ==========================================================

controllerData = load(fullfile( ...
    lqrResultsFolder, ...
    'final_tuned_lqr_controller.mat'));

K_lqr = controllerData.K_lqr_final;

active_mode = 1;
F_act_max = 3000;

%% =========================================================
% Model
% ==========================================================

modelName = 'RaceCar_7DOF_ActiveSuspension';

load_system(fullfile( ...
    projectFolder, ...
    'Model', ...
    [modelName '.slx']));

%% =========================================================
% Test Speeds
% ==========================================================

V_test = [20 40];

nCases = length(V_test);

Speed_mps = zeros(nCases,1);
Speed_kph = zeros(nCases,1);

KerbDuration_s = zeros(nCases,1);
RearDelay_s = zeros(nCases,1);

PeakHeave_mm = zeros(nCases,1);
PeakPitch_deg = zeros(nCases,1);
PeakRoll_deg = zeros(nCases,1);

RMSHeaveAccel = zeros(nCases,1);
RMSPitchAccel = zeros(nCases,1);
RMSRollAccel = zeros(nCases,1);

RollSettlingTime_s = zeros(nCases,1);

PeakForce_N = zeros(nCases,1);
MeanRMSForce_N = zeros(nCases,1);
MaxSaturation_pct = zeros(nCases,1);

%% =========================================================
% Simulation Loop
% ==========================================================

disp('====================================================')
disp('FINAL LQR - HIGH-SPEED ASYMMETRIC KERB')
disp('====================================================')

for k = 1:nCases

    V_case = V_test(k);

    kerb_duration_case = ...
        kerb_length / V_case;

    rear_delay_case = ...
        L / V_case;

    t_kerb_rear_case = ...
        t_kerb_front + rear_delay_case;

    %% Push values to Simulink

    assignin('base','V',V_case);

    assignin( ...
        'base', ...
        'kerb_duration', ...
        kerb_duration_case);

    assignin( ...
        'base', ...
        'rear_delay', ...
        rear_delay_case);

    assignin( ...
        'base', ...
        't_kerb_rear', ...
        t_kerb_rear_case);

    assignin( ...
        'base', ...
        'K_lqr', ...
        K_lqr);

    assignin( ...
        'base', ...
        'active_mode', ...
        1);

    assignin( ...
        'base', ...
        'F_act_max', ...
        F_act_max);

    fprintf( ...
        '\nRunning %.1f m/s (%.0f km/h)\n', ...
        V_case, ...
        V_case*3.6);

    %% Simulation

    simOut = sim( ...
        modelName, ...
        'StopTime', ...
        '5', ...
        'ReturnWorkspaceOutputs', ...
        'on');

    %% Signals

    zsSig = getLoggedSignal(simOut,'z_s_out');
    thetaSig = getLoggedSignal(simOut,'theta_out');
    phiSig = getLoggedSignal(simOut,'phi_out');

    zsDDotSig = getLoggedSignal(simOut,'z_s_ddot_out');
    thetaDDotSig = getLoggedSignal(simOut,'theta_ddot_out');
    phiDDotSig = getLoggedSignal(simOut,'phi_ddot_out');

    flSig = getLoggedSignal(simOut,'F_act_FL_out');
    frSig = getLoggedSignal(simOut,'F_act_FR_out');
    rlSig = getLoggedSignal(simOut,'F_act_RL_out');
    rrSig = getLoggedSignal(simOut,'F_act_RR_out');

    %% Data

    t = zsSig.Time;

    zs = zsSig.Data;

    theta = rad2deg(thetaSig.Data);
    phi = rad2deg(phiSig.Data);

    zsDDot = zsDDotSig.Data;

    thetaDDot = ...
        rad2deg(thetaDDotSig.Data);

    phiDDot = ...
        rad2deg(phiDDotSig.Data);

    FL = flSig.Data;
    FR = frSig.Data;
    RL = rlSig.Data;
    RR = rrSig.Data;

    %% Metrics

    PeakHeave_mm(k) = ...
        1000*max(abs(zs));

    PeakPitch_deg(k) = ...
        max(abs(theta));

    PeakRoll_deg(k) = ...
        max(abs(phi));

    RMSHeaveAccel(k) = ...
        rms(zsDDot);

    RMSPitchAccel(k) = ...
        rms(thetaDDot);

    RMSRollAccel(k) = ...
        rms(phiDDot);

    %% Roll settling

    threshold = ...
        0.02*max(abs(phi));

    settlingTime = NaN;

    for i = 1:length(phi)

        if t(i) >= t_kerb_front

            if all(abs(phi(i:end)) <= threshold)

                settlingTime = t(i);
                break;

            end

        end

    end

    RollSettlingTime_s(k) = ...
        settlingTime;

    %% Actuator effort

    PeakForce_N(k) = max([ ...
        max(abs(FL)), ...
        max(abs(FR)), ...
        max(abs(RL)), ...
        max(abs(RR))]);

    MeanRMSForce_N(k) = mean([ ...
        rms(FL), ...
        rms(FR), ...
        rms(RL), ...
        rms(RR)]);

    MaxSaturation_pct(k) = max([ ...
        100*mean(abs(FL) >= 0.999*F_act_max), ...
        100*mean(abs(FR) >= 0.999*F_act_max), ...
        100*mean(abs(RL) >= 0.999*F_act_max), ...
        100*mean(abs(RR) >= 0.999*F_act_max)]);

    %% Setup values

    Speed_mps(k) = V_case;
    Speed_kph(k) = V_case*3.6;

    KerbDuration_s(k) = ...
        kerb_duration_case;

    RearDelay_s(k) = ...
        rear_delay_case;

    fprintf( ...
        ['Peak roll = %.5f deg | ' ...
         'Roll RMS accel = %.2f deg/s^2 | ' ...
         'Peak force = %.1f N\n'], ...
        PeakRoll_deg(k), ...
        RMSRollAccel(k), ...
        PeakForce_N(k));

end

%% =========================================================
% Table
% ==========================================================

kerbSpeedTable = table( ...
    Speed_mps, ...
    Speed_kph, ...
    KerbDuration_s, ...
    RearDelay_s, ...
    PeakHeave_mm, ...
    PeakPitch_deg, ...
    PeakRoll_deg, ...
    RMSHeaveAccel, ...
    RMSPitchAccel, ...
    RMSRollAccel, ...
    RollSettlingTime_s, ...
    PeakForce_N, ...
    MeanRMSForce_N, ...
    MaxSaturation_pct);

disp(' ')
disp('====================================================')
disp('HIGH-SPEED KERB RESULTS')
disp('====================================================')

disp(kerbSpeedTable);

%% =========================================================
% Save Results
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'lqr_high_speed_kerb.csv');

writetable(kerbSpeedTable,csvFile);

matFile = fullfile( ...
    resultsFolder, ...
    'lqr_high_speed_kerb.mat');

save(matFile, ...
    'kerbSpeedTable');

txtFile = fullfile( ...
    resultsFolder, ...
    'lqr_high_speed_kerb.txt');

fid = fopen(txtFile,'w');

fprintf(fid, ...
    'FINAL TUNED LQR - HIGH SPEED KERB\n');

fprintf(fid, ...
    '================================\n\n');

for k = 1:nCases

    fprintf(fid, ...
        'Speed = %.1f km/h\n', ...
        Speed_kph(k));

    fprintf(fid, ...
        'Kerb duration = %.6f s\n', ...
        KerbDuration_s(k));

    fprintf(fid, ...
        'Rear delay = %.6f s\n', ...
        RearDelay_s(k));

    fprintf(fid, ...
        'Peak heave = %.5f mm\n', ...
        PeakHeave_mm(k));

    fprintf(fid, ...
        'Peak pitch = %.6f deg\n', ...
        PeakPitch_deg(k));

    fprintf(fid, ...
        'Peak roll = %.6f deg\n', ...
        PeakRoll_deg(k));

    fprintf(fid, ...
        'RMS heave accel = %.6f m/s^2\n', ...
        RMSHeaveAccel(k));

    fprintf(fid, ...
        'RMS pitch accel = %.6f deg/s^2\n', ...
        RMSPitchAccel(k));

    fprintf(fid, ...
        'RMS roll accel = %.6f deg/s^2\n', ...
        RMSRollAccel(k));

    fprintf(fid, ...
        'Roll settling time = %.4f s\n', ...
        RollSettlingTime_s(k));

    fprintf(fid, ...
        'Peak actuator force = %.2f N\n', ...
        PeakForce_N(k));

    fprintf(fid, ...
        'Mean RMS actuator force = %.2f N\n', ...
        MeanRMSForce_N(k));

    fprintf(fid, ...
        'Maximum saturation = %.4f %%\n\n', ...
        MaxSaturation_pct(k));

end

fclose(fid);

%% =========================================================
% Body Motion Plot
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'High-Speed Kerb Body Motion', ...
    'NumberTitle', ...
    'off');

subplot(3,1,1)

plot(Speed_kph,PeakHeave_mm,'-o','LineWidth',1.5);
grid on

ylabel('Peak Heave [mm]')
title('High-Speed Kerb - Heave');

subplot(3,1,2)

plot(Speed_kph,PeakPitch_deg,'-o','LineWidth',1.5);
grid on

ylabel('Peak Pitch [deg]')
title('High-Speed Kerb - Pitch');

subplot(3,1,3)

plot(Speed_kph,PeakRoll_deg,'-o','LineWidth',1.5);
grid on

xlabel('Vehicle Speed [km/h]')
ylabel('Peak Roll [deg]')
title('High-Speed Kerb - Roll');

bodyPNG = fullfile( ...
    projectFolder, 'Plots', 'Final', ...
    'lqr_high_speed_kerb_body_motion.png');

bodyFIG = fullfile( ...
    projectFolder, 'Plots', 'Final', ...
    'lqr_high_speed_kerb_body_motion.fig');

exportgraphics(fig1,bodyPNG,'Resolution',300);
savefig(fig1,bodyFIG);

%% =========================================================
% Acceleration Plot
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'High-Speed Kerb Acceleration', ...
    'NumberTitle', ...
    'off');

subplot(3,1,1)

plot(Speed_kph,RMSHeaveAccel,'-o','LineWidth',1.5);
grid on

ylabel('Heave [m/s^2]')
title('RMS Heave Acceleration');

subplot(3,1,2)

plot(Speed_kph,RMSPitchAccel,'-o','LineWidth',1.5);
grid on

ylabel('Pitch [deg/s^2]')
title('RMS Pitch Acceleration');

subplot(3,1,3)

plot(Speed_kph,RMSRollAccel,'-o','LineWidth',1.5);
grid on

xlabel('Vehicle Speed [km/h]')
ylabel('Roll [deg/s^2]')
title('RMS Roll Acceleration');

accelPNG = fullfile( ...
    plotsFolder, ...
    'lqr_high_speed_kerb_acceleration.png');

accelFIG = fullfile( ...
    plotsFolder, ...
    'lqr_high_speed_kerb_acceleration.fig');

exportgraphics(fig2,accelPNG,'Resolution',300);
savefig(fig2,accelFIG);

%% =========================================================
% Actuator Plot
% ==========================================================

fig3 = figure( ...
    'Name', ...
    'High-Speed Kerb Actuator Demand', ...
    'NumberTitle', ...
    'off');

plot( ...
    Speed_kph, ...
    PeakForce_N, ...
    '-o', ...
    'LineWidth',1.5);

hold on

yline(F_act_max,'--');

grid on

xlabel('Vehicle Speed [km/h]')
ylabel('Peak Actuator Force [N]')

title('High-Speed Kerb - Peak Actuator Demand');

actuatorPNG = fullfile( ...
    plotsFolder, ...
    'lqr_high_speed_kerb_actuator.png');

actuatorFIG = fullfile( ...
    plotsFolder, ...
    'lqr_high_speed_kerb_actuator.fig');

exportgraphics( ...
    fig3, ...
    actuatorPNG, ...
    'Resolution',300);

savefig(fig3,actuatorFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('High-speed kerb evaluation saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n',txtFile);
fprintf('%s\n',csvFile);
fprintf('%s\n',matFile);

fprintf('\nPlots:\n');
fprintf('%s\n',bodyPNG);
fprintf('%s\n',accelPNG);
fprintf('%s\n',actuatorPNG);

%% =========================================================
% Helper
% ==========================================================

function sig = getLoggedSignal(simOut,signalName)

    try
        sig = simOut.get(signalName);

        if ~isempty(sig)
            return;
        end
    catch
    end

    if evalin( ...
            'base', ...
            sprintf( ...
            'exist(''%s'',''var'')', ...
            signalName))

        sig = evalin( ...
            'base', ...
            signalName);

        return;
    end

    error( ...
        'Unable to locate logged signal: %s', ...
        signalName);

end
