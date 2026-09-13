%% Final Tuned LQR - Parameter Robustness Sweep
% Step 11A - Robustness screening under model uncertainty
%
% Cases:
% 1. Nominal
% 2. +10% sprung mass
% 3. -10% suspension stiffness
% 4. -10% damping
% 5. -10% tyre stiffness
%
% Test:
% Symmetric bump
%
% Saved outputs:
% Results/lqr_robustness_sweep.txt
% Results/lqr_robustness_sweep.csv
% Results/lqr_robustness_sweep.mat
%
% Plots/lqr_robustness_body_motion.png
% Plots/lqr_robustness_acceleration.png
% Plots/lqr_robustness_actuator_force.png
% Plots/lqr_robustness_normalized_summary.png

clear;
clc;
close all;

%% =========================================================
% Load Nominal Vehicle Parameters
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

resultsFolder = fullfile(projectFolder, 'Results', 'Robustness');
plotsFolder = fullfile(projectFolder, 'Plots', 'Robustness');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

%% =========================================================
% Load Final Tuned LQR Controller
% ==========================================================

controllerFile = fullfile( ...
    lqrResultsFolder, ...
    'final_tuned_lqr_controller.mat');

if ~isfile(controllerFile)
    error('Final tuned LQR controller file not found: %s', controllerFile);
end

controllerData = load(controllerFile);

K_lqr = controllerData.K_lqr_final;

assignin('base', 'K_lqr', K_lqr);

%% =========================================================
% Active Suspension Settings
% ==========================================================

active_mode = 1;
F_act_max = 3000;

assignin('base', 'active_mode', active_mode);
assignin('base', 'F_act_max', F_act_max);

%% =========================================================
% Symmetric Bump Operating Condition
% ==========================================================

V = 20;          % [m/s] = 72 km/h

bump_duration = bump_length / V;
rear_delay = L / V;
t_bump_rear = t_bump_front + rear_delay;

assignin('base', 'V', V);
assignin('base', 'bump_duration', bump_duration);
assignin('base', 'rear_delay', rear_delay);
assignin('base', 't_bump_rear', t_bump_rear);

%% =========================================================
% Store Nominal Parameters
% ==========================================================

nominal.m_s  = m_s;

nominal.k_sf = k_sf;
nominal.k_sr = k_sr;

nominal.c_sf = c_sf;
nominal.c_sr = c_sr;

nominal.k_t  = k_t;

%% =========================================================
% Robustness Cases
% ==========================================================

caseNames = { ...
    'Nominal'; ...
    '+10% Sprung Mass'; ...
    '-10% Suspension Stiffness'; ...
    '-10% Damping'; ...
    '-10% Tyre Stiffness'};

nCases = numel(caseNames);

%% =========================================================
% Result Storage
% ==========================================================

PeakHeave_mm = zeros(nCases,1);
PeakPitch_deg = zeros(nCases,1);

RMSHeaveAccel_mps2 = zeros(nCases,1);
RMSPitchAccel_degps2 = zeros(nCases,1);

SettlingTime_s = zeros(nCases,1);

PeakActuatorForce_N = zeros(nCases,1);
MeanRMSActuatorForce_N = zeros(nCases,1);

MaxSaturation_pct = zeros(nCases,1);

ClosedLoopStable = false(nCases,1);

%% =========================================================
% Time-History Storage
% ==========================================================

timeHistory = cell(nCases,1);

heaveHistory = cell(nCases,1);
pitchHistory = cell(nCases,1);

heaveAccelHistory = cell(nCases,1);
pitchAccelHistory = cell(nCases,1);

FLforceHistory = cell(nCases,1);
FRforceHistory = cell(nCases,1);
RLforceHistory = cell(nCases,1);
RRforceHistory = cell(nCases,1);

%% =========================================================
% Model
% ==========================================================

modelName = 'RaceCar_7DOF_ActiveSuspension';

modelFile = fullfile( ...
    projectFolder, ...
    'Model', ...
    [modelName '.slx']);

if ~isfile(modelFile)
    error('Simulink model not found: %s', modelFile);
end

load_system(modelFile);

%% =========================================================
% Sweep
% ==========================================================

disp('====================================================')
disp('FINAL TUNED LQR - PARAMETER ROBUSTNESS SWEEP')
disp('====================================================')

for caseIdx = 1:nCases

    %% -----------------------------------------------------
    % Reset Nominal Parameters
    % ------------------------------------------------------

    m_s_case = nominal.m_s;

    k_sf_case = nominal.k_sf;
    k_sr_case = nominal.k_sr;

    c_sf_case = nominal.c_sf;
    c_sr_case = nominal.c_sr;

    k_t_case = nominal.k_t;

    %% -----------------------------------------------------
    % Apply Parameter Variation
    % ------------------------------------------------------

    switch caseIdx

        case 1
            % Nominal

        case 2
            % +10% sprung mass
            m_s_case = 1.10 * nominal.m_s;

        case 3
            % -10% suspension stiffness
            k_sf_case = 0.90 * nominal.k_sf;
            k_sr_case = 0.90 * nominal.k_sr;

        case 4
            % -10% damping
            c_sf_case = 0.90 * nominal.c_sf;
            c_sr_case = 0.90 * nominal.c_sr;

        case 5
            % -10% tyre stiffness
            k_t_case = 0.90 * nominal.k_t;

    end

    %% -----------------------------------------------------
    % Push Case Parameters to Base Workspace
    % ------------------------------------------------------

    assignin('base', 'm_s', m_s_case);

    assignin('base', 'k_sf', k_sf_case);
    assignin('base', 'k_sr', k_sr_case);

    assignin('base', 'c_sf', c_sf_case);
    assignin('base', 'c_sr', c_sr_case);

    assignin('base', 'k_t', k_t_case);

    assignin('base', 'K_lqr', K_lqr);
    assignin('base', 'active_mode', 1);
    assignin('base', 'F_act_max', F_act_max);

    fprintf( ...
        '\nRunning case %d/%d: %s\n', ...
        caseIdx, ...
        nCases, ...
        caseNames{caseIdx});

    %% -----------------------------------------------------
    % Run Simulation
    % ------------------------------------------------------

    simOut = sim( ...
        modelName, ...
        'StopTime', ...
        '5', ...
        'ReturnWorkspaceOutputs', ...
        'on');

    %% -----------------------------------------------------
    % Get Signals
    % ------------------------------------------------------

    zs = getLoggedSignal(simOut, 'z_s_out');
    theta = getLoggedSignal(simOut, 'theta_out');

    zsDDot = getLoggedSignal(simOut, 'z_s_ddot_out');
    thetaDDot = getLoggedSignal(simOut, 'theta_ddot_out');

    FLforce = getLoggedSignal(simOut, 'F_act_FL_out');
    FRforce = getLoggedSignal(simOut, 'F_act_FR_out');
    RLforce = getLoggedSignal(simOut, 'F_act_RL_out');
    RRforce = getLoggedSignal(simOut, 'F_act_RR_out');

    %% -----------------------------------------------------
    % Extract Data
    % ------------------------------------------------------

    t = zs.Time;

    heave = zs.Data;
    pitch = rad2deg(theta.Data);

    heaveAccel = zsDDot.Data;
    pitchAccel = rad2deg(thetaDDot.Data);

    F_FL = FLforce.Data;
    F_FR = FRforce.Data;
    F_RL = RLforce.Data;
    F_RR = RRforce.Data;

    %% -----------------------------------------------------
    % Metrics
    % ------------------------------------------------------

    PeakHeave_mm(caseIdx) = ...
        1000 * max(abs(heave));

    PeakPitch_deg(caseIdx) = ...
        max(abs(pitch));

    RMSHeaveAccel_mps2(caseIdx) = ...
        rms(heaveAccel);

    RMSPitchAccel_degps2(caseIdx) = ...
        rms(pitchAccel);

    PeakActuatorForce_N(caseIdx) = ...
        max([ ...
            max(abs(F_FL)), ...
            max(abs(F_FR)), ...
            max(abs(F_RL)), ...
            max(abs(F_RR))]);

    RMSforces = [ ...
        rms(F_FL), ...
        rms(F_FR), ...
        rms(F_RL), ...
        rms(F_RR)];

    MeanRMSActuatorForce_N(caseIdx) = ...
        mean(RMSforces);

    saturationPctCorners = 100 * [ ...
        mean(abs(F_FL) >= 0.999*F_act_max), ...
        mean(abs(F_FR) >= 0.999*F_act_max), ...
        mean(abs(F_RL) >= 0.999*F_act_max), ...
        mean(abs(F_RR) >= 0.999*F_act_max)];

    MaxSaturation_pct(caseIdx) = ...
        max(saturationPctCorners);

    %% -----------------------------------------------------
    % Settling Time
    % ------------------------------------------------------
    %
    % Same general logic used previously:
    % after bump event, find last sample outside tolerance.
    %
    % Tolerance:
    % 2% of peak heave response with small absolute floor.

    peakHeave_m = max(abs(heave));

    settlingTolerance = max( ...
        0.02 * peakHeave_m, ...
        1e-5);

    eventEndTime = ...
        t_bump_rear + bump_duration;

    postEventIdx = ...
        find(t >= eventEndTime);

    heavePost = ...
        abs(heave(postEventIdx));

    lastOutside = ...
        find( ...
            heavePost > settlingTolerance, ...
            1, ...
            'last');

    if isempty(lastOutside)

        SettlingTime_s(caseIdx) = ...
            eventEndTime;

    else

        globalIdx = ...
            postEventIdx(lastOutside);

        SettlingTime_s(caseIdx) = ...
            t(globalIdx);

    end

    %% -----------------------------------------------------
    % Basic Stability Check
    % ------------------------------------------------------
    %
    % For this simulation-level robustness screen:
    % stable = finite signals and bounded response.

    finiteCheck = ...
        all(isfinite(heave)) && ...
        all(isfinite(pitch)) && ...
        all(isfinite(heaveAccel)) && ...
        all(isfinite(pitchAccel)) && ...
        all(isfinite(F_FL)) && ...
        all(isfinite(F_FR)) && ...
        all(isfinite(F_RL)) && ...
        all(isfinite(F_RR));

    boundedCheck = ...
        PeakHeave_mm(caseIdx) < 100 && ...
        PeakPitch_deg(caseIdx) < 10 && ...
        PeakActuatorForce_N(caseIdx) <= 1.01*F_act_max;

    ClosedLoopStable(caseIdx) = ...
        finiteCheck && boundedCheck;

    %% -----------------------------------------------------
    % Save Time Histories
    % ------------------------------------------------------

    timeHistory{caseIdx} = t;

    heaveHistory{caseIdx} = ...
        1000 * heave;

    pitchHistory{caseIdx} = ...
        pitch;

    heaveAccelHistory{caseIdx} = ...
        heaveAccel;

    pitchAccelHistory{caseIdx} = ...
        pitchAccel;

    FLforceHistory{caseIdx} = ...
        F_FL;

    FRforceHistory{caseIdx} = ...
        F_FR;

    RLforceHistory{caseIdx} = ...
        F_RL;

    RRforceHistory{caseIdx} = ...
        F_RR;

    %% -----------------------------------------------------
    % Console Output
    % ------------------------------------------------------

    fprintf( ...
        'Peak heave = %.4f mm | ', ...
        PeakHeave_mm(caseIdx));

    fprintf( ...
        'Peak pitch = %.5f deg | ', ...
        PeakPitch_deg(caseIdx));

    fprintf( ...
        'RMS heave accel = %.4f m/s^2 | ', ...
        RMSHeaveAccel_mps2(caseIdx));

    fprintf( ...
        'Peak actuator = %.1f N\n', ...
        PeakActuatorForce_N(caseIdx));

end

%% =========================================================
% Restore Nominal Parameters
% ==========================================================

assignin('base', 'm_s', nominal.m_s);

assignin('base', 'k_sf', nominal.k_sf);
assignin('base', 'k_sr', nominal.k_sr);

assignin('base', 'c_sf', nominal.c_sf);
assignin('base', 'c_sr', nominal.c_sr);

assignin('base', 'k_t', nominal.k_t);

assignin('base', 'K_lqr', K_lqr);
assignin('base', 'active_mode', active_mode);
assignin('base', 'F_act_max', F_act_max);

%% =========================================================
% Build Results Table
% ==========================================================

robustnessTable = table( ...
    caseNames, ...
    PeakHeave_mm, ...
    PeakPitch_deg, ...
    RMSHeaveAccel_mps2, ...
    RMSPitchAccel_degps2, ...
    SettlingTime_s, ...
    PeakActuatorForce_N, ...
    MeanRMSActuatorForce_N, ...
    MaxSaturation_pct, ...
    ClosedLoopStable, ...
    'VariableNames', { ...
    'Case', ...
    'PeakHeave_mm', ...
    'PeakPitch_deg', ...
    'RMSHeaveAccel_mps2', ...
    'RMSPitchAccel_degps2', ...
    'SettlingTime_s', ...
    'PeakActuatorForce_N', ...
    'MeanRMSActuatorForce_N', ...
    'MaxSaturation_pct', ...
    'Stable'});

%% =========================================================
% Percentage Change vs Nominal
% ==========================================================

heaveChange_pct = ...
    100 * ...
    (PeakHeave_mm / PeakHeave_mm(1) - 1);

pitchChange_pct = ...
    100 * ...
    (PeakPitch_deg / PeakPitch_deg(1) - 1);

heaveAccelChange_pct = ...
    100 * ...
    (RMSHeaveAccel_mps2 / RMSHeaveAccel_mps2(1) - 1);

pitchAccelChange_pct = ...
    100 * ...
    (RMSPitchAccel_degps2 / RMSPitchAccel_degps2(1) - 1);

settlingChange_pct = ...
    100 * ...
    (SettlingTime_s / SettlingTime_s(1) - 1);

forceChange_pct = ...
    100 * ...
    (PeakActuatorForce_N / PeakActuatorForce_N(1) - 1);

robustnessTable.HeaveChange_pct = ...
    heaveChange_pct;

robustnessTable.PitchChange_pct = ...
    pitchChange_pct;

robustnessTable.HeaveAccelChange_pct = ...
    heaveAccelChange_pct;

robustnessTable.PitchAccelChange_pct = ...
    pitchAccelChange_pct;

robustnessTable.SettlingChange_pct = ...
    settlingChange_pct;

robustnessTable.PeakForceChange_pct = ...
    forceChange_pct;

%% =========================================================
% Display Results
% ==========================================================

disp(' ')
disp('====================================================')
disp('LQR ROBUSTNESS RESULTS')
disp('====================================================')

disp(robustnessTable)

%% =========================================================
% Identify Worst Cases
% ==========================================================

[worstHeave, idxWorstHeave] = ...
    max(PeakHeave_mm);

[worstPitch, idxWorstPitch] = ...
    max(PeakPitch_deg);

[worstHeaveAccel, idxWorstHeaveAccel] = ...
    max(RMSHeaveAccel_mps2);

[worstPitchAccel, idxWorstPitchAccel] = ...
    max(RMSPitchAccel_degps2);

[worstForce, idxWorstForce] = ...
    max(PeakActuatorForce_N);

fprintf('\nWorst peak heave:\n');
fprintf( ...
    '%s = %.4f mm\n', ...
    caseNames{idxWorstHeave}, ...
    worstHeave);

fprintf('\nWorst peak pitch:\n');
fprintf( ...
    '%s = %.5f deg\n', ...
    caseNames{idxWorstPitch}, ...
    worstPitch);

fprintf('\nWorst RMS heave acceleration:\n');
fprintf( ...
    '%s = %.4f m/s^2\n', ...
    caseNames{idxWorstHeaveAccel}, ...
    worstHeaveAccel);

fprintf('\nWorst RMS pitch acceleration:\n');
fprintf( ...
    '%s = %.4f deg/s^2\n', ...
    caseNames{idxWorstPitchAccel}, ...
    worstPitchAccel);

fprintf('\nHighest actuator demand:\n');
fprintf( ...
    '%s = %.2f N\n', ...
    caseNames{idxWorstForce}, ...
    worstForce);

%% =========================================================
% Overall Robustness Result
% ==========================================================

if all(ClosedLoopStable)

    robustnessResult = ...
        'PASS - all tested parameter-variation cases remained bounded and stable.';

else

    robustnessResult = ...
        'WARNING - at least one tested uncertainty case failed the stability/boundedness check.';

end

fprintf('\n%s\n', robustnessResult);

%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'lqr_robustness_sweep.csv');

writetable( ...
    robustnessTable, ...
    csvFile);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'lqr_robustness_sweep.mat');

save( ...
    matFile, ...
    'robustnessTable', ...
    'caseNames', ...
    'timeHistory', ...
    'heaveHistory', ...
    'pitchHistory', ...
    'heaveAccelHistory', ...
    'pitchAccelHistory', ...
    'FLforceHistory', ...
    'FRforceHistory', ...
    'RLforceHistory', ...
    'RRforceHistory', ...
    'K_lqr', ...
    'robustnessResult');

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'lqr_robustness_sweep.txt');

fid = fopen(txtFile, 'w');

if fid == -1
    error('Unable to create robustness TXT file.');
end

fprintf(fid, ...
    'FINAL TUNED LQR - PARAMETER ROBUSTNESS SWEEP\n');

fprintf(fid, ...
    '============================================\n\n');

fprintf(fid, ...
    'Test condition: Symmetric bump at %.1f km/h\n\n', ...
    3.6*V);

for i = 1:nCases

    fprintf(fid, ...
        'CASE %d - %s\n', ...
        i, ...
        caseNames{i});

    fprintf(fid, ...
        'Peak heave = %.4f mm\n', ...
        PeakHeave_mm(i));

    fprintf(fid, ...
        'Peak pitch = %.5f deg\n', ...
        PeakPitch_deg(i));

    fprintf(fid, ...
        'RMS heave acceleration = %.5f m/s^2\n', ...
        RMSHeaveAccel_mps2(i));

    fprintf(fid, ...
        'RMS pitch acceleration = %.5f deg/s^2\n', ...
        RMSPitchAccel_degps2(i));

    fprintf(fid, ...
        'Settling time = %.4f s\n', ...
        SettlingTime_s(i));

    fprintf(fid, ...
        'Peak actuator force = %.2f N\n', ...
        PeakActuatorForce_N(i));

    fprintf(fid, ...
        'Mean RMS actuator force = %.2f N\n', ...
        MeanRMSActuatorForce_N(i));

    fprintf(fid, ...
        'Maximum saturation = %.4f %%\n', ...
        MaxSaturation_pct(i));

    fprintf(fid, ...
        'Stable = %d\n', ...
        ClosedLoopStable(i));

    fprintf(fid, ...
        'Peak heave change vs nominal = %.2f %%\n', ...
        heaveChange_pct(i));

    fprintf(fid, ...
        'Peak pitch change vs nominal = %.2f %%\n', ...
        pitchChange_pct(i));

    fprintf(fid, ...
        'RMS heave acceleration change = %.2f %%\n', ...
        heaveAccelChange_pct(i));

    fprintf(fid, ...
        'RMS pitch acceleration change = %.2f %%\n', ...
        pitchAccelChange_pct(i));

    fprintf(fid, ...
        'Peak actuator force change = %.2f %%\n\n', ...
        forceChange_pct(i));

end

fprintf(fid, ...
    '\nOVERALL RESULT\n');

fprintf(fid, ...
    '--------------\n');

fprintf(fid, ...
    '%s\n\n', ...
    robustnessResult);

fprintf(fid, ...
    'Worst peak heave: %s = %.4f mm\n', ...
    caseNames{idxWorstHeave}, ...
    worstHeave);

fprintf(fid, ...
    'Worst peak pitch: %s = %.5f deg\n', ...
    caseNames{idxWorstPitch}, ...
    worstPitch);

fprintf(fid, ...
    'Worst RMS heave acceleration: %s = %.4f m/s^2\n', ...
    caseNames{idxWorstHeaveAccel}, ...
    worstHeaveAccel);

fprintf(fid, ...
    'Worst RMS pitch acceleration: %s = %.4f deg/s^2\n', ...
    caseNames{idxWorstPitchAccel}, ...
    worstPitchAccel);

fprintf(fid, ...
    'Highest actuator demand: %s = %.2f N\n', ...
    caseNames{idxWorstForce}, ...
    worstForce);

fclose(fid);

%% =========================================================
% Plot 1 - Body Motion
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'LQR Robustness - Body Motion', ...
    'NumberTitle', ...
    'off');

subplot(2,1,1)

hold on

for i = 1:nCases

    plot( ...
        timeHistory{i}, ...
        heaveHistory{i}, ...
        'LineWidth', ...
        1.1);

end

grid on

xlabel('Time [s]')
ylabel('Heave [mm]')

title('LQR Robustness - Sprung Mass Heave')

legend( ...
    caseNames, ...
    'Location', ...
    'best');

subplot(2,1,2)

hold on

for i = 1:nCases

    plot( ...
        timeHistory{i}, ...
        pitchHistory{i}, ...
        'LineWidth', ...
        1.1);

end

grid on

xlabel('Time [s]')
ylabel('Pitch [deg]')

title('LQR Robustness - Pitch Response')

legend( ...
    caseNames, ...
    'Location', ...
    'best');

bodyPNG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_body_motion.png');

bodyFIG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_body_motion.fig');

exportgraphics( ...
    fig1, ...
    bodyPNG, ...
    'Resolution', ...
    300);

savefig(fig1, bodyFIG);

%% =========================================================
% Plot 2 - Accelerations
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'LQR Robustness - Acceleration', ...
    'NumberTitle', ...
    'off');

subplot(2,1,1)

hold on

for i = 1:nCases

    plot( ...
        timeHistory{i}, ...
        heaveAccelHistory{i}, ...
        'LineWidth', ...
        1.1);

end

grid on

xlabel('Time [s]')
ylabel('Heave Accel [m/s^2]')

title('LQR Robustness - Heave Acceleration')

legend( ...
    caseNames, ...
    'Location', ...
    'best');

subplot(2,1,2)

hold on

for i = 1:nCases

    plot( ...
        timeHistory{i}, ...
        pitchAccelHistory{i}, ...
        'LineWidth', ...
        1.1);

end

grid on

xlabel('Time [s]')
ylabel('Pitch Accel [deg/s^2]')

title('LQR Robustness - Pitch Acceleration')

legend( ...
    caseNames, ...
    'Location', ...
    'best');

accelPNG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_acceleration.png');

accelFIG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_acceleration.fig');

exportgraphics( ...
    fig2, ...
    accelPNG, ...
    'Resolution', ...
    300);

savefig(fig2, accelFIG);

%% =========================================================
% Plot 3 - Actuator Demand
% ==========================================================

fig3 = figure( ...
    'Name', ...
    'LQR Robustness - Actuator Demand', ...
    'NumberTitle', ...
    'off');

bar( ...
    categorical(caseNames), ...
    PeakActuatorForce_N);

hold on

yline( ...
    F_act_max, ...
    '--', ...
    'Actuator Limit');

grid on

xlabel('Parameter Case')
ylabel('Peak Actuator Force [N]')

title('LQR Robustness - Peak Actuator Demand')

xtickangle(20)

ylim([0 1.1*F_act_max]);

forcePNG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_actuator_force.png');

forceFIG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_actuator_force.fig');

exportgraphics( ...
    fig3, ...
    forcePNG, ...
    'Resolution', ...
    300);

savefig(fig3, forceFIG);

%% =========================================================
% Plot 4 - Normalized Summary
% ==========================================================

normalizedMetrics = [ ...
    PeakHeave_mm ./ PeakHeave_mm(1), ...
    PeakPitch_deg ./ PeakPitch_deg(1), ...
    RMSHeaveAccel_mps2 ./ RMSHeaveAccel_mps2(1), ...
    RMSPitchAccel_degps2 ./ RMSPitchAccel_degps2(1), ...
    SettlingTime_s ./ SettlingTime_s(1), ...
    PeakActuatorForce_N ./ PeakActuatorForce_N(1)];

fig4 = figure( ...
    'Name', ...
    'LQR Robustness - Normalized Summary', ...
    'NumberTitle', ...
    'off');

bar(normalizedMetrics);

grid on

xlabel('Parameter Case')
ylabel('Metric Relative to Nominal')

title('Final Tuned LQR - Parameter Robustness Summary')

xticks(1:nCases)

xticklabels(caseNames)

xtickangle(20)

legend( ...
    'Peak Heave', ...
    'Peak Pitch', ...
    'RMS Heave Accel', ...
    'RMS Pitch Accel', ...
    'Settling Time', ...
    'Peak Actuator Force', ...
    'Location', ...
    'best');

yline( ...
    1, ...
    '--', ...
    'Nominal');

summaryPNG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_normalized_summary.png');

summaryFIG = fullfile( ...
    plotsFolder, ...
    'lqr_robustness_normalized_summary.fig');

exportgraphics( ...
    fig4, ...
    summaryPNG, ...
    'Resolution', ...
    300);

savefig(fig4, summaryFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('====================================================')
disp('LQR ROBUSTNESS SWEEP SAVED SUCCESSFULLY')
disp('====================================================')

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', bodyPNG);
fprintf('%s\n', accelPNG);
fprintf('%s\n', forcePNG);
fprintf('%s\n', summaryPNG);

%% =========================================================
% Helper Function
% IMPORTANT:
% Keep local function at END of script.
% ==========================================================

function sig = getLoggedSignal(simOut, signalName)

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
