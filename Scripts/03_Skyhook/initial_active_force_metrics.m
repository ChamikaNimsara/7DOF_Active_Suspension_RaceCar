%% Initial Active Force Metrics

clc;

%% Locate Results Folder

scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
projectFolder = fileparts(fileparts(scriptFolder));
resultsFolder = fullfile(projectFolder, 'Results', 'Skyhook');

%% Extract force signals

FL = F_act_FL_out.Data;
FR = F_act_FR_out.Data;
RL = F_act_RL_out.Data;
RR = F_act_RR_out.Data;

%% Peak forces

peak_FL = max(abs(FL));
peak_FR = max(abs(FR));
peak_RL = max(abs(RL));
peak_RR = max(abs(RR));

%% RMS forces

rms_FL = rms(FL);
rms_FR = rms(FR);
rms_RL = rms(RL);
rms_RR = rms(RR);

%% Saturation check

sat_FL = 100 * mean(abs(FL) >= 0.999 * F_act_max);
sat_FR = 100 * mean(abs(FR) >= 0.999 * F_act_max);
sat_RL = 100 * mean(abs(RL) >= 0.999 * F_act_max);
sat_RR = 100 * mean(abs(RR) >= 0.999 * F_act_max);

%% Display

disp('==============================================')
disp('INITIAL SKYHOOK ACTUATOR FORCE CHECK')
disp('==============================================')

fprintf('Peak FL force = %.2f N\n', peak_FL);
fprintf('Peak FR force = %.2f N\n', peak_FR);
fprintf('Peak RL force = %.2f N\n', peak_RL);
fprintf('Peak RR force = %.2f N\n', peak_RR);

fprintf('\nRMS FL force = %.2f N\n', rms_FL);
fprintf('RMS FR force = %.2f N\n', rms_FR);
fprintf('RMS RL force = %.2f N\n', rms_RL);
fprintf('RMS RR force = %.2f N\n', rms_RR);

fprintf('\nFL saturation = %.3f %%\n', sat_FL);
fprintf('FR saturation = %.3f %%\n', sat_FR);
fprintf('RL saturation = %.3f %%\n', sat_RL);
fprintf('RR saturation = %.3f %%\n', sat_RR);

disp('==============================================')

%% Save results table

Corner = {
    'FL'
    'FR'
    'RL'
    'RR'
    };

PeakForce_N = [
    peak_FL
    peak_FR
    peak_RL
    peak_RR
    ];

RMSForce_N = [
    rms_FL
    rms_FR
    rms_RL
    rms_RR
    ];

SaturationPercent = [
    sat_FL
    sat_FR
    sat_RL
    sat_RR
    ];

resultsTable = table( ...
    Corner, ...
    PeakForce_N, ...
    RMSForce_N, ...
    SaturationPercent);

csvFile = fullfile( ...
    resultsFolder, ...
    'initial_skyhook_force_metrics.csv');

writetable(resultsTable, csvFile);

matFile = fullfile( ...
    resultsFolder, ...
    'initial_skyhook_force_metrics.mat');

save(matFile, ...
    'resultsTable', ...
    'peak_FL', ...
    'peak_FR', ...
    'peak_RL', ...
    'peak_RR', ...
    'rms_FL', ...
    'rms_FR', ...
    'rms_RL', ...
    'rms_RR', ...
    'sat_FL', ...
    'sat_FR', ...
    'sat_RL', ...
    'sat_RR');

disp('Force metrics saved successfully.');
