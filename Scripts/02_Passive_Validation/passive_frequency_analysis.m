%% Passive 7-DOF Frequency Sweep Analysis
% Identifies dominant body and wheel-hop frequencies

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

resultsFolder = fullfile(projectFolder, 'Results', 'Passive');
plotsFolder = fullfile(projectFolder, 'Plots', 'Passive');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end


%% =========================================================
% Extract Signals
% ==========================================================

t = z_s_out.Time;

z_body = squeeze(z_s_out.Data);
z_wheel = squeeze(z_u_FL_out.Data);
z_road = squeeze(road_sweep_out.Data);


%% =========================================================
% Sampling Information
% ==========================================================

dt = mean(diff(t));
Fs_analysis = 1/dt;

N = length(t);


%% =========================================================
% Remove Mean
% ==========================================================

z_body = z_body - mean(z_body);
z_wheel = z_wheel - mean(z_wheel);
z_road = z_road - mean(z_road);


%% =========================================================
% FFT
% ==========================================================

Y_body = fft(z_body);
Y_wheel = fft(z_wheel);
Y_road = fft(z_road);

P2_body = abs(Y_body/N);
P2_wheel = abs(Y_wheel/N);
P2_road = abs(Y_road/N);

P1_body = P2_body(1:floor(N/2)+1);
P1_wheel = P2_wheel(1:floor(N/2)+1);
P1_road = P2_road(1:floor(N/2)+1);

P1_body(2:end-1) = 2*P1_body(2:end-1);
P1_wheel(2:end-1) = 2*P1_wheel(2:end-1);
P1_road(2:end-1) = 2*P1_road(2:end-1);

f = Fs_analysis*(0:floor(N/2))/N;


%% =========================================================
% Limit Analysis to Sweep Range
% ==========================================================

freqMask = ...
    f >= sweep_f_start & ...
    f <= sweep_f_end;

f_valid = f(freqMask);

body_valid = P1_body(freqMask);
wheel_valid = P1_wheel(freqMask);
road_valid = P1_road(freqMask);


%% =========================================================
% Approximate Frequency Response Ratios
% ==========================================================

epsilon = 1e-12;

body_ratio = body_valid ./ ...
    max(road_valid, epsilon);

wheel_ratio = wheel_valid ./ ...
    max(road_valid, epsilon);


%% =========================================================
% Identify Dominant Frequencies
% ==========================================================

[body_peak_mag, body_idx] = max(body_ratio);

body_resonance_freq = ...
    f_valid(body_idx);


[wheel_peak_mag, wheel_idx] = max(wheel_ratio);

wheel_hop_freq = ...
    f_valid(wheel_idx);


%% =========================================================
% Symmetry Validation
% ==========================================================

theta_deg = rad2deg(theta_out.Data);
phi_deg = rad2deg(phi_out.Data);

max_abs_pitch_sweep = ...
    max(abs(theta_deg));

max_abs_roll_sweep = ...
    max(abs(phi_deg));


%% =========================================================
% Plot Frequency Response
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'Passive Frequency Response', ...
    'NumberTitle', ...
    'off');

subplot(2,1,1)

plot(f_valid, body_ratio, ...
    'LineWidth', 1.5);

grid on

xlabel('Frequency [Hz]')
ylabel('|Z_s / Z_r|')

title('Sprung Mass Heave Frequency Response')

xlim([sweep_f_start sweep_f_end])


subplot(2,1,2)

plot(f_valid, wheel_ratio, ...
    'LineWidth', 1.5);

grid on

xlabel('Frequency [Hz]')
ylabel('|Z_u / Z_r|')

title('Front Unsprung Mass Frequency Response')

xlim([sweep_f_start sweep_f_end])


sgtitle( ...
    'Passive 7-DOF Suspension Frequency Sweep')


%% =========================================================
% Save Frequency Response Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'passive_frequency_response.png');

figFile = fullfile( ...
    plotsFolder, ...
    'passive_frequency_response.fig');

exportgraphics( ...
    fig1, ...
    pngFile, ...
    'Resolution', 300);

savefig(fig1, figFile);


%% =========================================================
% Plot Time-Domain Sweep Response
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Passive Sweep Time Response', ...
    'NumberTitle', ...
    'off');

subplot(3,1,1)

plot(t, z_road, ...
    'LineWidth', 1);

grid on

xlabel('Time [s]')
ylabel('Road [m]')

title('Frequency Sweep Road Input')


subplot(3,1,2)

plot(t, z_body, ...
    'LineWidth', 1);

grid on

xlabel('Time [s]')
ylabel('Heave [m]')

title('Sprung Mass Response')


subplot(3,1,3)

plot(t, z_wheel, ...
    'LineWidth', 1);

grid on

xlabel('Time [s]')
ylabel('Wheel [m]')

title('Front Unsprung Mass Response')


sgtitle( ...
    'Passive Frequency Sweep Time Response')


%% =========================================================
% Save Time Response Plot
% ==========================================================

timePNG = fullfile( ...
    plotsFolder, ...
    'passive_frequency_sweep_time_response.png');

timeFIG = fullfile( ...
    plotsFolder, ...
    'passive_frequency_sweep_time_response.fig');

exportgraphics( ...
    fig2, ...
    timePNG, ...
    'Resolution', 300);

savefig(fig2, timeFIG);


%% =========================================================
% Display Results
% ==========================================================

disp('=================================================')
disp('PASSIVE FREQUENCY SWEEP RESULTS')
disp('=================================================')

fprintf( ...
    'Dominant body resonance = %.3f Hz\n', ...
    body_resonance_freq);

fprintf( ...
    'Dominant wheel response = %.3f Hz\n', ...
    wheel_hop_freq);

fprintf( ...
    'Body response ratio     = %.3f\n', ...
    body_peak_mag);

fprintf( ...
    'Wheel response ratio    = %.3f\n', ...
    wheel_peak_mag);

fprintf( ...
    'Maximum pitch           = %.8f deg\n', ...
    max_abs_pitch_sweep);

fprintf( ...
    'Maximum roll            = %.8f deg\n', ...
    max_abs_roll_sweep);

disp('=================================================')


%% =========================================================
% Save TXT Results
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'passive_frequency_results.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'PASSIVE 7-DOF FREQUENCY SWEEP RESULTS\n');

fprintf(fid, ...
    '======================================\n\n');

fprintf(fid, ...
    'Sweep range              = %.2f - %.2f Hz\n', ...
    sweep_f_start, sweep_f_end);

fprintf(fid, ...
    'Dominant body resonance  = %.3f Hz\n', ...
    body_resonance_freq);

fprintf(fid, ...
    'Dominant wheel response  = %.3f Hz\n', ...
    wheel_hop_freq);

fprintf(fid, ...
    'Body response ratio      = %.3f\n', ...
    body_peak_mag);

fprintf(fid, ...
    'Wheel response ratio     = %.3f\n', ...
    wheel_peak_mag);

fprintf(fid, ...
    'Maximum pitch            = %.8f deg\n', ...
    max_abs_pitch_sweep);

fprintf(fid, ...
    'Maximum roll             = %.8f deg\n', ...
    max_abs_roll_sweep);

fclose(fid);


%% =========================================================
% Save CSV Results
% ==========================================================

Metric = {
    'Dominant Body Resonance'
    'Dominant Wheel Response'
    'Body Response Ratio'
    'Wheel Response Ratio'
    'Maximum Pitch During Sweep'
    'Maximum Roll During Sweep'
    };

Value = [
    body_resonance_freq
    wheel_hop_freq
    body_peak_mag
    wheel_peak_mag
    max_abs_pitch_sweep
    max_abs_roll_sweep
    ];

Unit = {
    'Hz'
    'Hz'
    '-'
    '-'
    'deg'
    'deg'
    };

resultsTable = table( ...
    Metric, Value, Unit);

csvFile = fullfile( ...
    resultsFolder, ...
    'passive_frequency_results.csv');

writetable(resultsTable, csvFile);


%% =========================================================
% Save MATLAB Results
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'passive_frequency_results.mat');

save( ...
    matFile, ...
    'f_valid', ...
    'body_ratio', ...
    'wheel_ratio', ...
    'body_resonance_freq', ...
    'wheel_hop_freq', ...
    'body_peak_mag', ...
    'wheel_peak_mag', ...
    'max_abs_pitch_sweep', ...
    'max_abs_roll_sweep');


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Plots and results saved successfully.')

fprintf('\nPlots:\n%s\n%s\n', ...
    pngFile, timePNG);

fprintf('\nResults:\n%s\n%s\n%s\n', ...
    txtFile, csvFile, matFile);