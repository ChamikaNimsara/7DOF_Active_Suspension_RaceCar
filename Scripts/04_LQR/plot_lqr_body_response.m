%% LQR Body Motion Sanity Check

clc;

%% =========================================================
% Locate Plots Folder
% ==========================================================

scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
projectFolder = fileparts(fileparts(scriptFolder));
plotsFolder = fullfile(projectFolder, 'Plots', 'LQR');

%% =========================================================
% Extract Logged Signals
% ==========================================================

t = z_s_out.Time;

z_s_mm = 1000 * z_s_out.Data;

theta_deg = rad2deg(theta_out.Data);

phi_deg = rad2deg(phi_out.Data);

%% =========================================================
% Plot Body Motion
% ==========================================================

fig = figure( ...
    'Name', ...
    'Initial LQR Body Response', ...
    'NumberTitle', ...
    'off');

subplot(3,1,1)

plot(t, z_s_mm, ...
    'LineWidth', 1.4);

grid on
xlabel('Time [s]')
ylabel('Heave [mm]')
title('LQR Sprung-Mass Heave Response')

subplot(3,1,2)

plot(t, theta_deg, ...
    'LineWidth', 1.4);

grid on
xlabel('Time [s]')
ylabel('Pitch [deg]')
title('LQR Pitch Response')

subplot(3,1,3)

plot(t, phi_deg, ...
    'LineWidth', 1.4);

grid on
xlabel('Time [s]')
ylabel('Roll [deg]')
title('LQR Roll Response')

sgtitle('Initial LQR Symmetric-Bump Response')

%% =========================================================
% Save Plot
% ==========================================================

pngFile = fullfile( ...
    plotsFolder, ...
    'initial_lqr_body_response.png');

figFile = fullfile( ...
    plotsFolder, ...
    'initial_lqr_body_response.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', ...
    300);

savefig(fig, figFile);

disp('Initial LQR body-response plot saved.');

fprintf('\nPlot:\n%s\n', pngFile);
