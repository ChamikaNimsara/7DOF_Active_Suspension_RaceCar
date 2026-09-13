%% Initial Active Suspension Force Check

clc;

%% Locate Plots Folder

scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
projectFolder = fileparts(fileparts(scriptFolder));
plotsFolder = fullfile(projectFolder, 'Plots', 'Skyhook');

%% Plot actuator forces

fig = figure( ...
    'Name', 'Initial Active Suspension Forces', ...
    'NumberTitle', 'off');

plot(F_act_FL_out.Time, F_act_FL_out.Data, ...
    'LineWidth', 1.3);

hold on

plot(F_act_FR_out.Time, F_act_FR_out.Data, ...
    'LineWidth', 1.3);

plot(F_act_RL_out.Time, F_act_RL_out.Data, ...
    'LineWidth', 1.3);

plot(F_act_RR_out.Time, F_act_RR_out.Data, ...
    'LineWidth', 1.3);

grid on

xlabel('Time [s]')
ylabel('Actuator Force [N]')

title('Initial Skyhook Actuator Forces')

legend( ...
    'FL', ...
    'FR', ...
    'RL', ...
    'RR', ...
    'Location', 'best');

yline(F_act_max, '--', 'HandleVisibility', 'off');
yline(-F_act_max, '--', 'HandleVisibility', 'off');

%% Save plot

pngFile = fullfile( ...
    plotsFolder, ...
    'initial_skyhook_actuator_forces.png');

figFile = fullfile( ...
    plotsFolder, ...
    'initial_skyhook_actuator_forces.fig');

exportgraphics( ...
    fig, ...
    pngFile, ...
    'Resolution', 300);

savefig(fig, figFile);

disp('Initial actuator force plot saved.');
