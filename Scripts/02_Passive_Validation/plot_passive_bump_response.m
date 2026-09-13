%% Passive 7-DOF Bump Response
% Project:
% 7-DOF Active Suspension Modelling and Control for a Race Vehicle
%
% Passive suspension validation using a symmetric bump input.

clc;

%% =========================================================
% Locate Plots Folder
% ==========================================================

scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
projectFolder = fileparts(fileparts(scriptFolder));
plotsFolder = fullfile(projectFolder, 'Plots', 'Passive');

fprintf('Saving plots to:\n%s\n\n', plotsFolder);


%% =========================================================
% Figure 1 - Body Motion Response
% ==========================================================

fig1 = figure('Name', 'Passive Bump - Body Motion', ...
              'NumberTitle', 'off');

subplot(3,1,1)

plot(z_s_out.Time, z_s_out.Data, ...
    'LineWidth', 1.5);

grid on

xlabel('Time [s]')
ylabel('Heave [m]')

title('Sprung Mass Heave Response')


subplot(3,1,2)

plot(theta_out.Time, ...
    rad2deg(theta_out.Data), ...
    'LineWidth', 1.5);

grid on

xlabel('Time [s]')
ylabel('Pitch [deg]')

title('Sprung Mass Pitch Response')


subplot(3,1,3)

plot(phi_out.Time, ...
    rad2deg(phi_out.Data), ...
    'LineWidth', 1.5);

grid on

xlabel('Time [s]')
ylabel('Roll [deg]')

title('Sprung Mass Roll Response')


sgtitle('Passive 7-DOF Vehicle Response to Symmetric Bump')


%% Save Figure 1

motionPNG = fullfile(plotsFolder, ...
    'passive_bump_body_motion.png');

motionFIG = fullfile(plotsFolder, ...
    'passive_bump_body_motion.fig');

exportgraphics(fig1, motionPNG, ...
    'Resolution', 300);

savefig(fig1, motionFIG);


%% =========================================================
% Figure 2 - Body Acceleration Response
% ==========================================================

fig2 = figure('Name', 'Passive Bump - Body Acceleration', ...
              'NumberTitle', 'off');


subplot(3,1,1)

plot(z_s_ddot_out.Time, ...
    z_s_ddot_out.Data, ...
    'LineWidth', 1.5);

grid on

xlabel('Time [s]')
ylabel('Heave Accel [m/s^2]')

title('Sprung Mass Heave Acceleration')


subplot(3,1,2)

plot(theta_ddot_out.Time, ...
    rad2deg(theta_ddot_out.Data), ...
    'LineWidth', 1.5);

grid on

xlabel('Time [s]')
ylabel('Pitch Accel [deg/s^2]')

title('Sprung Mass Pitch Acceleration')


subplot(3,1,3)

plot(phi_ddot_out.Time, ...
    rad2deg(phi_ddot_out.Data), ...
    'LineWidth', 1.5);

grid on

xlabel('Time [s]')
ylabel('Roll Accel [deg/s^2]')

title('Sprung Mass Roll Acceleration')


sgtitle('Passive 7-DOF Vehicle Acceleration Response')


%% Save Figure 2

accelPNG = fullfile(plotsFolder, ...
    'passive_bump_body_acceleration.png');

accelFIG = fullfile(plotsFolder, ...
    'passive_bump_body_acceleration.fig');

exportgraphics(fig2, accelPNG, ...
    'Resolution', 300);

savefig(fig2, accelFIG);


%% =========================================================
% Display Confirmation
% ==========================================================

disp('==============================================')
disp('Passive bump response plots saved.')
disp('==============================================')

fprintf('\nBody Motion PNG:\n%s\n', motionPNG);
fprintf('\nBody Motion FIG:\n%s\n', motionFIG);

fprintf('\nBody Acceleration PNG:\n%s\n', accelPNG);
fprintf('\nBody Acceleration FIG:\n%s\n', accelFIG);

disp('==============================================')
