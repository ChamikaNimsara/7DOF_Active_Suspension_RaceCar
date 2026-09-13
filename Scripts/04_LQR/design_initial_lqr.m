%% Initial LQR Controller Design
% 7-DOF Active Suspension Race Car
%
% Controller is designed in normalized coordinates to avoid
% numerical problems caused by differently scaled physical states.

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

resultsFolder = fullfile(projectFolder, 'Results', 'LQR');
plotsFolder = fullfile(projectFolder, 'Plots', 'LQR');

if ~exist(plotsFolder, 'dir')
    mkdir(plotsFolder);
end

%% =========================================================
% Build Physical State-Space Model
% ==========================================================

oldFolder = pwd;

cd(projectFolder);

run(fullfile(projectFolder, 'Scripts', '01_Parameters', ...
    'build_lqr_state_space.m'));

cd(oldFolder);

%% =========================================================
% State Scaling
% ==========================================================
%
% Representative maximum state magnitudes.
%
% State order:
%
%  1  z_s
%  2  theta
%  3  phi
%  4  z_u_FL
%  5  z_u_FR
%  6  z_u_RL
%  7  z_u_RR
%  8  z_s_dot
%  9  theta_dot
% 10  phi_dot
% 11  z_u_FL_dot
% 12  z_u_FR_dot
% 13  z_u_RL_dot
% 14  z_u_RR_dot

stateScale = [ ...
    0.010     % z_s [m]
    0.010     % theta [rad]
    0.010     % phi [rad]
    0.020     % z_u_FL [m]
    0.020     % z_u_FR [m]
    0.020     % z_u_RL [m]
    0.020     % z_u_RR [m]
    0.20      % z_s_dot [m/s]
    0.20      % theta_dot [rad/s]
    0.20      % phi_dot [rad/s]
    1.00      % z_u_FL_dot [m/s]
    1.00      % z_u_FR_dot [m/s]
    1.00      % z_u_RL_dot [m/s]
    1.00      % z_u_RR_dot [m/s]
    ];

Sx = diag(stateScale);

%% =========================================================
% Input Scaling
% ==========================================================

inputScale = F_act_max;

Su = inputScale * eye(4);

%% =========================================================
% Normalized State-Space Model
% ==========================================================

A_norm = ...
    Sx \ (A_lqr * Sx);

B_norm = ...
    Sx \ (B_lqr * Su);

%% =========================================================
% Initial LQR State Weights
% ==========================================================
%
% Higher value = controller cares more about that state.
%
% Body attitude is weighted strongly because previous Skyhook
% tests showed large improvements in pitch/roll were desirable.
%
% Unsprung states receive smaller weights to avoid excessive
% actuator effort / wheel control.

q_zs = 8;

q_theta = 20;
q_phi   = 20;

q_zu = 1;

q_zs_dot = 5;

q_theta_dot = 8;
q_phi_dot   = 8;

q_zu_dot = 0.5;

Q_lqr_norm = diag([ ...
    q_zs ...
    q_theta ...
    q_phi ...
    q_zu ...
    q_zu ...
    q_zu ...
    q_zu ...
    q_zs_dot ...
    q_theta_dot ...
    q_phi_dot ...
    q_zu_dot ...
    q_zu_dot ...
    q_zu_dot ...
    q_zu_dot]);

%% =========================================================
% Control-Effort Weight
% ==========================================================
%
% Because actuator force has already been normalized by
% F_act_max, R = I gives a sensible initial penalty.

R_lqr_norm = eye(4);

%% =========================================================
% Solve LQR
% ==========================================================

[K_norm, S_lqr, closedLoopPoles] = ...
    lqr(A_norm, B_norm, Q_lqr_norm, R_lqr_norm);

%% =========================================================
% Convert Gain Back to Physical Coordinates
% ==========================================================
%
% u_bar = -K_norm*x_bar
%
% u     = Su*u_bar
% x_bar = inv(Sx)*x
%
% therefore:
%
% u = -Su*K_norm*inv(Sx)*x

K_lqr = ...
    Su * (K_norm / Sx);

%% =========================================================
% Physical Closed-Loop Matrix
% ==========================================================

A_cl = ...
    A_lqr - B_lqr*K_lqr;

physicalClosedLoopPoles = ...
    eig(A_cl);

maxClosedLoopRealPart = ...
    max(real(physicalClosedLoopPoles));

isClosedLoopStable = ...
    maxClosedLoopRealPart < 0;

%% =========================================================
% Gain Dimensions
% ==========================================================

[numInputsK, numStatesK] = ...
    size(K_lqr);

%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('INITIAL LQR CONTROLLER DESIGN')
disp('====================================================')

fprintf( ...
    'LQR gain size             = %d x %d\n', ...
    numInputsK, ...
    numStatesK);

fprintf( ...
    'Maximum actuator scale    = %.0f N\n', ...
    inputScale);

fprintf( ...
    'Max closed-loop pole real = %.6f 1/s\n', ...
    maxClosedLoopRealPart);

if isClosedLoopStable

    disp('Closed-loop result        = STABLE')

else

    disp('Closed-loop result        = UNSTABLE')

end

disp(' ')
disp('Physical LQR Gain Matrix K_lqr:')
disp(K_lqr)

disp('====================================================')

%% =========================================================
% Closed-Loop Modal Information
% ==========================================================

poleReal = ...
    real(physicalClosedLoopPoles);

poleImag = ...
    imag(physicalClosedLoopPoles);

wn = ...
    abs(physicalClosedLoopPoles);

frequencyHz = ...
    wn/(2*pi);

dampingRatio = zeros(size(wn));

valid = wn > 1e-10;

dampingRatio(valid) = ...
    -poleReal(valid)./wn(valid);

%% Keep positive imaginary modes only

modeMask = ...
    poleImag > 1e-6;

modalPoles = ...
    physicalClosedLoopPoles(modeMask);

modalFrequencyHz = ...
    frequencyHz(modeMask);

modalDamping = ...
    dampingRatio(modeMask);

[modalFrequencyHz, order] = ...
    sort(modalFrequencyHz);

modalPoles = ...
    modalPoles(order);

modalDamping = ...
    modalDamping(order);

Mode = ...
    (1:length(modalFrequencyHz))';

Frequency_Hz = ...
    modalFrequencyHz(:);

DampingRatio = ...
    modalDamping(:);

PoleReal = ...
    real(modalPoles(:));

PoleImag = ...
    imag(modalPoles(:));

modalTable = table( ...
    Mode, ...
    Frequency_Hz, ...
    DampingRatio, ...
    PoleReal, ...
    PoleImag);

%% =========================================================
% Save Controller Data
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_controller.mat');

save(matFile, ...
    'K_lqr', ...
    'K_norm', ...
    'Q_lqr_norm', ...
    'R_lqr_norm', ...
    'Sx', ...
    'Su', ...
    'A_norm', ...
    'B_norm', ...
    'A_cl', ...
    'physicalClosedLoopPoles', ...
    'modalTable', ...
    'isClosedLoopStable');

%% =========================================================
% Save Gain Matrix CSV
% ==========================================================

stateNames = { ...
    'z_s', ...
    'theta', ...
    'phi', ...
    'z_u_FL', ...
    'z_u_FR', ...
    'z_u_RL', ...
    'z_u_RR', ...
    'z_s_dot', ...
    'theta_dot', ...
    'phi_dot', ...
    'z_u_FL_dot', ...
    'z_u_FR_dot', ...
    'z_u_RL_dot', ...
    'z_u_RR_dot'};

gainTable = array2table( ...
    K_lqr, ...
    'VariableNames', ...
    stateNames);

gainTable.Properties.RowNames = { ...
    'F_FL', ...
    'F_FR', ...
    'F_RL', ...
    'F_RR'};

csvFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_gain_matrix.csv');

writetable( ...
    gainTable, ...
    csvFile, ...
    'WriteRowNames', true);

%% =========================================================
% Save Closed-Loop Modes CSV
% ==========================================================

modeCSV = fullfile( ...
    resultsFolder, ...
    'initial_lqr_closed_loop_modes.csv');

writetable(modalTable, modeCSV);

%% =========================================================
% Save TXT Summary
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'initial_lqr_controller.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'INITIAL LQR CONTROLLER DESIGN\n');

fprintf(fid, ...
    '=============================\n\n');

fprintf(fid, ...
    'Controller gain size = %d x %d\n', ...
    numInputsK, ...
    numStatesK);

fprintf(fid, ...
    'Input scaling = %.0f N\n', ...
    inputScale);

fprintf(fid, ...
    'Maximum closed-loop pole real part = %.8f 1/s\n', ...
    maxClosedLoopRealPart);

fprintf(fid, ...
    'Closed-loop stable = %d\n\n', ...
    isClosedLoopStable);

fprintf(fid, ...
    'NORMALIZED Q DIAGONAL\n');

fprintf(fid, ...
    '---------------------\n');

qdiag = diag(Q_lqr_norm);

for i = 1:length(qdiag)

    fprintf(fid, ...
        'Q(%d) = %.4f\n', ...
        i, ...
        qdiag(i));

end

fprintf(fid, ...
    '\nPHYSICAL LQR GAIN MATRIX\n');

fprintf(fid, ...
    '------------------------\n');

for r = 1:size(K_lqr,1)

    fprintf(fid, ...
        'Actuator %d:\n', ...
        r);

    for c = 1:size(K_lqr,2)

        fprintf(fid, ...
            '  K(%d,%d) = %.8f\n', ...
            r, ...
            c, ...
            K_lqr(r,c));

    end

end

fclose(fid);

%% =========================================================
% Plot Open vs Closed Loop Poles
% ==========================================================

openPoles = ...
    eig(A_lqr);

fig1 = figure( ...
    'Name', ...
    'Open vs Closed Loop LQR Poles', ...
    'NumberTitle', ...
    'off');

plot( ...
    real(openPoles), ...
    imag(openPoles), ...
    'x', ...
    'MarkerSize', 9, ...
    'LineWidth', 1.4);

hold on

plot( ...
    real(physicalClosedLoopPoles), ...
    imag(physicalClosedLoopPoles), ...
    'o', ...
    'MarkerSize', 7, ...
    'LineWidth', 1.4);

xline(0, '--');

grid on

xlabel('Real Axis [1/s]')
ylabel('Imaginary Axis [rad/s]')

title('Open-Loop vs Initial LQR Closed-Loop Poles')

legend( ...
    'Passive / Open Loop', ...
    'LQR Closed Loop', ...
    'Location', ...
    'best');

polePNG = fullfile( ...
    plotsFolder, ...
    'initial_lqr_open_vs_closed_poles.png');

poleFIG = fullfile( ...
    plotsFolder, ...
    'initial_lqr_open_vs_closed_poles.fig');

exportgraphics( ...
    fig1, ...
    polePNG, ...
    'Resolution', 300);

savefig(fig1, poleFIG);

%% =========================================================
% Plot Closed-Loop Damping Ratios
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Initial LQR Modal Damping', ...
    'NumberTitle', ...
    'off');

bar( ...
    categorical(string(Mode)), ...
    DampingRatio);

grid on

xlabel('Mode')
ylabel('Damping Ratio')

title('Initial LQR Closed-Loop Modal Damping');

dampingPNG = fullfile( ...
    plotsFolder, ...
    'initial_lqr_modal_damping.png');

dampingFIG = fullfile( ...
    plotsFolder, ...
    'initial_lqr_modal_damping.fig');

exportgraphics( ...
    fig2, ...
    dampingPNG, ...
    'Resolution', 300);

savefig(fig2, dampingFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Initial LQR controller saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', modeCSV);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', polePNG);
fprintf('%s\n', dampingPNG);
