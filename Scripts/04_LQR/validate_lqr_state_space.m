%% LQR State-Space Validation
% Checks controllability, stability and modal frequencies
% of the 14-state 7-DOF vehicle model.

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
% Build State-Space Model
% ==========================================================

currentFolder = pwd;

cd(projectFolder);

run(fullfile(projectFolder, 'Scripts', '01_Parameters', ...
    'build_lqr_state_space.m'));

cd(currentFolder);


%% =========================================================
% Controllability
% ==========================================================

Co = ctrb(A_lqr, B_lqr);

controllability_rank = rank(Co);

is_controllable = ...
    controllability_rank == n_states;


%% =========================================================
% Open-Loop Eigenvalues
% ==========================================================

open_loop_poles = eig(A_lqr);

real_parts = real(open_loop_poles);
imag_parts = imag(open_loop_poles);

is_open_loop_stable = ...
    all(real_parts < 0);


%% =========================================================
% Natural Frequencies and Damping Ratios
% ==========================================================

wn_rad_s = abs(open_loop_poles);

freq_hz = ...
    wn_rad_s / (2*pi);

damping_ratio = zeros(size(open_loop_poles));

nonzeroMask = wn_rad_s > 1e-10;

damping_ratio(nonzeroMask) = ...
    -real_parts(nonzeroMask) ./ ...
    wn_rad_s(nonzeroMask);


%% =========================================================
% Keep Positive-Imaginary Modes Only
% Avoid listing conjugate pairs twice
% ==========================================================

modeMask = imag_parts > 1e-6;

modal_poles = ...
    open_loop_poles(modeMask);

modal_freq_hz = ...
    freq_hz(modeMask);

modal_damping = ...
    damping_ratio(modeMask);


%% =========================================================
% Sort Modes by Frequency
% ==========================================================

[modal_freq_hz, sortIdx] = ...
    sort(modal_freq_hz);

modal_poles = ...
    modal_poles(sortIdx);

modal_damping = ...
    modal_damping(sortIdx);


%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('7-DOF LQR STATE-SPACE VALIDATION')
disp('====================================================')

fprintf( ...
    'Number of states          = %d\n', ...
    n_states);

fprintf( ...
    'Controllability rank      = %d\n', ...
    controllability_rank);

if is_controllable

    disp('Controllability result    = FULLY CONTROLLABLE')

else

    disp('Controllability result    = NOT FULLY CONTROLLABLE')

end


fprintf('\nOpen-loop stability:\n');

if is_open_loop_stable

    disp('Passive state-space model = STABLE')

else

    disp('Passive state-space model = NOT STRICTLY STABLE')

end


disp(' ')
disp('OPEN-LOOP MODES')
disp('----------------------------------------------------')

for i = 1:length(modal_freq_hz)

    fprintf( ...
        ['Mode %d: %.4f Hz   ' ...
         'zeta = %.4f   ' ...
         'pole = %.4f %+.4fj\n'], ...
        i, ...
        modal_freq_hz(i), ...
        modal_damping(i), ...
        real(modal_poles(i)), ...
        imag(modal_poles(i)));

end

disp('====================================================')


%% =========================================================
% Create Modal Results Table
% ==========================================================

Mode = (1:length(modal_freq_hz))';

Frequency_Hz = modal_freq_hz(:);

DampingRatio = modal_damping(:);

PoleReal = real(modal_poles(:));

PoleImag = imag(modal_poles(:));

modalTable = table( ...
    Mode, ...
    Frequency_Hz, ...
    DampingRatio, ...
    PoleReal, ...
    PoleImag);


%% =========================================================
% Save Modal Results CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'lqr_open_loop_modes.csv');

writetable(modalTable, csvFile);


%% =========================================================
% Save TXT Results
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'lqr_state_space_validation.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    '7-DOF LQR STATE-SPACE VALIDATION\n');

fprintf(fid, ...
    '================================\n\n');

fprintf(fid, ...
    'Number of states = %d\n', ...
    n_states);

fprintf(fid, ...
    'Controllability rank = %d\n', ...
    controllability_rank);

fprintf(fid, ...
    'Fully controllable = %d\n\n', ...
    is_controllable);

fprintf(fid, ...
    'Open-loop stable = %d\n\n', ...
    is_open_loop_stable);

fprintf(fid, ...
    'OPEN-LOOP MODES\n');

fprintf(fid, ...
    '---------------\n');

for i = 1:height(modalTable)

    fprintf(fid, ...
        ['Mode %d\n' ...
         'Frequency = %.6f Hz\n' ...
         'Damping ratio = %.6f\n' ...
         'Pole = %.6f %+.6fj\n\n'], ...
        modalTable.Mode(i), ...
        modalTable.Frequency_Hz(i), ...
        modalTable.DampingRatio(i), ...
        modalTable.PoleReal(i), ...
        modalTable.PoleImag(i));

end

fclose(fid);


%% =========================================================
% Save MAT Results
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'lqr_state_space_validation.mat');

save(matFile, ...
    'Co', ...
    'controllability_rank', ...
    'is_controllable', ...
    'open_loop_poles', ...
    'is_open_loop_stable', ...
    'modalTable');


%% =========================================================
% Pole Map
% ==========================================================

fig1 = figure( ...
    'Name', ...
    '7-DOF Open-Loop Pole Map', ...
    'NumberTitle', ...
    'off');

plot( ...
    real(open_loop_poles), ...
    imag(open_loop_poles), ...
    'x', ...
    'MarkerSize', 9, ...
    'LineWidth', 1.5);

grid on

xlabel('Real Axis [1/s]')
ylabel('Imaginary Axis [rad/s]')

title('Passive 7-DOF Open-Loop Pole Map')

xline(0, '--');


polePNG = fullfile( ...
    plotsFolder, ...
    'lqr_open_loop_pole_map.png');

poleFIG = fullfile( ...
    plotsFolder, ...
    'lqr_open_loop_pole_map.fig');

exportgraphics(fig1, ...
    polePNG, ...
    'Resolution', 300);

savefig(fig1, poleFIG);


%% =========================================================
% Modal Frequency Plot
% ==========================================================

fig2 = figure( ...
    'Name', ...
    '7-DOF Modal Frequencies', ...
    'NumberTitle', ...
    'off');

bar( ...
    categorical(string(Mode)), ...
    Frequency_Hz);

grid on

xlabel('Mode')
ylabel('Natural Frequency [Hz]')

title('Passive 7-DOF Modal Frequencies');


freqPNG = fullfile( ...
    plotsFolder, ...
    'lqr_open_loop_modal_frequencies.png');

freqFIG = fullfile( ...
    plotsFolder, ...
    'lqr_open_loop_modal_frequencies.fig');

exportgraphics(fig2, ...
    freqPNG, ...
    'Resolution', 300);

savefig(fig2, freqFIG);


%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('State-space validation saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', polePNG);
fprintf('%s\n', freqPNG);
