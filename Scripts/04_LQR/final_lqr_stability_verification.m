%% Final Tuned LQR - Closed-Loop Stability Verification
% Step 11C
%
% Verifies:
% 1. Nominal closed-loop stability
% 2. Closed-loop pole locations
% 3. Modal natural frequencies and damping ratios
% 4. Stability margin
% 5. Open-loop vs closed-loop pole comparison
% 6. Closed-loop stability under parameter uncertainty
%
% All results and plots are automatically saved.

clear;
clc;
close all;

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

resultsFolder = fullfile(projectFolder, 'Results', 'LQR');
plotsFolder = fullfile(projectFolder, 'Plots', 'LQR');
robustnessPlotsFolder = fullfile(projectFolder, 'Plots', 'Robustness');
finalPlotsFolder = fullfile(projectFolder, 'Plots', 'Final');

if ~exist(plotsFolder,'dir')
    mkdir(plotsFolder);
end

%% =========================================================
% Load Final Tuned LQR Controller
% ==========================================================

controllerFile = fullfile( ...
    resultsFolder, ...
    'final_tuned_lqr_controller.mat');

if ~isfile(controllerFile)

    error( ...
        'Final tuned LQR controller file not found: %s', ...
        controllerFile);

end

controllerData = load(controllerFile);

K_lqr = controllerData.K_lqr_final;

%% =========================================================
% Build Nominal State-Space Model
% ==========================================================

run(fullfile(projectFolder, 'Scripts', '01_Parameters', ...
    'build_lqr_state_space.m'));

%% =========================================================
% Closed-Loop Matrix
% ==========================================================

A_cl = ...
    A_lqr - B_lqr*K_lqr;

%% =========================================================
% Open-Loop and Closed-Loop Poles
% ==========================================================

openLoopPoles = ...
    eig(A_lqr);

closedLoopPoles = ...
    eig(A_cl);

%% =========================================================
% Stability Check
% ==========================================================

openLoopStable = ...
    all(real(openLoopPoles) < 0);

closedLoopStable = ...
    all(real(closedLoopPoles) < 0);

maxClosedLoopReal = ...
    max(real(closedLoopPoles));

stabilityMargin = ...
    -maxClosedLoopReal;

%% =========================================================
% Closed-Loop Modal Data
% ==========================================================

[wn,zeta] = damp(A_cl);

freq_Hz = ...
    wn/(2*pi);

%% =========================================================
% Sort Modes by Frequency
% ==========================================================

[sortedFreq,sortIdx] = ...
    sort(freq_Hz);

sortedZeta = ...
    zeta(sortIdx);

sortedPoles = ...
    closedLoopPoles(sortIdx);

%% =========================================================
% Pole Data Table
% ==========================================================

poleIndex = ...
    (1:length(closedLoopPoles))';

poleReal = ...
    real(closedLoopPoles);

poleImag = ...
    imag(closedLoopPoles);

poleMagnitude = ...
    abs(closedLoopPoles);

poleTable = table( ...
    poleIndex, ...
    poleReal, ...
    poleImag, ...
    poleMagnitude, ...
    'VariableNames',{ ...
    'PoleIndex', ...
    'RealPart_1ps', ...
    'ImagPart_radps', ...
    'Magnitude'});

%% =========================================================
% Modal Data Table
% ==========================================================

modeIndex = ...
    (1:length(sortedFreq))';

modalTable = table( ...
    modeIndex, ...
    sortedFreq, ...
    sortedZeta, ...
    real(sortedPoles), ...
    imag(sortedPoles), ...
    'VariableNames',{ ...
    'Mode', ...
    'NaturalFrequency_Hz', ...
    'DampingRatio', ...
    'PoleRealPart_1ps', ...
    'PoleImagPart_radps'});

%% =========================================================
% Parameter-Uncertainty Stability Verification
% ==========================================================

disp('====================================================')
disp('FINAL LQR CLOSED-LOOP STABILITY VERIFICATION')
disp('====================================================')

fprintf('Nominal open-loop stable  = %d\n',openLoopStable);
fprintf('Nominal closed-loop stable = %d\n',closedLoopStable);
fprintf('Maximum closed-loop pole real part = %.6f 1/s\n', ...
    maxClosedLoopReal);
fprintf('Closed-loop stability margin = %.6f 1/s\n', ...
    stabilityMargin);

%% =========================================================
% Store Nominal Parameters
% ==========================================================

nominal.m_s = m_s;

nominal.k_sf = k_sf;
nominal.k_sr = k_sr;

nominal.c_sf = c_sf;
nominal.c_sr = c_sr;

nominal.k_t = k_t;

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

caseStable = false(nCases,1);

caseMaxReal = zeros(nCases,1);

caseMargin = zeros(nCases,1);

caseMinDamping = zeros(nCases,1);

caseMaxFrequency = zeros(nCases,1);

%% =========================================================
% Loop Through Parameter Cases
% ==========================================================

for k = 1:nCases

    %% -----------------------------------------------------
    % Restore Nominal
    % ------------------------------------------------------

    m_s = nominal.m_s;

    k_sf = nominal.k_sf;
    k_sr = nominal.k_sr;

    c_sf = nominal.c_sf;
    c_sr = nominal.c_sr;

    k_t = nominal.k_t;

    %% -----------------------------------------------------
    % Apply Uncertainty
    % ------------------------------------------------------

    switch k

        case 1

            % Nominal

        case 2

            m_s = ...
                1.10 * nominal.m_s;

        case 3

            k_sf = ...
                0.90 * nominal.k_sf;

            k_sr = ...
                0.90 * nominal.k_sr;

        case 4

            c_sf = ...
                0.90 * nominal.c_sf;

            c_sr = ...
                0.90 * nominal.c_sr;

        case 5

            k_t = ...
                0.90 * nominal.k_t;

    end

    %% -----------------------------------------------------
    % Rebuild State-Space Model for Modified Parameters
    % ------------------------------------------------------

    run(fullfile(projectFolder, 'Scripts', '01_Parameters', ...
        'build_lqr_state_space.m'));

    %% -----------------------------------------------------
    % Closed Loop for Case
    % ------------------------------------------------------

    A_cl_case = ...
        A_lqr - B_lqr*K_lqr;

    polesCase = ...
        eig(A_cl_case);

    %% -----------------------------------------------------
    % Stability Metrics
    % ------------------------------------------------------

    caseStable(k) = ...
        all(real(polesCase) < 0);

    caseMaxReal(k) = ...
        max(real(polesCase));

    caseMargin(k) = ...
        -caseMaxReal(k);

    [wnCase,zetaCase] = ...
        damp(A_cl_case);

    caseMinDamping(k) = ...
        min(zetaCase);

    caseMaxFrequency(k) = ...
        max(wnCase/(2*pi));

    %% -----------------------------------------------------
    % Console
    % ------------------------------------------------------

    fprintf( ...
        '\nCase %d/%d: %s\n', ...
        k, ...
        nCases, ...
        caseNames{k});

    fprintf( ...
        'Stable = %d | Max real pole = %.6f 1/s | Margin = %.6f 1/s | Min zeta = %.4f\n', ...
        caseStable(k), ...
        caseMaxReal(k), ...
        caseMargin(k), ...
        caseMinDamping(k));

end

%% =========================================================
% Restore Nominal Parameters
% ==========================================================

m_s = nominal.m_s;

k_sf = nominal.k_sf;
k_sr = nominal.k_sr;

c_sf = nominal.c_sf;
c_sr = nominal.c_sr;

k_t = nominal.k_t;

run(fullfile(projectFolder, 'Scripts', '01_Parameters', ...
    'build_lqr_state_space.m'));

%% =========================================================
% Stability Robustness Table
% ==========================================================

stabilityTable = table( ...
    caseNames, ...
    caseStable, ...
    caseMaxReal, ...
    caseMargin, ...
    caseMinDamping, ...
    caseMaxFrequency, ...
    'VariableNames',{ ...
    'Case', ...
    'Stable', ...
    'MaximumPoleRealPart_1ps', ...
    'StabilityMargin_1ps', ...
    'MinimumDampingRatio', ...
    'MaximumNaturalFrequency_Hz'});

%% =========================================================
% Identify Worst Stability Case
% ==========================================================

[worstMaxReal,worstIdx] = ...
    max(caseMaxReal);

worstCaseName = ...
    caseNames{worstIdx};

worstMargin = ...
    caseMargin(worstIdx);

allCasesStable = ...
    all(caseStable);

%% =========================================================
% Display Tables
% ==========================================================

disp(' ')
disp('====================================================')
disp('CLOSED-LOOP POLES')
disp('====================================================')

disp(poleTable)

disp(' ')
disp('====================================================')
disp('CLOSED-LOOP MODAL DATA')
disp('====================================================')

disp(modalTable)

disp(' ')
disp('====================================================')
disp('PARAMETER-UNCERTAINTY STABILITY')
disp('====================================================')

disp(stabilityTable)

fprintf( ...
    '\nWorst stability case = %s\n', ...
    worstCaseName);

fprintf( ...
    'Worst-case maximum pole real part = %.6f 1/s\n', ...
    worstMaxReal);

fprintf( ...
    'Worst-case stability margin = %.6f 1/s\n', ...
    worstMargin);

if allCasesStable

    disp('RESULT: PASS')
    disp('All tested parameter-variation cases remain closed-loop stable.')

else

    disp('RESULT: FAIL')
    disp('At least one parameter-variation case is unstable.')

end

%% =========================================================
% Save Pole CSV
% ==========================================================

poleCSV = fullfile( ...
    resultsFolder, ...
    'final_lqr_closed_loop_poles.csv');

writetable( ...
    poleTable, ...
    poleCSV);

%% =========================================================
% Save Modal CSV
% ==========================================================

modalCSV = fullfile( ...
    resultsFolder, ...
    'final_lqr_closed_loop_modes.csv');

writetable( ...
    modalTable, ...
    modalCSV);

%% =========================================================
% Save Robustness CSV
% ==========================================================

stabilityCSV = fullfile( ...
    resultsFolder, ...
    'final_lqr_stability_robustness.csv');

writetable( ...
    stabilityTable, ...
    stabilityCSV);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'final_lqr_stability_verification.mat');

save( ...
    matFile, ...
    'K_lqr', ...
    'A_cl', ...
    'openLoopPoles', ...
    'closedLoopPoles', ...
    'openLoopStable', ...
    'closedLoopStable', ...
    'maxClosedLoopReal', ...
    'stabilityMargin', ...
    'wn', ...
    'zeta', ...
    'freq_Hz', ...
    'poleTable', ...
    'modalTable', ...
    'stabilityTable', ...
    'caseStable', ...
    'caseMaxReal', ...
    'caseMargin', ...
    'caseMinDamping', ...
    'caseMaxFrequency', ...
    'allCasesStable', ...
    'worstCaseName', ...
    'worstMargin');

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'final_lqr_stability_verification.txt');

fid = fopen(txtFile,'w');

if fid == -1
    error('Unable to create stability verification TXT file.');
end

fprintf(fid, ...
    'FINAL TUNED LQR CLOSED-LOOP STABILITY VERIFICATION\n');

fprintf(fid, ...
    '==================================================\n\n');

fprintf(fid, ...
    'Nominal open-loop stable   = %d\n', ...
    openLoopStable);

fprintf(fid, ...
    'Nominal closed-loop stable = %d\n', ...
    closedLoopStable);

fprintf(fid, ...
    'Maximum closed-loop pole real part = %.8f 1/s\n', ...
    maxClosedLoopReal);

fprintf(fid, ...
    'Nominal stability margin = %.8f 1/s\n\n', ...
    stabilityMargin);

fprintf(fid, ...
    'CLOSED-LOOP POLES\n');

fprintf(fid, ...
    '--------------------------------------------------\n');

for i = 1:length(closedLoopPoles)

    fprintf(fid, ...
        'Pole %2d = % .8f %+.8fj\n', ...
        i, ...
        real(closedLoopPoles(i)), ...
        imag(closedLoopPoles(i)));

end

fprintf(fid, ...
    '\nCLOSED-LOOP MODAL DATA\n');

fprintf(fid, ...
    '--------------------------------------------------\n');

for i = 1:length(sortedFreq)

    fprintf(fid, ...
        ['Mode %2d | f_n = %.5f Hz | ' ...
         'zeta = %.5f | pole = % .8f %+.8fj\n'], ...
        i, ...
        sortedFreq(i), ...
        sortedZeta(i), ...
        real(sortedPoles(i)), ...
        imag(sortedPoles(i)));

end

fprintf(fid, ...
    '\nPARAMETER-UNCERTAINTY STABILITY\n');

fprintf(fid, ...
    '--------------------------------------------------\n');

for k = 1:nCases

    fprintf(fid, ...
        ['%s | Stable = %d | ' ...
         'Max real pole = %.8f 1/s | ' ...
         'Margin = %.8f 1/s | ' ...
         'Min zeta = %.5f\n'], ...
        caseNames{k}, ...
        caseStable(k), ...
        caseMaxReal(k), ...
        caseMargin(k), ...
        caseMinDamping(k));

end

fprintf(fid, ...
    '\nWorst stability case = %s\n', ...
    worstCaseName);

fprintf(fid, ...
    'Worst-case stability margin = %.8f 1/s\n', ...
    worstMargin);

fprintf(fid, ...
    'All tested cases stable = %d\n', ...
    allCasesStable);

fclose(fid);

%% =========================================================
% Plot 1 - Open Loop vs Closed Loop Pole Map
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'Final LQR Pole Comparison', ...
    'NumberTitle', ...
    'off');

plot( ...
    real(openLoopPoles), ...
    imag(openLoopPoles), ...
    'x', ...
    'LineWidth',1.5, ...
    'MarkerSize',8);

hold on

plot( ...
    real(closedLoopPoles), ...
    imag(closedLoopPoles), ...
    'o', ...
    'LineWidth',1.5, ...
    'MarkerSize',7);

xline(0,'--');

grid on

xlabel('Real Axis [1/s]')
ylabel('Imaginary Axis [rad/s]')

title('Open-Loop vs Final Tuned LQR Closed-Loop Poles')

legend( ...
    'Passive / Open Loop', ...
    'Final Tuned LQR', ...
    'Stability Boundary', ...
    'Location','best');

polePNG = fullfile( ...
    finalPlotsFolder, ...
    'final_lqr_open_vs_closed_poles.png');

poleFIG = fullfile( ...
    finalPlotsFolder, ...
    'final_lqr_open_vs_closed_poles.fig');

exportgraphics( ...
    fig1, ...
    polePNG, ...
    'Resolution',300);

savefig(fig1,poleFIG);

%% =========================================================
% Plot 2 - Closed-Loop Modal Damping
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Final LQR Modal Damping', ...
    'NumberTitle', ...
    'off');

bar(sortedZeta)

grid on

xlabel('Mode')
ylabel('Damping Ratio')

title('Final Tuned LQR Closed-Loop Modal Damping')

dampingPNG = fullfile( ...
    plotsFolder, ...
    'final_lqr_modal_damping.png');

dampingFIG = fullfile( ...
    plotsFolder, ...
    'final_lqr_modal_damping.fig');

exportgraphics( ...
    fig2, ...
    dampingPNG, ...
    'Resolution',300);

savefig(fig2,dampingFIG);

%% =========================================================
% Plot 3 - Closed-Loop Natural Frequencies
% ==========================================================

fig3 = figure( ...
    'Name', ...
    'Final LQR Natural Frequencies', ...
    'NumberTitle', ...
    'off');

bar(sortedFreq)

grid on

xlabel('Mode')
ylabel('Natural Frequency [Hz]')

title('Final Tuned LQR Closed-Loop Natural Frequencies')

frequencyPNG = fullfile( ...
    plotsFolder, ...
    'final_lqr_modal_frequencies.png');

frequencyFIG = fullfile( ...
    plotsFolder, ...
    'final_lqr_modal_frequencies.fig');

exportgraphics( ...
    fig3, ...
    frequencyPNG, ...
    'Resolution',300);

savefig(fig3,frequencyFIG);

%% =========================================================
% Plot 4 - Stability Margin Across Parameter Cases
% ==========================================================

fig4 = figure( ...
    'Name', ...
    'Final LQR Stability Margin Robustness', ...
    'NumberTitle', ...
    'off');

bar(caseMargin)

grid on

ylabel('Stability Margin [1/s]')
xlabel('Parameter Case')

title('Final Tuned LQR Stability Margin Under Parameter Uncertainty')

xticks(1:nCases)
xticklabels(caseNames)
xtickangle(15)

marginPNG = fullfile( ...
    robustnessPlotsFolder, ...
    'final_lqr_stability_margin_robustness.png');

marginFIG = fullfile( ...
    robustnessPlotsFolder, ...
    'final_lqr_stability_margin_robustness.fig');

exportgraphics( ...
    fig4, ...
    marginPNG, ...
    'Resolution',300);

savefig(fig4,marginFIG);

%% =========================================================
% Plot 5 - Maximum Closed-Loop Pole Real Part
% ==========================================================

fig5 = figure( ...
    'Name', ...
    'Final LQR Pole Real-Part Robustness', ...
    'NumberTitle', ...
    'off');

bar(caseMaxReal)

hold on

yline( ...
    0, ...
    '--', ...
    'Stability Boundary');

grid on

ylabel('Maximum Pole Real Part [1/s]')
xlabel('Parameter Case')

title('Closed-Loop Pole Stability Under Parameter Uncertainty')

xticks(1:nCases)
xticklabels(caseNames)
xtickangle(15)

realPartPNG = fullfile( ...
    plotsFolder, ...
    'final_lqr_max_pole_real_part.png');

realPartFIG = fullfile( ...
    plotsFolder, ...
    'final_lqr_max_pole_real_part.fig');

exportgraphics( ...
    fig5, ...
    realPartPNG, ...
    'Resolution',300);

savefig(fig5,realPartFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('====================================================')
disp('FINAL LQR STABILITY VERIFICATION SAVED')
disp('====================================================')

fprintf('\nResults:\n');

fprintf('%s\n',txtFile);
fprintf('%s\n',poleCSV);
fprintf('%s\n',modalCSV);
fprintf('%s\n',stabilityCSV);
fprintf('%s\n',matFile);

fprintf('\nPlots:\n');

fprintf('%s\n',polePNG);
fprintf('%s\n',dampingPNG);
fprintf('%s\n',frequencyPNG);
fprintf('%s\n',marginPNG);
fprintf('%s\n',realPartPNG);
