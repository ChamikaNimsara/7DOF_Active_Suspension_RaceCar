%% LQR Weight Sweep
% Systematic tuning of normalized LQR Q/R weights

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
% Build Model
% ==========================================================

oldFolder = pwd;

cd(projectFolder);

run(fullfile(projectFolder, 'Scripts', '01_Parameters', ...
    'build_lqr_state_space.m'));

cd(oldFolder);

%% =========================================================
% State Scaling
% ==========================================================

stateScale = [ ...
    0.010
    0.010
    0.010
    0.020
    0.020
    0.020
    0.020
    0.20
    0.20
    0.20
    1.00
    1.00
    1.00
    1.00];

Sx = diag(stateScale);

Su = F_act_max * eye(4);

A_norm = ...
    Sx \ (A_lqr * Sx);

B_norm = ...
    Sx \ (B_lqr * Su);

%% =========================================================
% Base Q Structure
% ==========================================================

Q_base = diag([ ...
    8 ...
    20 ...
    20 ...
    1 ...
    1 ...
    1 ...
    1 ...
    5 ...
    8 ...
    8 ...
    0.5 ...
    0.5 ...
    0.5 ...
    0.5]);

R_base = eye(4);

%% =========================================================
% Sweep Values
% ==========================================================

Q_scale_list = [ ...
    0.5 ...
    1 ...
    2 ...
    4];

R_scale_list = [ ...
    1 ...
    2 ...
    4 ...
    8];

nQ = length(Q_scale_list);
nR = length(R_scale_list);

totalCases = nQ * nR;

%% =========================================================
% Preallocate
% ==========================================================

QScale = zeros(totalCases,1);
RScale = zeros(totalCases,1);

MaxClosedLoopRealPart = zeros(totalCases,1);
Stable = false(totalCases,1);

GainNorm = zeros(totalCases,1);

caseIndex = 0;

%% =========================================================
% Sweep
% ==========================================================

disp('====================================================')
disp('LQR Q/R WEIGHT SWEEP')
disp('====================================================')

for i = 1:nQ

    for j = 1:nR

        caseIndex = caseIndex + 1;

        qScale = Q_scale_list(i);
        rScale = R_scale_list(j);

        Q = qScale * Q_base;
        R = rScale * R_base;

        [K_norm, ~, ~] = ...
            lqr(A_norm, B_norm, Q, R);

        K_physical = ...
            Su * (K_norm / Sx);

        A_cl = ...
            A_lqr - B_lqr*K_physical;

        poles = eig(A_cl);

        maxReal = ...
            max(real(poles));

        isStable = ...
            maxReal < 0;

        QScale(caseIndex) = qScale;
        RScale(caseIndex) = rScale;

        MaxClosedLoopRealPart(caseIndex) = ...
            maxReal;

        Stable(caseIndex) = ...
            isStable;

        GainNorm(caseIndex) = ...
            norm(K_physical, 'fro');

        fprintf( ...
            ['Case %2d/%2d | Q scale = %.2f | ' ...
             'R scale = %.2f | max pole = %.3f | stable = %d\n'], ...
            caseIndex, ...
            totalCases, ...
            qScale, ...
            rScale, ...
            maxReal, ...
            isStable);

    end

end

%% =========================================================
% Results Table
% ==========================================================

sweepTable = table( ...
    QScale, ...
    RScale, ...
    MaxClosedLoopRealPart, ...
    Stable, ...
    GainNorm);

disp(' ')
disp(sweepTable)

%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'lqr_weight_sweep.csv');

writetable(sweepTable, csvFile);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'lqr_weight_sweep.mat');

save(matFile, ...
    'sweepTable', ...
    'Q_scale_list', ...
    'R_scale_list', ...
    'Q_base', ...
    'R_base', ...
    'Sx', ...
    'Su');

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'lqr_weight_sweep.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'LQR Q/R WEIGHT SWEEP\n');

fprintf(fid, ...
    '====================\n\n');

for i = 1:height(sweepTable)

    fprintf(fid, ...
        ['Case %d\n' ...
         'Q scale = %.4f\n' ...
         'R scale = %.4f\n' ...
         'Max closed-loop pole real part = %.6f\n' ...
         'Stable = %d\n' ...
         'Gain Frobenius norm = %.6e\n\n'], ...
        i, ...
        sweepTable.QScale(i), ...
        sweepTable.RScale(i), ...
        sweepTable.MaxClosedLoopRealPart(i), ...
        sweepTable.Stable(i), ...
        sweepTable.GainNorm(i));

end

fclose(fid);

%% =========================================================
% Plot Stability Margin
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'LQR Stability Margin Sweep', ...
    'NumberTitle', ...
    'off');

scatter( ...
    RScale, ...
    MaxClosedLoopRealPart, ...
    80, ...
    QScale, ...
    'filled');

grid on

xlabel('R Scale')
ylabel('Maximum Closed-Loop Pole Real Part [1/s]')

title('LQR Q/R Sweep - Stability Margin')

cb = colorbar;
ylabel(cb, 'Q Scale');

stabilityPNG = fullfile( ...
    plotsFolder, ...
    'lqr_weight_sweep_stability.png');

stabilityFIG = fullfile( ...
    plotsFolder, ...
    'lqr_weight_sweep_stability.fig');

exportgraphics( ...
    fig1, ...
    stabilityPNG, ...
    'Resolution', ...
    300);

savefig(fig1, stabilityFIG);

%% =========================================================
% Plot Gain Magnitude
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'LQR Gain Magnitude Sweep', ...
    'NumberTitle', ...
    'off');

scatter( ...
    RScale, ...
    GainNorm, ...
    80, ...
    QScale, ...
    'filled');

grid on

xlabel('R Scale')
ylabel('||K||_F')

title('LQR Q/R Sweep - Controller Gain Magnitude')

cb2 = colorbar;
ylabel(cb2, 'Q Scale');

gainPNG = fullfile( ...
    plotsFolder, ...
    'lqr_weight_sweep_gain_norm.png');

gainFIG = fullfile( ...
    plotsFolder, ...
    'lqr_weight_sweep_gain_norm.fig');

exportgraphics( ...
    fig2, ...
    gainPNG, ...
    'Resolution', ...
    300);

savefig(fig2, gainFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('LQR weight sweep saved successfully.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', stabilityPNG);
fprintf('%s\n', gainPNG);
