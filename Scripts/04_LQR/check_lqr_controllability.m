%% Detailed LQR Controllability Check
% Uses multiple numerical tests because ctrb(A,B)
% can be severely ill-conditioned for mechanical systems.

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
% Build Vehicle State-Space Model
% ==========================================================

oldFolder = pwd;
cd(projectFolder);

run(fullfile(projectFolder, 'Scripts', '01_Parameters', ...
    'build_lqr_state_space.m'));

cd(oldFolder);

n = size(A_lqr,1);

%% =========================================================
% Standard Controllability Matrix
% ==========================================================

Co = ctrb(A_lqr, B_lqr);

rank_default = rank(Co);

sCo = svd(Co);

condition_Co = ...
    sCo(1) / sCo(end);

%% =========================================================
% Examine Rank at Different Relative Tolerances
% ==========================================================

tolList = [ ...
    1e-6
    1e-8
    1e-10
    1e-12
    1e-14
    ];

rankByTolerance = zeros(size(tolList));

for i = 1:length(tolList)

    tol = ...
        tolList(i) * sCo(1);

    rankByTolerance(i) = ...
        sum(sCo > tol);

end

%% =========================================================
% PBH Controllability Test
% ==========================================================
%
% System is controllable if
%
% rank([lambda*I - A, B]) = n
%
% for every eigenvalue lambda.

lambda = eig(A_lqr);

pbhRank = zeros(length(lambda),1);
pbhMinSV = zeros(length(lambda),1);

for i = 1:length(lambda)

    PBH = [ ...
        lambda(i)*eye(n) - A_lqr, ...
        B_lqr];

    pbhRank(i) = rank(PBH);

    sPBH = svd(PBH);

    pbhMinSV(i) = min(sPBH);

end

minimumPBHRank = min(pbhRank);

isPBHControllable = ...
    minimumPBHRank == n;

%% =========================================================
% State Scaling
% ==========================================================
%
% Mechanical states have very different natural magnitudes:
% translations [m], angles [rad], velocities etc.
%
% Define representative state scales.

stateScale = [ ...
    0.01      % z_s        [m]
    0.01      % theta      [rad]
    0.01      % phi        [rad]
    0.01      % z_u_FL     [m]
    0.01      % z_u_FR     [m]
    0.01      % z_u_RL     [m]
    0.01      % z_u_RR     [m]
    0.1       % z_s_dot    [m/s]
    0.1       % theta_dot  [rad/s]
    0.1       % phi_dot    [rad/s]
    0.5       % z_u_FL_dot [m/s]
    0.5       % z_u_FR_dot [m/s]
    0.5       % z_u_RL_dot [m/s]
    0.5       % z_u_RR_dot [m/s]
    ];

Sx = diag(stateScale);

%% Scaled state model
%
% x = Sx*x_scaled
%
% x_scaled_dot =
% inv(Sx)*A*Sx*x_scaled + inv(Sx)*B*u

A_scaled = ...
    Sx \ (A_lqr * Sx);

B_scaled = ...
    Sx \ B_lqr;

Co_scaled = ...
    ctrb(A_scaled, B_scaled);

scaledSingularValues = ...
    svd(Co_scaled);

scaledRank = ...
    rank(Co_scaled);

scaledCondition = ...
    scaledSingularValues(1) / ...
    scaledSingularValues(end);

%% =========================================================
% PBH Test on Scaled Model
% ==========================================================

lambdaScaled = eig(A_scaled);

scaledPBHRank = zeros(length(lambdaScaled),1);
scaledPBHMinSV = zeros(length(lambdaScaled),1);

for i = 1:length(lambdaScaled)

    PBH = [ ...
        lambdaScaled(i)*eye(n) - A_scaled, ...
        B_scaled];

    scaledPBHRank(i) = rank(PBH);

    sPBH = svd(PBH);

    scaledPBHMinSV(i) = min(sPBH);

end

minimumScaledPBHRank = ...
    min(scaledPBHRank);

scaledPBHControllable = ...
    minimumScaledPBHRank == n;

%% =========================================================
% Display Results
% ==========================================================

disp('====================================================')
disp('DETAILED LQR CONTROLLABILITY CHECK')
disp('====================================================')

fprintf( ...
    'Number of states                 = %d\n', ...
    n);

fprintf( ...
    'Default ctrb rank                = %d\n', ...
    rank_default);

fprintf( ...
    'Controllability matrix condition = %.3e\n\n', ...
    condition_Co);

disp('RANK VS RELATIVE SINGULAR-VALUE TOLERANCE')
disp('----------------------------------------------------')

for i = 1:length(tolList)

    fprintf( ...
        'Tolerance %.0e : rank = %d\n', ...
        tolList(i), ...
        rankByTolerance(i));

end

fprintf('\nPBH TEST\n');
fprintf( ...
    'Minimum PBH rank                 = %d\n', ...
    minimumPBHRank);

if isPBHControllable

    disp('PBH result                       = CONTROLLABLE')

else

    disp('PBH result                       = NOT CONTROLLABLE')

end

fprintf('\nSCALED MODEL\n');

fprintf( ...
    'Scaled ctrb rank                 = %d\n', ...
    scaledRank);

fprintf( ...
    'Scaled ctrb condition            = %.3e\n', ...
    scaledCondition);

fprintf( ...
    'Minimum scaled PBH rank          = %d\n', ...
    minimumScaledPBHRank);

if scaledPBHControllable

    disp('Scaled PBH result                = CONTROLLABLE')

else

    disp('Scaled PBH result                = NOT CONTROLLABLE')

end

disp('====================================================')

%% =========================================================
% PBH Results Table
% ==========================================================

PoleReal = real(lambda);
PoleImag = imag(lambda);

PBHRank = pbhRank;
MinimumPBHSingularValue = pbhMinSV;

pbhTable = table( ...
    PoleReal, ...
    PoleImag, ...
    PBHRank, ...
    MinimumPBHSingularValue);

%% =========================================================
% Save CSV
% ==========================================================

csvFile = fullfile( ...
    resultsFolder, ...
    'lqr_controllability_detailed.csv');

writetable(pbhTable, csvFile);

%% =========================================================
% Save TXT
% ==========================================================

txtFile = fullfile( ...
    resultsFolder, ...
    'lqr_controllability_detailed.txt');

fid = fopen(txtFile, 'w');

fprintf(fid, ...
    'DETAILED LQR CONTROLLABILITY CHECK\n');

fprintf(fid, ...
    '==================================\n\n');

fprintf(fid, ...
    'Default controllability rank = %d / %d\n', ...
    rank_default, n);

fprintf(fid, ...
    'Controllability condition number = %.6e\n\n', ...
    condition_Co);

fprintf(fid, ...
    'Minimum PBH rank = %d / %d\n', ...
    minimumPBHRank, n);

fprintf(fid, ...
    'PBH controllable = %d\n\n', ...
    isPBHControllable);

fprintf(fid, ...
    'Scaled controllability rank = %d / %d\n', ...
    scaledRank, n);

fprintf(fid, ...
    'Scaled condition number = %.6e\n', ...
    scaledCondition);

fprintf(fid, ...
    'Minimum scaled PBH rank = %d / %d\n', ...
    minimumScaledPBHRank, n);

fprintf(fid, ...
    'Scaled PBH controllable = %d\n', ...
    scaledPBHControllable);

fclose(fid);

%% =========================================================
% Save MAT
% ==========================================================

matFile = fullfile( ...
    resultsFolder, ...
    'lqr_controllability_detailed.mat');

save(matFile, ...
    'Co', ...
    'sCo', ...
    'rank_default', ...
    'condition_Co', ...
    'tolList', ...
    'rankByTolerance', ...
    'pbhTable', ...
    'isPBHControllable', ...
    'Sx', ...
    'A_scaled', ...
    'B_scaled', ...
    'scaledRank', ...
    'scaledCondition', ...
    'scaledPBHControllable');

%% =========================================================
% Plot Controllability Singular Values
% ==========================================================

fig1 = figure( ...
    'Name', ...
    'Controllability Singular Values', ...
    'NumberTitle', ...
    'off');

semilogy( ...
    1:length(sCo), ...
    sCo, ...
    '-o', ...
    'LineWidth', 1.5);

grid on

xlabel('Singular Value Index')
ylabel('Singular Value')

title('Unscaled Controllability Matrix Singular Values');

singularPNG = fullfile( ...
    plotsFolder, ...
    'lqr_controllability_singular_values.png');

singularFIG = fullfile( ...
    plotsFolder, ...
    'lqr_controllability_singular_values.fig');

exportgraphics( ...
    fig1, ...
    singularPNG, ...
    'Resolution', 300);

savefig(fig1, singularFIG);

%% =========================================================
% Plot Scaled Singular Values
% ==========================================================

fig2 = figure( ...
    'Name', ...
    'Scaled Controllability Singular Values', ...
    'NumberTitle', ...
    'off');

semilogy( ...
    1:length(scaledSingularValues), ...
    scaledSingularValues, ...
    '-o', ...
    'LineWidth', 1.5);

grid on

xlabel('Singular Value Index')
ylabel('Singular Value')

title('Scaled Controllability Matrix Singular Values');

scaledPNG = fullfile( ...
    plotsFolder, ...
    'lqr_scaled_controllability_singular_values.png');

scaledFIG = fullfile( ...
    plotsFolder, ...
    'lqr_scaled_controllability_singular_values.fig');

exportgraphics( ...
    fig2, ...
    scaledPNG, ...
    'Resolution', 300);

savefig(fig2, scaledFIG);

%% =========================================================
% Confirmation
% ==========================================================

disp(' ')
disp('Detailed controllability check saved.');

fprintf('\nResults:\n');
fprintf('%s\n', txtFile);
fprintf('%s\n', csvFile);
fprintf('%s\n', matFile);

fprintf('\nPlots:\n');
fprintf('%s\n', singularPNG);
fprintf('%s\n', scaledPNG);
