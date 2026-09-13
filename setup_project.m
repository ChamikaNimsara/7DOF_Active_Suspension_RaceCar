%% Configure MATLAB paths for the packaged project

projectFolder = fileparts(mfilename('fullpath'));

addpath(genpath(fullfile(projectFolder, 'Scripts')));
addpath(fullfile(projectFolder, 'Model'));

fprintf('Project paths configured from:\n%s\n', projectFolder);
