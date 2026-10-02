function setup_tools
%SETUP_TOOLS Add the maintained MATLAB tool folders to the search path.
% From the repository root: addpath('tools'); setup_tools;
toolRoot = fileparts(mfilename('fullpath'));
folders = {'calibration', 'noise_analysis', 'visualization'};
for k = 1:numel(folders)
    addpath(fullfile(toolRoot, folders{k}));
end
end
