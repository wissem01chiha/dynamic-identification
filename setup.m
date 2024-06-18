VERBOSE = true;
MATLAB_VERSION = version;
INCLUDE_PATH = ["tests","release","figures",...
     "robots","datasets","exemples","docs","internal","external"];
RUN_TESTS = false;
COMPILE_MEX_FUNCTIONS = true;
%%  
if VERBOSE 
    disp("Setting up...");
end
if VERBOSE 
        disp(['MATLAB VERSION DETECTED ',MATLAB_VERSION]);
        disp("Add toolbox subfolders to matlab search path ...");
end

for i = 1:numel(INCLUDE_PATH)
    path = fullfile(pwd, INCLUDE_PATH(i));

    if exist(path, 'dir')
        addpath(path);
    else
        fprintf('Folder "%s" does not exist.\n', path);
    end
    subfolders = genpath(fullfile(pwd, INCLUDE_PATH{i}));
    folderPaths = strsplit(subfolders, pathsep);
    for j = 1:numel(folderPaths)
        path = folderPaths{j};
        if ~isempty(path) && exist(path, 'dir')
            addpath(path);
        elseif ~isempty(path)
            fprintf('Folder "%s" does not exist.\n', path);
        end
    end
end
if VERBOSE
    disp('Compiling MEX functions...');
end 
if COMPILE_MEX_FUNCTIONS
    mexfileList = findFilesWithExtension(pwd, '.c'); 
    for i = 1:numel(mexfileList)
        try
            mex(mexfileList{i});
            disp(['Compiled: ', mexfileList{i}]);
        catch
            warning(['Failed to compile: ', mexfileList{i}]);
        end
    end
end
clear all;

function fileList = findFilesWithExtension(directory, extension)
    contents = dir(directory);
    fileList = {};
    for i = 1:numel(contents)
        item = contents(i);
        if strcmp(item.name, '.') || strcmp(item.name, '..')
            continue;
        end
        if item.isdir
            subdir = fullfile(directory, item.name);
            fileList = [fileList; findFilesWithExtension(subdir, extension)];
        elseif endsWith(item.name, extension)
            fileList = [fileList; fullfile(directory, item.name)];
        end
    end
end
