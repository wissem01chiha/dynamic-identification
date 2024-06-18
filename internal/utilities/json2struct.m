function  struct = json2struct(filePath)
%% 
% JSON2STRUCT( FILEPATH )
%
% Inputs:
%   filePath - robot description file path. 
%
% Returns:
%   struct   - MATLAB array struct.
%% 
if exist(filePath, 'file') ~= 2
        error('File not found: %s', filePath);
end
[~, ~, ext] = fileparts(filePath);
if ~strcmpi(ext, '.json')
     error('Only JSON files are supported');
end
try
    json = fileread(filePath);
    struct = jsondecode(json);
catch
    error('Error reading or decoding JSON file');
end 
end