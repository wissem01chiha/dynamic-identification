function struct2json(structData, filename)
%% -----------------------------------------------------------------------
% STRUCT2JSON(STRUCT, FILE) Saves a MATLAB struct to a JSON file.
%
% Author: Wissem CHIHA
%% -----------------------------------------------------------------------
if nargin ~= 2
   error('Function requires exactly two input arguments: structData and filename');
end
if ~isstruct(structData)
     error('The first input argument must be a MATLAB struct');
end
if ~ischar(filename) || isempty(filename)
     error('The second input argument must be a non-empty string representing the filename');
end
jsonString = jsonencode(structData);   
fileId = fopen(filename, 'w');
if fileId == -1
    error('Could not create or open the file for writing');
end    
fprintf(fileId, '%s', jsonString);
fclose(fileId);    
end