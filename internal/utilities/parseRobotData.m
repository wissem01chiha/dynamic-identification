function data = parseRobotData(dataFilePath, intIndex, stepIndex, fnlIndex)
%% 
% PARSEROBOTDATA( DATAFILEPATH )
% load a .csv file data obtained from kinova gen 3 mesurements and 
% return the structre representing this data.
%
% Inputs:
%   dataFilePath - data file full path.
%   intIndex     - index which is assumed to be the first one, used to 
%                  delete garbage or unneseccary data.
%   fnlIndex     - index which is assumed to be the final, severs the same
%                  purpose as intIndex
%   setpIndex    - index peroidclly ignoring data rows.
%
% Returns:
%   data  - MATLAB array struct.
%%
try 
    dataTable = readtable(dataFilePath);
catch ME
    error('Error loading data: %s', ME.message);
end
data.numRows = height(dataTable);
data.numCols = width(dataTable);
data.time = table2array(dataTable(intIndex:stepIndex:fnlIndex,1));
data.velocity = table2array(dataTable(intIndex:stepIndex:fnlIndex,9:15));
data.torque = table2array(dataTable(intIndex:stepIndex:fnlIndex,30:36));
data.current = table2array(dataTable(intIndex:stepIndex:fnlIndex,44:50));
data.position = table2array(dataTable(intIndex:stepIndex:fnlIndex,2:8));
data.temperature = table2array(dataTable(intIndex:stepIndex:fnlIndex,37:43));
data.desiredVelocity =table2array(dataTable(intIndex:stepIndex:fnlIndex,58:64));
data.desiredPosition =table2array(dataTable(intIndex:stepIndex:fnlIndex,51:57));
data.desiredAcceleration = table2array(dataTable(intIndex:stepIndex:fnlIndex,65:71));

data.timeStep = 1e-3;
data.samplingRate = 1000;
data.duration= (length(data.time)-1)*data.timeStep;

data.maxJointVelocity = max(data.velocity,[],1);
data.maxJointTorque = max(data.torque,[],1);
data.maxJointTorque = max(data.current,[],1);
data.maxJointTemperature = max(data.temperature,[],1);
data.maxJointPosition = max(data.position,[],1);
data.maxDesiredJointAcceleration = max(data.desiredAcceleration,[],1);

data.minJointVelocity = min(data.velocity,[],1);
data.minJointTorque = min(data.torque,[],1);
data.minJointCurrent = min(data.current,[],1);
data.minJointTemperature = min(data.temperature,[],1);
data.filtredData = min(data.position,[],1);
data.minDesiredJointAcceleration = min(data.desiredAcceleration,[],1);

data.corrJointPosition = corrcoef(data.position);
data.corrJointVelocity = corrcoef(data.velocity);
data.corrJointTorque   = corrcoef(data.torque);
data.corrJointCureent  = corrcoef(data.current);
data.corrJointAcceleration = corrcoef(data.desiredAcceleration);
data.timeUnit = 'milliseconds';
end