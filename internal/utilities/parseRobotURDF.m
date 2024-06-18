function robot = parseRobotURDF(robotDescriptionFilePath)
%% 
% PARSEROBOTURDF( FILE_PATH )
%
% Inputs:
%   robotDescriptionFilePath - URDF file of the robot.
%
% Returns:
%   robot  - MATLAB arry struct.
%
%%
try
    disp("Loading Robot Parameters from Description File...");
    robot = importrobot(robotDescriptionFilePath);
    robot.DataFormat = 'row'; 
    robot.Gravity = [0 0 -9.81];
    disp("Robot parameters loaded successfully.");
catch ME
    error('Error loading robot parameters: %s', ME.message);
end
disp("Checking Physical Consistency of Robot Parameters...");
if ~ checkPhysicalConsistency(robot)
   error('The parameters of the robot are not physically consistent!');
else
   disp("Physical consistency check passed successfully.");
end
end