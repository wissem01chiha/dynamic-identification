function trajectorySimulator()
%% TRAJECTORYSIMULATOR.M
%
% Author: Wissem CHIHA
% 
%%
disp(" Starting Trajectory Simulation ... ");
endEffector = 'spherical_wrist_2_link';
timeStep = 0.1; 
initTime = 0;
distance = 10;
toolSpeed = 0.1;
finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
 
robot  = loadRobotModel('robots/kinovaGen3.urdf');
currentRobotJConfig = homeConfiguration(robot);
numJoints = numel(currentRobotJConfig);

 

stateJoint = getRecordedData("data/Trajectory1.csv","position");
disp(" Diplay Vislusation Results ...");
close all;
show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on;
for i=1:length(trajTimes)
    configNow = [0 stateJoint(i,1:7) 0  0 0 0 0];
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'r.','MarkerSize',5);
    hold on;
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([0 1.3]);
    title('Kinova© Gen3 Manipulator Trajectory');
    drawnow;
end
end