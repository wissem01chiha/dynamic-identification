function  simulateDynamicModel(varargin)
%% 
% SIMULATEINVDYNAMICMODEL  
%
% simulate and plot the response of the inverse dynmaic model of
% the manipulator toward aan input trajectory 
% we gav the robot the trajectory motion deired and see the output 
% use RunTrajectory1.csv with 19432 sample data point
%
%
%
%% 
disp('Run inverse dynamic model Simulation ... ');
disp('Initialization ...');
tic;
dataFilePath = "datasets/kinova/identification_data/RunTrajectory1.csv";
if isempty(varargin)
    robot = kinovaGen3();
else
    robot = varargin{1};
end
data  = parseRobotData(dataFilePath, 1, 1, 1940);
data = filterRobotData(data);
torque = zeros(size(data.velocity));
friction = zeros(size(data.velocity));
sampleNum = size(data.velocity,1);
MAE  = zeros(size(data.velocity));
MSE =  zeros(size(data.velocity));
RMSE = zeros(size(data.velocity));

disp('Start calculations ...');
for t = 1 : sampleNum
    dQ_dt = data.velocity(t,:);
    Q_t =   data.position(t,:);
    d2Q_d2t = data.acceleration(t,:);
    torque(t,:)= computeInvDynamicModel(robot,Q_t, dQ_dt, d2Q_d2t);
    friction(t,:) = computeFrictionTorque(robot, dQ_dt);
    MAE(t, :) = mean(abs(data.torque(1:t,:) - torque(1:t,:)));
    MSE(t, :) = mean((data.torque(1:t,:) - torque(1:t,:)).^2);
    RMSE(t,:) = sqrt(MSE(t, :));
end

disp('Display Results ...');
  close all;
    figure(1);
    for  i=1:7
        subplot(3,3,i);
        plot(1.5.*data.torque(:,i),'g--');
        hold on;
        plot(torque(:,i),'r-');
        hold on;
        plot(friction(:,i),'b-.');
        legend('filtred','simulation','friction');
        xlabel("Time (seconds)");
        ylabel("Torque (N.m) ");
        title(['Joint'  num2str(i)]);
    end
    set(1, 'Position', [200, 150, 1000, 600]);
    sgtitle(['Inverse Dynamic Model Simulation, sampling frequency = ', ...
     num2str(1/data.timeStep) ], 'FontSize', 11);
 
    figure(2);
    for  i=1:7
        subplot(3,3,i);
        plot(MAE(:,i),'b-');
        hold on;
        plot(RMSE(:,i),'r-');
        legend('MAE','RMSE');
        xlabel("Time (seconds)");
        ylabel("Error");
        title(['Joint'  num2str(i)]) ;
    end
    set(2, 'Position', [200, 150, 1000, 600]);
    sgtitle(['Inverse Dynamic Model Simulation Error, sampling frequency = ', ...
     num2str(1/data.timeStep) ], 'FontSize', 11);
toc
end

