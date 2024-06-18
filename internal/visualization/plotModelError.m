function plotModelError(varargin)
%% 
%
%
%%

disp('Initialization ...');
dataFilePath = "datasets/kinova/identification_data/RunTrajectory1.csv";
data  = parseRobotData(dataFilePath, 1, 1, 1940);
robot = kinovaGen3();
data = filterRobotData(data);
sampleNum = size(data.velocity,1);
error= zeros(1,sampleNum);
disp('Start calculations ...');
close all;
figure(1);
for t =1:sampleNum
    disp(['Processing Trajectory Point :', num2str(t),...
           ' / ',num2str(length(data.time))]);
    dQ_dt = [data.velocity(t,:) zeros(1,6)];
    Q_t =   [data.position(t,:) zeros(1,6)];
    d2Q_d2t = [data.acceleration(t,:) zeros(1,6)];
    %x = robot.getRobotParams();
    error(t) = computeModelError(robot, Q_t, dQ_dt, d2Q_d2t, data.torque(t,:), 1.5.*rand(1,224));
    plot(norm(data.torque(t,:),2),error(t),'*','MarkerSize', 3);
    hold on;
    drawnow;
end
xlabel("Mesured Torque Norm2");
ylabel("RMSE ");
hold off;
