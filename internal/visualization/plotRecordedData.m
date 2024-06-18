function plotRecordedData(varargin)
%%  
% debug the recorded data files of the manipulator 
%
%
%% 
if (~exist("figures/plotRecordedData","dir"))
    mkdir figures/plotRecordedData
end
if numel(varargin)== 0
    dataFileName =...
     "datasets/kinova/identification_data/RunTrajectory1.csv";
elseif numel(varargin) == 1
    dataFileName = varargin{1};
else
    error("Invalid Inputs Number !");
end
data = parseRobotData(dataFileName, 1, 1, 1940);

disp('');
close all;
figure(1);
for i=1:7
    subplot(3,3,i);
    plot(data.velocity(:,i));
    xlabel("Time (seconds)");
    ylabel("Angular Velocity ");
    title(['Joint'  num2str(i)]) ;
end
set(1, 'Position', [200, 150, 1000, 600]);
sgtitle('Joints Recorded Velocity ','FontSize', 11);
saveas(1, 'figures/plotRecordedData/joints_velocity.png');

figure(2);
for i=1:7
    subplot(3,3,i);
    plot(data.position(:,i));
    xlabel("Time (seconds)");
    ylabel("Angular Position ");
    title(['Joint'  num2str(i)]) ;
end
set(2, 'Position', [200, 150, 1000, 600]);
sgtitle('Joints Recorded Position','FontSize', 11);
saveas(2, 'figures/plotRecordedData/joints_position.png');

figure(3);
for i=1:7
    subplot(3,3,i);
    plot(data.torque(:,i));
    xlabel("Time (seconds)");
    ylabel("Torque (N.m) ");
    title(['Joint'  num2str(i)]) ;
end
set(3, 'Position', [200, 150, 1000, 600]);
sgtitle('Joints Recorded Torque','FontSize', 11);
saveas(3, 'figures/plotRecordedData/joints_torques.png');

figure(4);
for i=1:7
    subplot(3,3,i);
    plot(data.current(:,i));
    xlabel("Time (seconds)");
    ylabel("Current (mA) ");
    title(['Joint'  num2str(i)]);
end
set(4, 'Position', [200, 150, 1000, 600]);
sgtitle('Joints Recorded Current','FontSize', 11);
saveas(4, 'figures/plotRecordedData/joints_current.png');


figure(5);
for i=1:7
   subplot(3,3,i);
   scatter3(data.velocity(:,i), data.position(:,i),...
       data.torque(:,i), 2, 'filled');
    xlabel("Velocity");
    ylabel("Position");
    zlabel("Torque (N.m)");
    title(['Joint ' num2str(i)]);
end
set(5, 'Position', [200, 150, 1000, 600]);
sgtitle('Velocity vs Position vs Torque','FontSize', 11);
saveas(5, 'figures/plotRecordedData/velocity_acceleration_torque.png');


end