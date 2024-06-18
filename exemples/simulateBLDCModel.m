function simulateBLDCModel(varargin)
%% -----------------------------------------------------------------------
% Simulate the developped torque of the Brushedd DC motor model.
% 
%
%% -----------------------------------------------------------------------
disp('');
data  = parseRobotData(...
  "datasets/kinova/identification_data/RunTrajectory2.csv", 1, 1, 24000);

robot = kinovaGen3();
data = filterRobotData(data);
current = data.current;
torque = data.torque;
simCurrent = zeros(size(current));
simTorque = zeros(size(torque)); 
torqueError = zeros(1,robot.ndof);
currentError = zeros(1,robot.ndof);

for i =1:robot.ndof
    Q_t = data.position(:,i);
    dQ_dt = data.velocity(:,i);
    [Td, I] = BLDC(Q_t, dQ_dt, robot.samplingFrequency);
    simCurrent(:,i) =  I;
    simTorque(:,i)  =  Td;
    torqueError(i) = mean(abs(simTorque(:,i) - torque(:,i)));
    currentError(i) = mean( abs(simCurrent(:,i) - current(:,i)));
end

close all;
figure(1);
for i =1:robot.ndof
    subplot(3,3,i);
    plot(simCurrent(:,i), 'r--');
    hold on;
    plot(current(:,i), 'b-');
    title(['Joint'  num2str(i)]);
    legend('simulation','recorded');
end
set(1, 'Position', [200, 150, 1000, 600]);
sgtitle(['BLDC simulation current, sampling frequency = ', ...
     num2str(1/data.timeStep) ], 'FontSize', 11);

figure(2);
for i =1:robot.ndof
    subplot(3,3,i);
    plot(simTorque(:,i), 'r--');
    hold on;
    plot(torque(:,i), 'b-');
    title(['Joint'  num2str(i)]);
    legend('simulation','recorded');
end
set(2, 'Position', [200, 150, 1000, 600]);
sgtitle(['BLDC simulation torque, sampling frequency = ', ...
     num2str(1/data.timeStep) ], 'FontSize', 11);

 
end

