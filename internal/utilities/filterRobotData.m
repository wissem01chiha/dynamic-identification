function filtredData = filterRobotData(data, varargin)
%% 
% FILTERROBOTDATA( DATA )
%  filters the manipulator mesurements data using a low pass filter.
%
% Inputs:
%   data        - MATLAB array struct.
%
% Returns:
%   filtredData - MATLAB array struct.
%
%% 
validScalarNum = @(x) isnumeric(x) && isscalar(x) && (x > 0);
p = inputParser;
defaultFilterOrder = 3;
defaultFilterCuttoffHz = 30;
defaultDisplayOption = false;

addOptional(p,'filterOrder', defaultFilterOrder, validScalarNum);
addOptional(p,'cutoffFrequancy', defaultFilterCuttoffHz, validScalarNum);
addOptional(p,'display',defaultDisplayOption);

parse(p, varargin{:});
filterOrder      = p.Results.filterOrder;
cutoffFrequancy  = p.Results.cutoffFrequancy;
displayOption    = p.Results.display;

Fs = data.samplingRate;
cutoffFrequancy = 2*cutoffFrequancy /Fs;
filter = designfilt('lowpassiir','FilterOrder',filterOrder, ...
        'HalfPowerFrequency',cutoffFrequancy,'DesignMethod','butter');

filtredData.time = data.time;    
filtredData.timeStep = data.timeStep;
filtredData.duration= (length(filtredData.time)-1)*filtredData.timeStep;
filtredData.position = zeros(size(data.position));
filtredData.velocity = zeros(size(data.velocity));
filtredData.acceleration = zeros(size(data.velocity));
filtredData.torque = zeros(size(data.torque));
filtredData.current = zeros(size(data.current));
filtredData.temperature = zeros(size(data.temperature));

filtredData.desiredPosition = data.desiredPosition;
filtredData.desiredVelocity = data.desiredVelocity;
filtredData.desiredAcceleration = data.desiredAcceleration ;

for i =1:7
    filtredData.position(:,i)= filtfilt(filter,data.position(:,i));
    filtredData.velocity(:,i)= filtfilt(filter,data.velocity(:,i));
    filtredData.torque(:,i)= filtfilt(filter,data.torque(:,i));
    filtredData.temperature(:,i)= filtfilt(filter,data.temperature(:,i));
    filtredData.current(:,i)= filtfilt(filter,data.current(:,i));
    
    filtredData.acceleration(:,i) = diffcent(filtredData.velocity(:,i),data.timeStep);
    filtredData.acceleration(:,i) = filtfilt(filter,filtredData.acceleration(:,i));
end

filtredData.minJointPosition = min(filtredData.position,[],1);
filtredData.minJointTorque = min(filtredData.torque,[],1);
filtredData.minJointCurrent = min(filtredData.current,[],1);
filtredData.minJointVelocity = min(filtredData.velocity,[],1);
filtredData.minDesiredJointAcceleration = min(filtredData.acceleration,[],1);
filtredData.minJointTemperature = min(filtredData.temperature,[],1);

if displayOption
    close all;
    figure(1);
    for  i=1:7
        subplot(3,3,i);
        plot(data.velocity(:,i),'b-');
        hold on;
        plot(data.desiredVelocity(:,i),'r--');
        hold on;
        plot(filtredData.velocity(:,i), 'g-.');
        legend('recorded','desired','filtred');
        xlabel("Time (seconds)");
        ylabel("Angular Velocity ");
        title(['Joint'  num2str(i)]) ;
    end
    set(1, 'Position', [200, 150, 1000, 600]);
    sgtitle(['Joints Velocity, sampling = ',...
    num2str(Fs),'Hz, cutoff = ',...
    num2str(cutoffFrequancy*Fs/2),'Hz'],'FontSize', 11);
    
    figure(2);
    for i=1:7
        subplot(3,3,i);
        plot(data.torque(:,i),'b-');
        hold on
        plot(filtredData.torque(:,i),'r-');
        legend("recorded","filtred");
        ylabel("Torque (N.m)");
        title(['Joint'  num2str(i)]);
    end 
    sgtitle(['Joints Torque, sampling = ',...
    num2str(1/data.timeStep),'Hz, cutoff = ',...
      num2str(cutoffFrequancy*Fs/2),'Hz'],'FontSize', 11);
    set(2, 'Position', [200, 150, 1000, 600]);

    figure(3);
    for i=1:7
        subplot(3,3,i);
        plot(data.desiredAcceleration(:,i),'b-');
        hold on
        plot(filtredData.acceleration(:,i),'r-');
        legend("desired","estimated");
        ylabel("Acceleration (m/s^2)");
        title(['Joint'  num2str(i)]);
    end 
    sgtitle(['Joints Acceleration, sampling = ',...
    num2str(Fs),'Hz, cutoff = ',num2str(cutoffFrequancy*Fs/2),'Hz'],...
      'FontSize', 11);
    set(3, 'Position', [200, 150, 1000, 600]);
    
    figure(4);
    for i=1:7
        subplot(3,3,i);
        plot(data.current(:,i),'b-');
        hold on
        plot(filtredData.current(:,i),'r-');
        legend("recorded","filtred");
        ylabel("Current ( A )");
        title(['Joint'  num2str(i)]);
    end 
    sgtitle(['Joints Actuator Current, sampling  = ',...
    num2str(1/data.timeStep),'Hz, cutoff  = ',...
        num2str(cutoffFrequancy*Fs/2),'Hz'],'FontSize', 11);
    set(4, 'Position', [200, 150, 1000, 600]);    
end
end

