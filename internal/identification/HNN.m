function HNN(dataFilePath, varargin)
%% -----------------------------------------------------------------------
% HNN 
% Prediction of joint torques using hopfiedl neural network 
% use RunTrajectory2.csv !
%
% Author: Wissem CHIHA
%% -----------------------------------------------------------------------
defaultDisplayOption = false;
p   = inputParser;

addOptional(p, 'dispaly', defaultDisplayOption);
parse(p, varargin{:});
displayOption = p.Results.dispaly;

data  = parseRobotData(dataFilePath, 1, 1, 3000);
filtredData = filterRobotData(data);

dataset = filtredData.torque';
estimatedJointTorque = zeros(7,length(dataset));
for t=1:length(dataset)
    [X, ~, ~, ~] = hopfield(10, 0.02,...
    dataset(:,1:t), dataset(:,1:t));
    estimatedJointTorque(:,t) = X;
end

if displayOption
    close all;
    figure(1);
    for i=1:7
        subplot(3,3,i);
        plot(estimatedJointTorque(i,:));
        hold on;
        
        
        
    end  
end
end

