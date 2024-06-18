function [X, W, tarinLoss, testLoss] = hopfield(epochs, learningRate,...
    trainData, testData)
%%  
% Implementation of hopfield neural network with no self neuron
% connections and asynchronous update.
%
% Inputs:
%   trainingData  - MATLAB array of size (N * m) where N is the size of
%                   the network (number of neurons) and m is the number 
%                   of patterns. 
%
% Returns:
%   X             - Final state of the network.
%   W             - Weight matrix.
%   loss          - Training loss in the network.
%
%%  
assert(learningRate < 1 & learningRate > 0,...
    "Network learning rate should be in [0,1]! ");
assert(size(testData,1)==size(trainData,1),...
    "Test and train data should have same rows number !");
[N, m] = size(trainData);
W = zeros(N, N);
tarinLoss = zeros(epochs, 1);

for p = 1:m
    W = W + learningRate .* (trainData(:, p) * trainData(:, p)');  
end
W = W - diag(diag(W));

X = 15.*rand(N, 1);
for epoch = 1:epochs
    for i = 1:N
        input = W(i, :) * X;
        X(i) = 1 / (1 + exp(-input));  
    end
    closestPattern = findClosestPattern(X, trainData);
    tarinLoss(epoch) = mse(X, closestPattern);
    X = closestPattern;
    disp(['Epoch: ', num2str(epoch), ' - Loss: ', num2str(tarinLoss(epoch))]);
end
testLoss = computetestLoss(W, testData);
end

function testLoss = computetestLoss(W, testData)
%% ------------------------------------------------------------------------
% Computes the mean squared error over the test dataset.
%
% Inputs:
%   W        - Weight matrix
%   testData - MATLAB array of test patterns
%
% Returns:
%   test_loss - Mean squared error over the test dataset
%% ------------------------------------------------------------------------
[N, k] = size(testData);
totalError = 0;

for t = 1:k
    X = testData(:, t);
    for i = 1:N
        input = W(i, :) * X;
        X(i) = input;   
    end
    closestPattern = findClosestPattern(X, testData);
    totalError = totalError + mse(X, closestPattern);
end
testLoss = totalError / k;
end
function closestPattern = findClosestPattern(X, trainingData)
%% ------------------------------------------------------------------------
% Finds the closest training pattern to the current state vector X.
%
% Inputs:
%   X             - Current state vector
%   trainingData  - MATLAB array of training patterns
%
% Returns:
%   closestPattern - The training pattern closest to X
%% ------------------------------------------------------------------------
distances = sqrt(sum((trainingData - X).^2, 1));
[~, minIndex] = min(distances);
closestPattern = trainingData(:, minIndex);
end

function error = mse(X, target)
%% ------------------------------------------------------------------------
% Computes the mean squared error between the current state 
% vector X and the target vector.
%
% Inputs:
%   X       - Current state vector
%   target  - Target vector (closest training pattern)
%
% Returns:
%   error   - Mean squared error
%% ------------------------------------------------------------------------
error = mean((X - target).^2);
end