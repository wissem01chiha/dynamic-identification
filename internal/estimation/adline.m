function adline()
%% 
%
%%
end

clc;
clear;
close all;

x = [
        0.67044,-0.437;
        -0.35508,-0.53923;
        0.10452,0.42226;
        0.95826,0.24915;
        0.098617,0.18122;
        -0.33915,0.32088;
        0.23894,-0.90489;
        -0.27873,-0.30243;
        0.51302,-0.09732;
        -0.1722,-0.51819;
        -0.01531,0.43009;
        0.38949,0.71236;
        0.94547,-0.43698;
        -0.34449,0.4621;
        0.67561,-0.72447;
        0.47814,0.67345;
        0.90835,-0.7228;
        -0.93615,0.17642;
        -0.28626,-0.26769;
        0.32531,0.61352;
];
t = [ 1 -1 1 1 1 1 -1 -1 1 -1 1 1 1 1 -1 1 -1 -1 -1 1 ];
w = rand(1,size(x,2))-0.5;
b = 0;
bias = 1;
LearningRate = 0.01;
epsilon = 0.008;
epochs = 0;

while true
    LargestChange = -inf;
    for i=1:size(x,1)
        % compute net input
        yin = sum(x(i,:).*w) + b*bias;
        % calculae differs
        DeltaB = LearningRate*(t(i)-yin)*bias;
        DeltaW = LearningRate*(t(i)-yin).*x(i,:);
        % update new weights
        b = b + DeltaB;
        w = w + DeltaW;
        % store largest change
        LocalMaximum = max(abs(DeltaW));
        if LocalMaximum > LargestChange
            LargestChange = LocalMaximum;
        end
    end
    
    epochs = epochs + 1;
    AdalinePlot(x, t, 1, ['Epoch number ',num2str(epochs)], [w(1) w(2) b])
    pause(0.1);
    
    if LargestChange < epsilon
        break
    end
end

fprintf('w1 = %d\n', w(1));
fprintf('w2 = %d\n', w(2));
fprintf('b = %d\n', b);
AdalinePlot(x, t, 2, ['Adaline with ',num2str(epochs),' epochs'], [w(1) w(2) b])

% question: bias random ya 0? bias jozve maximum differ ya na?