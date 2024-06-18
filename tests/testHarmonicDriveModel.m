function testHarmonicDriveModel()
%% 
tic;
clear all 
close all;
clc
disp("******************************************************************");
disp("       STARTING SIMULATION OF TRANSMISSION SYSTEM DYNAMICS        ");
disp("******************************************************************");
disp("INITIALIZATION ...");
amplitude    = 1.5;       
frequency    = 15;       
phase        = 0;           
duration     = 1;       
samplingRate = 100;  
reductionRatio = 5;

time = linspace(0, duration, duration * samplingRate);
inputTorque = amplitude .* sin(2 * pi .* frequency .* time + phase)'+...
    amplitude .* sin(8 * pi .* frequency .* time + phase)';
inputVelocity = amplitude .* sin(2 * pi .* frequency .* time + phase + 10)'+...
    amplitude .* sin(4*2 * pi .* frequency .* time + phase + 10)';
disp("START CALCULATIONS ...");
[outputTorque, outputVelocity, frictionTorque, efficacity, compliance] =...
computeHarmonicDriveModel(reductionRatio,inputTorque,...
 inputVelocity, samplingRate);
%%
disp("CALCULATION DONE SUCCESFFULLY ... ");
disp("DISPLAY RESULTS ...");

figure(1);
plot(outputVelocity);
hold on;
plot(inputVelocity);
legend("output","input");
title("Input-Output Velocity Property");

figure(2);
plot(inputTorque);
hold on;
plot(outputTorque);
legend("input","output");
title("Input-Output Torque Property");

figure(3);
plot(frictionTorque);
hold on;
plot(inputTorque);
legend("friction", "input");
title("Friction Torque ");

figure(4);
subplot(2,1,1);
plot(compliance);
title("Compliance ");
subplot(2,1,2);
plot(1./compliance(10:end));
title("Stiffness");
figure(5);
plot(efficacity);
title("Stiffness");
toc
end

 
