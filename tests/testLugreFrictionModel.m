%% lugreFrictionModel_test 
%  test the lugre friction model toward all paramters variations 
%%
% Define input parameters
Fc = 0.1;
Fs = 0.2;
v = 0.05;
vs = 0.03;
sigma1 = 0.01;
sigma0 = 0.02;
sigma2 = 0.03;
tinit = 0;
ts = 0.01;
tspan = 50;
z0 = 0;

 
 [t, F, Fss, err]=lugreFrictionModel(Fc, Fs, v, vs, sigma1, sigma0, sigma2, tinit, ts, tspan, z0);

% Plot the results
figure(1);
subplot(2, 1, 1);
plot(t, F, 'b', t, Fss, 'r');
xlabel('Time');
ylabel('Friction Force');
legend('Friction Force', 'Steady State Friction Force');
title('Friction Force vs. Time');

subplot(2, 1, 2);
plot(t, err, 'g');
xlabel('Time');
ylabel('Error');
title('Error vs. Time');
