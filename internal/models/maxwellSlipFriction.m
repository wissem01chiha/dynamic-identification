function [t, F] = maxwellSlipFriction(n, velocity, k, c, sigma0, samplingRate)
%% ------------------------------------------------------------------------
% maxwellSlipFriction - Compute Maxwell Slip Friction Model.
%
% Inputs:
%   n         - Number of Maxwell elements.
%   velocity  - Velocity (m/s)
%   k         - Stiffness of Maxwell elements (N/m)
%   c         - Damping coefficients of Maxwell elements (Ns/m)
%   sigma0    - Static friction force (N)

% Returns:
%   t    - Simulation time vector.
%   F    - Friction Force for the given velocity
%
% Ref:
%   FUNDAMENTALS OF FRICTION MODELING - Farid Al-Bender - 2010.
%
% Author: Wissem CHIHA
%% ------------------------------------------------------------------------ 
timeSpan = (length(velocity)-1)/samplingRate;
timeStep = 1/samplingRate;
t = 0:timeStep:timeSpan; 

initial_conditions = zeros(1, 2*n);  
[~, y] = ode45(@(t, y) maxwell(t, y, n, k, c, sigma0,...
    velocity), t, initial_conditions);
F = sum(y(:, n+1:end), 2);
end

function dydt = maxwell(~, y, n, k, c, sigma0, v)
    dydt = zeros(2*n, 1);
    x = y(1:n);
    F = y(n+1:end);
    for i = 1:n
        dxdt = v(i) - F(i)/c(i); 
        dFdt = k(i) * dxdt; 
        dydt(i) = dxdt;
        dydt(n+i) = dFdt;
    end
    F_total = sum(F);
    if abs(F_total) < sigma0
        dydt(n+1:end) = 0; 
    end
end


