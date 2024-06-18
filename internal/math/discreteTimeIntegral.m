function intVector = discreteTimeIntegral(vector,timeStep)
%% ------------------------------------------------------------------------
% DISCRETETIMEINTEGRAL( V, Fs)  
%   Takes a sampled matlab vector with given sampling rate and compute the
%   time integral of this vector.
%
% Inputs:
%   vector      - MATLAB 1-D size input vector.
%   sampligRate - Samplig frequancy witch this vector was recorded with.
%
% Retuns:
%   intVector   - MATLAB 1-D size vector.
% 
% Author : Wissem CHIHA 
%% ------------------------------------------------------------------------
assert(isnumeric(timeStep) && isscalar(timeStep) && ...
    timeStep > 0, ...
"Input validation failed: 'time step' must be numeric, positive scalar.");
if timeStep > 1
    warning("time step is too high: results may be inaccurate !");
end
if timeStep < realmin
    warning("time step is too small: results may be inaccurate !");
end
intVector = zeros(size(vector));
for i = 1:length(vector)
    if i == 1 
        intVector(i)= vector(i);
    else
        intVector(i) = intVector(i-1) + 2 * vector(i) ;
    end
end
intVector = (timeStep/2).*intVector;
end