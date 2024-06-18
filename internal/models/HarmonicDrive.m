function [outputTorque, outputVelocity, frictionTorque, efficacity,...
    compliance] = HarmonicDrive(reductionRatio,inputTorque,...
    inputVelocity,samplingRate,varargin)
%% -----------------------------------------------------------------------
% COMPUTEHARMONICDRIVEMODEL 
%  Simulate Harmonic Drive Reduction System Dynamics Response.
%
% Ref:
%   Understanding and Modeling the Behavior of a Harmonic Drive Gear 
%   Transmission - Timothy D.Tuttlie - MIT Artificial Intelligence Lab
%   1993.
%   A Refined Dynamic Model of Harmonic Drive and Its Dynamic
%   Response Analysis- X.Zhang, T.T, G.Jiang, X.Mei, C.Zou - 2020
%
% Author: Wissem CHIHA 
%% -----------------------------------------------------------------------
p   = inputParser;
default_kinematicErrorParams  = [0.27 0.365 0.72; 0.01 0.01 0.02];
default_complianceParams      = [0.01 0.01];
default_frictionParams        = [1.1 1.05 1.03 1.09 1.09];
timeStep                      = 1/samplingRate;

addOptional(p, 'kinematicErrorParams',default_kinematicErrorParams);
addOptional(p, 'complianceParams',default_complianceParams);
addOptional(p, 'frictionParams',default_frictionParams);

parse(p, varargin{:});
kinematicErrorParams = p.Results.kinematicErrorParams;
complianceParams     = p.Results.complianceParams;
frictionParams       = p.Results.frictionParams;

assert(all(size(kinematicErrorParams) == [2 3]),...
    "Invalid dimensions: 'kinematicErrorParams' must be a 2x3 matrix.");
assert(length(frictionParams) == 5,...
    "Invalid length: 'frictionParams' must be a vector of length 5.");
assert(length(complianceParams) == 2,...
    "Invalid length: 'complianceParams' must be a vector of length 2.");
assert(all(size(inputTorque) == size(inputVelocity)),...
    "Size mismatch: 'inputTorque' and 'inputVelocity' must have the same dimensions.");

timeSpan = (length(inputVelocity)-1)/samplingRate;
time = 0:timeStep:timeSpan;

inputTorqueDerivative   =  columnVector(diffcent(inputTorque, timeStep));
inputVelocityDerivative =  columnVector(diffcent(inputVelocity, timeStep));
inputVelocity           =  columnVector(inputVelocity);
inputTorque             =  columnVector(inputTorque);

fun = @(t,x) f(x, complianceParams(1), complianceParams(2),...
    inputVelocity, reductionRatio, inputTorque, inputTorqueDerivative,...
    inputVelocityDerivative);
 
x0 = 0.0001.*ones(1,length(inputVelocity));
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-2, 'MaxStep', 0.1);
disp("solving");
[~,flexSplineVelocity] = ode45(fun,time,x0,options);
disp("endd solver");
maxInputVelocity = max(inputVelocity);

[numRows, numCols] = size(flexSplineVelocity);
for i = 1:numRows
    for j = 1:numCols
        if abs(flexSplineVelocity(i, j)) > maxInputVelocity
            flexSplineVelocity(i, j) =  maxInputVelocity;
        end
    end
end

flexSplineVelocity = min(flexSplineVelocity,[],2);

flexSplineVelocity = columnVector(flexSplineVelocity);

compliance = computeCompliance(reductionRatio, inputVelocity, ...
    flexSplineVelocity, complianceParams(1), complianceParams(2));
    
outputVelocity = flexSplineVelocity + ...
        kinematicErrorParams(1,1).*sin(inputVelocity +...
            kinematicErrorParams(2,1))+ ...
        kinematicErrorParams(1,2).*sin(2.*inputVelocity +...
            kinematicErrorParams(2,2))+ ...
        kinematicErrorParams(1,3).*sin(4.*inputVelocity+...
            kinematicErrorParams(2,3))+ 1/reductionRatio.*inputVelocity;       
flexplineVelocityDerivative = diffcent(flexSplineVelocity, timeStep);

frictionTorque = frictionParams(1) + frictionParams(2).*...
    flexplineVelocityDerivative...
    +frictionParams(3).*flexplineVelocityDerivative.^3 + ...
    frictionParams(4).*cos(flexplineVelocityDerivative)+...
    frictionParams(5).*sin(flexplineVelocityDerivative);

maxInputTorque = abs(max(inputTorque));
[numRows, numCols] = size(frictionTorque);
for i = 1:numRows
    for j = 1:numCols
        if abs(frictionTorque(i, j)) > maxInputTorque
            frictionTorque(i, j) = sign(frictionTorque(i, j))* maxInputTorque;
        end
    end
end
outputTorque = reductionRatio.* inputTorque + frictionTorque; 
efficacity = abs(outputTorque.*outputVelocity./(inputTorque.*inputVelocity));
end

function y = f(x, c1, c2, inputSpeed,N,inputTorque,inputTorqueDerivative,...
    inputSpeedDerivative)
%%  Retrun The derivative of the output flexpline veclocity without kinemtaic
% error in the harmonic drive system
C = computeCompliance(N, inputSpeed, x, c1, c2);
y =((inputSpeed + N .* x )./inputTorque - C .* ...
    inputTorqueDerivative ./inputTorque  - ...
    inputSpeedDerivative.*(c1 + 3.*c2.*inputSpeed.^2))./...
    (N .*(c1 + 3.* c2 .* x.^2));
y(isnan(y)) = -N.* x(isnan(y));
y(isinf(y)) = eps;
y =  columnVector(y);
end

function compliance = computeCompliance(reductionRatio, inputSpeed,...
    outputSpeed, c1, c2)
%% COMPUTECOMPLIANCE
assert(c1 ~= 0, 'Compliance coefficient 1 should be non-null!');
assert(all(size(inputSpeed)==size(outputSpeed)),...
    " Input speed vectors must be same size");

if all(all(inputSpeed >= outputSpeed))
    outputSpeed(inputSpeed < outputSpeed)= inputSpeed(inputSpeed < outputSpeed);
end
compliance = c1 .*(inputSpeed + reductionRatio.* outputSpeed)+ c2.* ...
    (inputSpeed + reductionRatio.*outputSpeed).^3;
if any(compliance <= 0)
    compliance = abs(compliance);
end
end