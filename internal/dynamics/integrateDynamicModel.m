function [d2Q_d2t, Q_tt_1] = integrateDynamicModel(robot, Q_t, dQ_dt, torque)
%% ------------------------------------------------------------------------
% INTEGRATEDYNAMICMODEL
% Integrates the robot general direct dynamic model using finite
% differance method.
%        d2Q_d2t  = M^(-1)*(T - N(Q, dQ_dt))
% 
% Inputs:
%   Q_t          - Joint Position Vector at t time date. ( ndof * 1 )
%   dQ_dt        - Joint Velocity Vector at t time date. ( ndof * 1 )
%   samplingRate - Sampling frequancy.
%   T_t          - Input torque vector  at t time date   ( ndof * 1 ).
%
% Returns:
%   d2Q_d2t  - Joint Acceleration Vector Estimation at t time date. 
%   Q_tt_1   - Joint Position Vector Estimation at t+1 time date.
% 
%% -----------------------------------------------------------------------
timeStep = 1/robot.samplingFrequancy;
M = massMatrix(robot,Q_t);
[~, N] = computeCorlolisMatrix(robot, Q_t, dQ_dt);
if rcond(M) < 0.001
    
end
d2Q_d2t = inv(M).*(torque - N);
Q_tt_1 = d2Q_d2t.*timeStep^2 - dQ_dt + 2 .* Q_t;
end