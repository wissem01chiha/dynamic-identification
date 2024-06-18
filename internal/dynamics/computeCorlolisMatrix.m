function [C, N] = computeCorlolisMatrix(robot, Q_t, dQ_dt)
%% 
% Computes the Coriolis forces matrix.
%
% Inputs:
%    robot - robot model MATLAB struct.
%    Q_t   - Joint positions at time t.
%    dQ_dt - Joint velocities.
%
% Returns:
%   C  - Coriolis forces matrix.
%
%% 
timeStep = 1/robot.samplingFrequency;
ndof     = robot.ndof;
M_t      = massMatrix(robot.rigidBodyTree, Q_t);  
Q_t_1    = discreteTimeIntegral(dQ_dt, timeStep) + Q_t;
M_t_1    = massMatrix(robot.rigidBodyTree, Q_t_1);
diff_M   = ( M_t_1 - M_t )./timeStep;

C = zeros(ndof);
for i = 1:ndof
    for j = 1:ndof
        for k = 1:ndof
            Christoffel = 1/2.*( diff_M(i, j) + diff_M(i, k) ...
                - diff_M(j,k) );
            C(i,j) = C(i,j)+ Christoffel .* dQ_dt(k);
        end
    end
end 
N = M_t(1:ndof,1:ndof) - 2.* C ;
end