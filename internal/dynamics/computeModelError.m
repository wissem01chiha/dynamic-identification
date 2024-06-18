function error = computeModelError(robot, Q_t, dQ_dt, d2Q_d2t, torques, x)
    %%
    % Computes, for each joint, the root mean square error, of the model 
    % given an input trajectory data, and the robot parmter vecor.
    %
    % Inputs:
    %   robot   - MATLAB array struct
    %   torques - Recorded torques value at time date t             ( 1 * ndof )
    %   Q_t     - Recorded joints positions vector at time date t   ( 1 * ndof )
    %   dQ_dt   - Recorded joints velocity vector at time date t    ( 1 * ndof ) 
    %   d2Q_d2t - Recorded joints accelerations vector at time date t( 1 * ndof )
    %   X       - Robot Paramter vector    ( 32 * ndof , 1) 
    %
    % Returns:
    %   error - RMSE value vector ( 1 * 1 ) 
    %%
    ndof = length(torques);
    robot.updateRobotParams(x);
    simTorques = computeInvDynamicModel(robot, Q_t, dQ_dt, d2Q_d2t);
    assert(length(simTorques)== ndof && all(size(simTorques)==size(torques')));
    error = sqrt(mean((simTorques - torques').^2));
end