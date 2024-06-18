function [dx_dt, x, A, B, C, D] = computeStateSpaceModel(robot, q, dq_dt,...
    d2q_d2t, varargin)
    %%  
    % Computes the state space robot model paramters
    %
    % Inputs:
    %   robot   - Robot MATLAB struct Array
    %   q       - joint position vector      ( 1 * ndof )
    %   dQ_dt   - joints velocity vector     ( 1 * ndof )
    %   d2Q_d2t - joints acceleration vector ( 1 * ndof )
    %
    % Returns:
    %   A  - State Space matrix A ( 2 * ndof , 2 * ndof)
    %   B  - State Space matrix B ( 2 * ndof, ndof )
    %   C  - State Space matrix C ( ndof, 2 * ndof )
    %   D  - State Space matrix D ( 2 * ndof, 2 * ndof)
    %
    %%
    ndof = robot.ndof;
    M = massMatrix(robot.rigidBodyTree,q);
    C = computeCorlolisMatrix(robot,q,dq_dt);
    K = computeStiffnessMatrix(robot);
    x     = [q; dq_dt];
    dx_dt = [dq_dt; d2q_d2t];
    A = [zeros(ndof) eye(ndof);-inv(M)*K -inv(M)*C ];
    B =[zeros(ndof); inv(M)];
    C =[eye(ndof) zeros(ndof)];
    D = zeros(2*ndof);
    if rank(obsv(A, C)) ~= size(A, 1)
        disp('The system is not observable.');
        disp('');
        
        
    end
    if rank(ctrb(A, B)) ~= size(A, 1)
        disp('The system is not controllable.');
        
        
        
        
    end
end

