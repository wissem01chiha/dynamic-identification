function KALMAN(varargin)
    %%
    % Kalman filtering identification-based algorithm implementation 
    %
    % The goal is to estimate the augmented state system vector 
    % z(t) = [x(t) u(t)], where x(t) is the system state vector variable 
    % and u(t) is the system input torque 
    %
    % Ref:
    %    Constrained State Estimation - A Review - 
    %
    %%
    disp('Running Kalman Filter Identification ...');
    disp('Initialisations ... ');
    defaultMaxSamplesNum = 1940;
    
    defaultDataFilePath = ...
     "datasets/kinova/identification_data/RunTrajectory1.csv";
    
    
    
    robot = kinovaGen3();
    data  = parseRobotData(defaultDataFilePath,1,100,defaultMaxSamplesNum);
    data  = filterRobotData(data);
    
    disp('Start Calculations ...');
    
    
    
    
    xOpt = zeros(3*robot.ndof,1);        
    P = eye(length(xOpt));   
    Q = eye(size(xOpt));   
    R = eye(robot.ndof);   
    num_steps = length(data.time);
    X_estimates = zeros(length(xOpt), num_steps);
    for t = 1:num_steps
        Q_t = data.position(t,:);
        dQ_dt = data.velocity(t,:);
        d2Q_d2t = data.acceleration(t,:);
        [~, xOpt, A, B, C, ~] = computeStateSpaceModel(robot,...
            Q_t, dQ_dt, d2Q_d2t);
        % compute augmented state space model 
        AA = [A B; zeros(2*robot.ndof) eye(2*robot.ndof,size(B,2))];
        size(AA),
        CC = [C zeros(size(C,1))];
        ZPred_t = AA * xOpt ;
        PPred_t = AA * P * AA' + Q;
   
        Y_t = z_t - CC * ZPred_t ;
        S_t = CC * PPred_t * CC' + R;
        K_t = PPred_t * CC' ./ S_t;
        xOpt = ZPred_t + K_t * Y_t;
        P = (eye(size(K_t,1)) - K_t * CC) * PPred_t;
        X_estimates(:, t) = xOpt;
    end
end