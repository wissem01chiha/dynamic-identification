function DIDIM(varargin)
    %%
    % DIDIM
    %
    % Classical Direct and Inverse Dynamic identifcation Alorithm
    % Implementation.The identification problem is formulated as a 
    % non linear optimization problem given by:
    %
    %     XOpt =  argmin(X)|| IDM(Q_t, dQ_dt, d2Q_d2t, X) - tau ||
    %
    % where X is the robot paramter vector, Q_t, dQ_dt, and d2Q_d2t 
    % are the trajectoy state variables ( position, velocity, 
    % acceleration)
    %
    %%
    disp('Running DIDIM identification ...');
    disp('Initlisations ... ');
    defaultDataFilePath   = "datasets/kinova/identification_data/RunTrajectory1.csv";
    defaultMaxSamplesNum  = 1940;
    defaultOptimisation   = 'PSO';
    defaultMaxIterNum     = 10;
    defaultStopErrorLimit = 0.5;

    p = inputParser;
    addOptional(p,'iterations',defaultMaxIterNum);
    addOptional(p,'optimisation',defaultOptimisation);
    addOptional(p,'error',defaultStopErrorLimit);
    parse(p, varargin{:});
    optimisation = p.Results.optimisation;
    robot = kinovaGen3();
    data  = parseRobotData(defaultDataFilePath,1,60,defaultMaxSamplesNum);
    data = filterRobotData(data,'cutoffFrequancy',35);
    X = robot.getRobotParams();
    disp('Start calculations ...');
    if strcmp(optimisation ,'PSO')
        disp('Using particle swarm optimisation...');
        nVars = length(X);
        lb = -2.5 * ones(1,nVars);
        ub =  2.5 * ones(1, nVars);
        options = optimoptions('particleswarm','SwarmSize',3,...
        'MaxIterations',20, 'Display','iter'); 
        optFunc = @(x) computeObjError(robot, data.position, data.velocity...
           ,data.acceleration, data.torque, x);
       [XOpt, ~] = particleswarm(optFunc, nVars, lb, ub, options);
    elseif strcmp(optimisation, 'PATTERNSEARCH')
        disp('Using pattern search optimization...');
        options = optimoptions('patternsearch', 'Display', 'iter', 'MaxIterations', 1);
        X0 = X;  
        nVars = length(X);
        lb = -2 * ones(1,nVars);
        ub =  2 * ones(1, nVars);
        optFunc = @(x) computeObjError(robot, data.position, data.velocity...
           ,data.acceleration, data.torque, x);
        [XOpt, ~] = patternsearch(optFunc, X0, [], [], [], [], lb, ub, [], options);
    end 
       robot.updateRobotParams(XOpt);
       close all;
       simulateDynamicModel(robot);
end


 