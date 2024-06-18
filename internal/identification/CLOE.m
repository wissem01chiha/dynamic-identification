function[cloe_error,betaOpt,Ys,torque]= CLOE(robot,Q_d,Q,Kp,Ki,Kd,varargin)
%% CLOE 
% Classical Closed Loop Output Error Identification  Algorithm.
% 
% Inputs:
%   robot   - Robot array struct.
%   N       - Sampling Number of Trajectory points.
%   Q_d     - Desired joint postion vector.             ( 2.ndof * N )
%   Q       - The vector of actual joint position.      ( 2.ndof * N )
%   Kp      - Propertiel controller gain matrix.        ( 2.ndof * 2.ndof )
%   Ki      - Intergator controller gain matrix.        ( 2.ndof * 2.ndof )
%   kd      - Derivative controller gain matrix.        ( 2.ndof * 2.ndof )
%
% Returns:
%   betaOpt    - the robot paramter vector estimated by the CLOE
%                alogrithm.                             ( 39.ndof * 1 )
%   J          -                                        ( N * 1 )
%   error      -                                        ( 2.ndof * N )
%  
% The minimization of algorithm error is a nonlinear LS-optimization
% problem solved by running iterative algorithms such as the gradient 
% or Newton methods which are based on a first- or a second-order 
% Taylor�s expansion of 
% the value function J(?).
% The unknown parameters are therefore updated iteratively so that the 
% simulated model output fits the measured system output.
% we need the DDM (Direct dynamic model for the robot) to compute the
% joints simulated Q_s vector, solution of the non linear diffrential 
% equation:
%
%  DDM : M*Qs_ddot + C*Qs_dot + K*Qs + G = torque - friction
% 
% torque feeded to the DDM is obtained from the controller 
% law function give by :
%    (2): torque = G(s) *(Q_d - Q_s)
%
% to solve the DDM given the torque tau computed from comptroller 
% Please note that alternative optimization methods using the
% derivative-free Nelder�Mead nonlinear simplex method,
% Genetic Algorithm (GA), 
% Particle
%  Another approach is the direct and inverse dynamic
%identification models (DIDIM) method, which is a closed-loop
%output error (CLOE) method minimizing the quadratic error
% between the actual and simulated joint torques.
% h the usual CLOE method, which
% minimizes the quadratic error between the actual and simulated
% joint positions.
% Swarm Optimization (PSO) can potentially be used to tackle this issue
% for the PSO the eroor of an iteration is computed as the norm2 of the 
% error of each particle k commited at the iteration 
% the goal is to find the opitmal parmter beta that when runing the
% function 
% [error,~,Ys,~]= computePredictionParams(robot, Q_d, Q, Kp, Ki, Kd);
% gives the minimal error values for all time eg error(:,j) is minal bound
% for any other error(:,j) computed at each time setp j a tolerance 
% is fixed so if the error at j inf the tol then it passed to next step and
% return the simulated value that allow the tol achived 
% Ref:
%   Comparison Between the CLOE Method and the DIDIM Method for Robots 
%   Identification - A.Janot, M.Gautier, A.Jubien, P.O.Vandanjon IEEE-2014.
%
%   Closed-Loop Output Error Identification Algorithms for Nonlinear Plants
%   I.D.Landau, B.D.O.Anderson, F.De Bruynel-1999. 
%
%   Inertial Parameter Identification in Robotics: A Survey.
%   Q.Leboutet, J.Roux, A.Janot, J.Rogelio, G.Cheng-2021.
%
% Author: Wissem CHIHA
% Email: chihawissem08@gmail.com
% Last Revision: 11-04-2024
%% 
 if( numel(varargin) == 0 )
     disp("Optimisiation Disabled,Computing Prediction Parmters...");
    [cloe_error,~,Ys,torque]=computePredictionParams(robot,Q_d,Q,Kp,Ki,Kd);
    betaOpt = getRobotParamsVector(robot);
    
 elseif( numel( varargin ) >= 1 )     
     if( varargin{1} == "PSO" ) 
         disp("using Particle Swarm Optimization...");
         if numel(varargin) == 1 
             disp("using Default Paramters...");
             num_particles  = 10;
             num_iterations = 10;  
             c = [2.2 1.8]; 
             r=[1.78 1.125];
             w  = 2.7; 
         elseif  numel( varargin ) == 2 
             num_iterations = varargin{2};
             assert( varargin{2} > 1,...
                 "Iterations Number Must be Positive Integer > 1");
             num_particles  = 5; 
             c = [2.2 1.8]; 
             r=[1.78 1.125];
             w  = 5.222;
         elseif  numel(varargin) == 3 
             num_iterations = varargin{2};
             num_particles  = varargin{3};
             assert( varargin{3} > 0,...
                 "Particles Number Must be Positive Integer !");
             c = [2.2 1.8]; 
             r = [1.78 1.125];
             w  = 1.22;
         elseif numel(varargin) == 4 
             num_iterations = varargin{2};
             num_particles  = varargin{3};
             w              = varargin{4};
         elseif numel(varargin) == 5 
             num_iterations = varargin{2};
             num_particles  = varargin{3};
             w              = varargin{4};
             r              = varargin{5};
         elseif numel(varargin) == 6
             num_iterations = varargin{2};
             num_particles  = varargin{3};
             w              = varargin{4};
             r              = varargin{5};
             c              = varargin{6};
         end
         disp("Initilisation...");
         
         beta = getRobotParamsVector(robot);
         particles_robot = cell(1, num_particles);
         for i=1:num_particles
             particles_robot{i} = robot;
         end 
         rng('default');
         particle_state         = 1.5.*rand(size(beta,1), num_particles);
         particle_velocity      = 2.*rand(size(beta,1), num_particles);  
         particle_best          = zeros(size(beta,1), num_particles);
         particle_global_best   = beta;
         tol_error              = 1e-3;
         [cloe_error,J,~,~]     = computePredictionParams(robot,Q_d,Q,Kp,Ki,Kd);
         
         stop = false;
         if norm(joint_error(cloe_error)) <= tol_error
                 disp("Stop Criteria Resached ! Exsit");
                 stop = true;
                 betaOpt = getRobotParamsVector(robot);
                 [cloe_error,~,Ys]=computePredictionParams(robot,Q_d,...
                     Q,Kp,Ki,Kd);
         end
         i = 2; 
         particle_error = zeros(size(Q,1),size(Q,2),...
             num_particles,num_iterations);
 
         for k = 1:num_particles
         [particle_error(:,:,k,1),~,~,~]=computePredictionParams(...
             robot,Q_d,Q,Kp,Ki,Kd);
         end
         
         while( i <= num_iterations && ~stop)
           disp(['iteration: ' num2str(i) '/' num2str(num_iterations)...
           ', error = ' num2str(norm(joint_error(cloe_error)))...
           ', criteria = ' num2str(max(J)) ...
           ', Time = ' num2str(toc)]);
            for k = 2:num_particles
                
                particle_best(:,k)= beta;
                
                particle_velocity(:,k) = w.*particle_velocity(:,k)...
                    +c(1)*r(1).*(particle_best(:,k)-particle_state(:,k))...
                 +r(2)*c(2).*(particle_global_best-particle_state(:,k));
             
                particle_state(:,k) = particle_state(:,k) + ...
                    particle_velocity(:,k);
                 
                [particles_robot{k},~] = updateRobotParams(...
                    particles_robot{k},particle_state(:,k));
  
                [particle_error(:,:,k,i),~,~,~]=computePredictionParams(...
                    particles_robot{k},Q_d,Q,Kp,Ki,Kd);
                
                if norm(joint_error(particle_error(:,:,k,i))) <= norm(...
                        joint_error(particle_error(:,:,k,i-1)))
                    particle_best(:,k) = particle_state(:,k);
                end 
            end
           if norm(joint_error(particle_error(:,:,k,i))) < norm(...
                   joint_error(particle_error(:,:,k-1,i)))
               particle_global_best = particle_best(:,k);
           end 
           betaOpt = particle_global_best;
           [robotOpt,~] = updateRobotParams(robot,betaOpt);
           [cloe_error,J,Ys,torque] = computePredictionParams(...
            robotOpt,Q_d,Q,Kp,Ki,Kd);
           if norm(joint_error(cloe_error)) < tol_error
               disp("Stop Criteria Resached ! exsit");
               stop = true;
           end 
           i=i+1;
         end
    
     elseif ( varargin{1} == "NLS" )
         disp("Using Non Linear Least Square Optimisation...");
         if numel(varargin) == 1
             disp("using Default Paramters...");
             alpha         = 1.00;
             tol_error     = 1e-3;
             max_iteration = 5;
         elseif numel(varargin)== 2
             alpha         = varargin{2};
             tol_error     = 1e-1;
             max_iteration = 10;
         elseif numel(varargin) == 3
             alpha         =  varargin{2};
             tol_error     =  varargin{3};
             max_iteration = 10;
         elseif numel(varargin)==4
             alpha         = varargin{2};
             tol_error     = varargin{3};
             max_iteration = varargin{4};
         end
         disp("Initilisation...");
         stop = false;
         k = 1;
         
         while  ~stop 
         
         beta_k_1 =  getRobotParamsVector(robot);   
         [~,~,Q_s_k_1,~] = computePredictionParams(robot,Q_d,Q,Kp,Ki,Kd);
         f_k_1 = Q_s_k_1 - Q_d;
         beta_diff = beta_k_1 + 0.0001.*ones(size(beta_k_1));
         [robot_diff,~] = updateRobotParams(robot,beta_diff);
         [~,~,Q_s_diff,~] = computePredictionParams(robot_diff,Q_d,Q,Kp,Ki,Kd);
         f_k_diff = Q_s_diff - Q_d;
         gradient = zeros(size(f_k_1,1),size(beta_k_1,2),size(Q,2));
         for t=1:size(Q,2)
             for i=1:size(f_k_1,1)
                 for j=1:size(beta_k_1,2)
                     gradient(i,j,t)= ( f_k_diff( i, t) - f_k_1( i, t) )/...
                         (beta_diff(j ,1)- beta_k_1(j, 1));
                 end
             end
         end
         descent_vector = zeros(size(beta_k_1,1),size(Q,2));
         for t = 1:size(Q,2)
             Jacobian = gradient(:,:,t);
            if( any(isnan(Jacobian), 'all') || any(isinf(Jacobian), 'all'))
                Jacobian(isnan(Jacobian)) = 0;
                Jacobian(isinf(Jacobian)) = 0;
            end
            descent_vector(:,t)= -1.*pinv(Jacobian)*f_k_1(:,t);
         end
         beta_k = beta_k_1 + alpha.*descent_vector;
         [robot,~] = updateRobotParams(robot, beta_k);
         [err,J,Ys,torque] = computePredictionParams(robot,Q_d,Q,Kp,Ki,Kd);
         
         
         disp(['iteration: ' num2str(k)...
           ', error = ' num2str( norm( beta_k - beta_k_1 ))...
           ', criteria = ' num2str(max(J)) ...
           ', Time = ' num2str(toc)]);
       k = k+1;
         if k > max_iteration
             disp("Maximum Iterations Number Reached ! Exsit... ");
             stop = true;
         end 
         if max(joint_error(err)) <= tol_error  
             disp(['Convergance Criteria Reached ! : ' num2str(max(joint_error(err)))]);
             stop = true;
         end
         end
         betaOpt = getRobotParamsVector(robot); 
         cloe_error = err;
         
     elseif (  varargin{1} == "ML")
         disp("Using Maximum Likhood Optimisation...");
         
         
         
         
     else
         error(...
 "Optimisation Not Supported, Only Supported are PSO, GWO, NLS, Max-Nelder");
         
end
 end
end

function [err,J,Q_s,torque]=computePredictionParams(robot,Q_d,Q,Kp,Ki,Kd)
%%  computePredictionParams 
% Compute the Closed Loop Error Algorithm Paramters.
%
% Inputs:
%     Q_d        - Desired Joints Positions            ( 2.ndof * N )
%     Q          - Actual Joints Poistions             ( 2.ndof * N )
%
% Returns:
%     err        - Joints Position error              ( 2.ndof * N )
%     J          - Objective optimisation function    ( N * 1 )
%     torque     - Torque computed by controller      ( 2.ndof * N )
%     beta     - Robot Static Paramter vector.      ( 39.ndof * 1 )
% Note:
%   -if 
%%
if size(Q, 2) ~= size(Q_d, 2)  ||  size(Q, 1) ~= size(Q_d, 1)
    error("Sizes of Input Matrices do not Match");
end

Q_s    = zeros(size(Q));
err    = zeros(size(Q));
J      = zeros(size(Q,2),1);
torque = zeros(size(Q));

Q_s(:,1) = Q(:,1);
Q_s(:,2) = Q(:,2);
err(:,1) = Q_d(:,1) - Q(:,1);
err(:,2) = Q_d(:,2) - Q(:,2);
if size(Q, 2) > 2
    for i=3:size(Q, 2)
        torque(:,i) = Kp *(Q_d(:,i)- Q(:,i))+...
        Kd *( Q_d(:, i)- Q(:,i) - Q_d(:, i-1) + Q(:,i-1) );
        for k = 2:i
            torque(:,i) = torque(:,i) + Ki*(Q_d(:,k)-Q(:,k-1));
        end
        Q_t   = Q_s(:,i-1);
        Q_t_1 = Q_s(:,i-2);
        Q_s(:,i)= integrateDynamicsModel(robot,Q_t,Q_t_1,torque(:,i),1);
        err(:,i)= Q_d(:,i) - Q_s(:,i);
        J(i,1) = 1/2.*( Q_s(:,i) - Q_d(:,i) )'*( Q_s(:, i) - Q_d(:, i) );
    end
end 

for j=1:size(Q_s,2)
    for  i = 1:size(Q_s,1)
        Q_s(i,j)= wrap2pi(Q(i,j));
    end
end
end
function Q_tt_1 = integrateDynamicsModel(robot,Q_t,Q_t_1,input_torque,...
                                         varargin)
%%  integrateDynamicsModel
% Integrates the robot general direct dynamic model using finite
% differance method.
% 
% Inputs:
%   Q_t  : Joint Position Vector at t time date.  ( 2.ndof * 1 )
%   Q_t_1: Joint Position Vector at t-1 time date.( 2.ndof * 1 )
%
% Options:
%   ts    - step intergration step.
%   tspan - simulation time.
%
% Returns:
%   Q_tt_1: Joint Position Vector Estimation at t+1 time date.
%%
ndof      = robot.degrees_of_freedom;
if(numel(varargin) == 0)
    ts    = 0.001;
    tspan = 10;
elseif(numel(varargin)==1)
    ts = varargin{1};
    tspan = 10;
elseif( numel(varargin)== 2 ) 
    ts    = varargin{1};
    tspan = varargin{2};
else
    error("Too many input arguments.");
end
Qdot      = Q_t - Q_t_1;
friction_torque = computeFrictionTorque(robot, Qdot( 1:ndof ),...
    Qdot( ndof+1:end ), tspan);  
[~,~,~,~,~,~,Qddot] = computeRobotDynamics( robot,Q_t( ndof+1:end )',...
    Q_t( 1:ndof )', Qdot( ndof+1:end )', Qdot( 1:ndof )', input_torque,...
    friction_torque );
Q_tt_1 = Qddot.* ts^2 - Q_t_1 + 2 .* Q_t;
end

function error_vector = joint_error(error_matrix,varargin)
%% joint_error
% given an errror matrix reprsent all joints errors at 1..N time steps
% compute and returns the colun vector of each joint error over time using
% infinte norm 
% if provided an optional integre it returns the rel value 1*1 
% vector of only this
% joint error
if numel(varargin) == 0
    error_vector = zeros(size(error_matrix,1),1);
    for i=1:size(error_matrix,1)
        error_vector (i,1) = max(abs(error_matrix(i,:)));
    end
elseif numel(varargin)==1
    joint_index = varargin{1};
    error_vector  = max(abs(error_matrix(joint_index,:)));
else 
    error("Invalid Number of Inputs");
end
end
function [max_joint_err,min_joint_err,max_time_err,min_time_err,...
    max_relative_joint_err, min_relative_joint_err]=...
    computeErrorBounds(error_samples_matrix,varargin)
%% computeErrorBounds
% max_err_per_joint maxium value of the eoor of a joint "i", of not given
% "i" return the vector for all joints same as min_err_per_joint
% max_err_per_time mxaium error value of a joint recorded all time
% relative_joint_err : compute l erreur rative at time date t (sampel "j")
% l erreu du joint "i" sur e max des errr at same time 
% if joint index not provided compute for all 
if numel(varargin) == 0
    max_joint_err = zeros(size(error_samples_matrix,1),1);
    min_joint_err = zeros(size(error_samples_matrix,1),1);
    relative_joint_err = zeros(size(error_samples_matrix));
    
    max_time_err = max(error_samples_matrix);
    min_time_err = min(error_samples_matrix);
    for i = 1:size(error_samples_matrix,1)
        for j=1:size(error_samples_matrix,2)
             if  max(abs(error_samples_matrix(:,j))) ~= 0
                 relative_joint_err(i,j)= ...
                error_samples_matrix(i,j)./max(abs(error_samples_matrix(:,j)));
             else
                 relative_joint_err(i,j)= 0;
             end
        end
    end
    for i=1:size(error_samples_matrix,1)
       max_joint_err(i,1)= max(error_samples_matrix(i,:));
       min_joint_err(i,1)= min(error_samples_matrix(i,:));
       max_relative_joint_err = max(relative_joint_err(i,:));
       min_relative_joint_err =min(relative_joint_err(i,:)); 
    end
elseif numel(varargin) == 1
    joint_index = varargin{1};
    max_joint_err = max(error_samples_matrix(joint_index,:));
    min_joint_err = min(error_samples_matrix(joint_index,:));
    relative_joint_err = zeros(size(error_samples_matrix,1),1);
    for j=1:size(error_samples_matrix,2)
        relative_joint_err(joint_index,j)= error_samples_matrix(...
            joint_index,j)./max(abs(error_samples_matrix(:,j)));
    end
    max_relative_joint_err = max(relative_joint_err);
    min_relative_joint_err = min(relative_joint_err);
else
    error("Invalid Number of Inputs !");
end

end
function correlation = computeCorrelation(sample_matrix,varargin)
%% computes the correlation between the joints error vectors 
% compute the variance matrix of the sampled matrix 
%

end
function lkh = computeLikehood(error_samples_matrix)
%% computeLikehood 
% compute and return the likehood function values for a given  



end
