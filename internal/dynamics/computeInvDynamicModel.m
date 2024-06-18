function torque = computeInvDynamicModel(robot, Q_t, dQ_dt, d2Q_d2t,varargin)
%% 
%  COMPUTEINVDYNAMICMODEL
%
% given the robot stucture model and the trajectory motion parameters
% i.e ( position, velocity and accelartion) it compute the joint torques
% using either the explict dynamic equation or from lagrange equation 
% derivation.   
% 
% 
% Options:
%     RNE            - compute the inverse dynamic model using the recursive
%                      newton euler alogrithm
%     LAGRANGE       - Compute the inverse dynamic model using the lagrange
%                      formulation 
%     friction       - Include the friction forces effect in the model 
%     stiffness      - Include joints stiffness effect in the model 
%     actuatorTorque - Include joints actuator torques.
%
%%
includeFriction = true;   
includeStiffness = true;
includeActuatorTorque = true;
method = 'LAGRANGE';

p = inputParser;
addOptional(p,'friction', includeFriction);
addOptional(p,'actuator', includeActuatorTorque);
addOptional(p,'stiffness', includeStiffness);
addOptional(p,'method',method);

parse(p, varargin{:});
includeFriction = p.Results.friction;
includeStiffness = p.Results.stiffness;
method = p.Results.method;

if strcmp(method,'RNE')
    M = recursiveNewtonEuler(robot, Q_t, dQ_dt, d2Q_d2t); 
    C = recursiveNewtonEuler(robot, Q_t, dQ_dt, d2Q_d2t);
    G = recursiveNewtonEuler(robot, Q_t, dQ_dt, d2Q_d2t);
    baseTorque = M * d2Q_d2t' + C * dQ_dt' + G';
elseif strcmp(method,'LAGRANGE')
    M = massMatrix(robot.rigidBodyTree,Q_t);
    C = computeCorlolisMatrix(robot, Q_t, dQ_dt);
    G = gravityTorque(robot.rigidBodyTree,Q_t);
    baseTorque = M * d2Q_d2t' + C * dQ_dt' + G';
end

if includeStiffness
    K = computeStiffnessMatrix(robot);
    torque = baseTorque + K * Q_t';
end

if includeFriction
    frictionTorques = computeFrictionTorque(robot, dQ_dt);
    torque = baseTorque - frictionTorques';
end

if includeActuatorTorque
    for i = 1:robot.ndof
        [Td,~]= BLDC(Q_t(i), dQ_dt(i),d2Q_d2t(i),...
            'inertia',robot.joint.actuator.inertia(i),...
            'torqueConstant',robot.joint.actuator.torqueConstant(i),...
            'damping',robot.joint.actuator.damping(i),'Ta',...
            robot.joint.actuator.Ta(i),'Tb',robot.joint.actuator.Tb(i),...
            'Tck',robot.joint.actuator.Tck);
        torque(i) = torque(i) + Td;
    end
end
end