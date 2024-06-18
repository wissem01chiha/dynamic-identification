classdef kinovaGen3 < handle
    %% 
    % Base class model for Kinova Gen3 robot manipulator.
    %% 
    properties
        name
        descriptionFile
        baseFrame
        ndof
        joint
        Geometry
        rigidBodyTree
        inertia
        friction
        samplingFrequency
        controlFrequency
        inertiaParamsVector
    end
    
    methods
        function obj = kinovaGen3(descriptionFile)
            %%
            % Default Class Constructor.
            %%
            if nargin < 1
                descriptionFile = 'robots/kinovaGen3.urdf';
            end
            obj.name = 'kinova Gen3'; 
            obj.descriptionFile = descriptionFile;
            obj.baseFrame.name = 'base_link';
            try
                urdfRobot = parseRobotURDF(obj.descriptionFile);
            catch
                error('Failed to parse the URDF file.');
            end
            obj.rigidBodyTree = urdfRobot;
            obj.ndof = obj.getRobotDOF();
            obj.joint.type = repmat({'revolute'}, 1, obj.ndof);
            obj.Geometry.transformations = zeros(4, 4, obj.ndof);
            
            for i = 1:obj.ndof
                obj.Geometry.transformations(:, :, i) = ...
                obj.getRobotTransformation(i,i + 1);
            end
           
            try
                M = massMatrix(urdfRobot);
            catch
                error('Failed to compute the mass matrix.');
            end
            
            obj.inertia.massMatrix = M(1:obj.ndof, 1:obj.ndof);
            obj.inertia.linksInertia = zeros(1, 6, obj.ndof);
            obj.inertia.mass = zeros(1, obj.ndof);
            
            for i = 1:obj.ndof
                obj.inertia.linksInertia(:, :, i) = obj.getLinkInertia(i);
                obj.inertia.mass(i) = obj.getLinkMass(i);
            end
            obj.samplingFrequency = 1000;
            obj.inertia.gravity.value = urdfRobot.Gravity(3);
            obj.inertia.gravity.vector = urdfRobot.Gravity;
            obj.inertia.COM = centerOfMass(urdfRobot);
            
            obj.friction.lugre.Fc = [40.0; 1.02; 1.025; 1.01; 3.05; 0.07; 1.025];
            obj.friction.lugre.vs = 0.135.*ones(1,obj.ndof);
            obj.friction.lugre.Fs = [1.01; 0.200; 1.150; 1.20; 1.01230; 0.81; 0.1];
            obj.friction.lugre.sigma0 = [1.02; 1.0; 3.1; 2.02; 1.01; 0.01; 0.01];
            obj.friction.lugre.sigma1 =[0.91; 0.0; 0.17; 0.01; 0.052; 2.123; 1.05];
            obj.friction.lugre.sigma2 = [1.91; 1.71; 1.0; 1.0; 1.0; 2.0; 1.52];
            obj.friction.lugre.tspan = 0.001;
            obj.friction.lugre.z0 = 0.01.*ones(1, obj.ndof);
            
            obj.friction.viscous.Fc = [1.25 2.36 2.3 2.9 1.2 2.1 1];
            obj.friction.viscous.Fs = [2.1 0 2.3 1.5 2.6 3.9 2.5];
            
            obj.friction.maxwell.n = 3;
            obj.friction.maxwell.k = 100.*ones(1,obj.friction.maxwell.n); 
            obj.friction.maxwell.c = 10.*ones(1, obj.friction.maxwell.n);
            obj.friction.maxwell.sigma0 = 1;
            
            obj.joint.position.max = pi * ones(1, obj.ndof);
            obj.joint.position.min = -pi * ones(1, obj.ndof);
            obj.joint.velocity.max = [];
            obj.joint.acceleration.max = [];
            
            obj.samplingFrequency = 1000;
            obj.controlFrequency = 1;
            
            obj.joint.stiffness = [1.31 0.021 0.31 0.51 0.01 0.01 0.01];
            
            obj.joint.actuator.reductionRatio = 100.*ones(1, obj.ndof);
            obj.joint.actuator.inertia = 0.00558 .* ones(1, obj.ndof);
            obj.joint.actuator.torqueConstant = 0.11 .* ones(1, obj.ndof);
            obj.joint.actuator.velocityConstant = 84 .* ones(1, obj.ndof);
            obj.joint.actuator.damping = 0.01 .* ones(1, obj.ndof);
            obj.joint.actuator.Ta  = 0.012;
            obj.joint.actuator.Tb  = 0.021;
            obj.joint.actuator.Tck = [0.0198, 0.0111, 0.0158, 0.0123, 0.0125];
            
            obj.inertiaParamsVector = zeros(7 * obj.ndof, 1);
            for i = 1:obj.ndof
                m = obj.getLinkMass(i);
                I = obj.getLinkInertia(i);
                obj.inertiaParamsVector(obj.ndof*i-6:obj.ndof*i) =[m; I(:)];
            end
        end
        
        function vector = getRobotParams(obj, varargin)
            %%
            % GETROBOTPARAMS() Returns the 224 * 1 robot full parameters vector
            %
            % GETROBOTPARAMS( N ) Returns the 32 * 1 parameters vector of
            % robot link number N
            %%
            vector = [];
            if isempty(varargin)
                for i = 1:obj.ndof
                    m = obj.getLinkMass(i);
                    I = obj.getLinkInertia(i);
                    params = [m; I'; obj.friction.lugre.Fc(i); ...
                      obj.friction.lugre.Fs(i); obj.friction.lugre.sigma0(i); ...
                      obj.friction.lugre.sigma1(i); obj.friction.lugre.sigma2(i); ...
                      obj.friction.lugre.z0(i); obj.friction.viscous.Fc(i); ...
                      obj.friction.viscous.Fs(i); obj.friction.maxwell.n; ...
                      obj.friction.maxwell.k'; obj.friction.maxwell.sigma0; ...
                      obj.joint.stiffness(i); obj.joint.actuator.reductionRatio(i); ...
                      obj.joint.actuator.inertia(i); obj.joint.actuator.torqueConstant(i); ...
                      obj.joint.actuator.damping(i);obj.joint.actuator.Ta'; ...
                      obj.joint.actuator.Tb'; obj.joint.actuator.Tck'];
                  vector = [vector; params];
                end
            else
                linkId = varargin{1};
                m = obj.getLinkMass(linkId);
                I = obj.getLinkInertia(linkId);
                vector = [m; I(:); obj.friction.lugre.Fc(linkId); ...
                  obj.friction.lugre.Fs(linkId); obj.friction.lugre.sigma0(linkId); ...
                  obj.friction.lugre.sigma1(linkId); obj.friction.lugre.sigma2(linkId); ...
                  obj.friction.lugre.z0(linkId); obj.friction.viscous.Fc(linkId); ...
                  obj.friction.viscous.Fs(linkId); obj.friction.maxwell.n; ...
                  obj.friction.maxwell.k'; obj.friction.maxwell.sigma0; ...
                  obj.joint.stiffness(linkId); obj.joint.actuator.reductionRatio(linkId); ...
                  obj.joint.actuator.inertia(linkId); obj.joint.actuator.torqueConstant(linkId); ...
                  obj.joint.actuator.damping(linkId); obj.joint.actuator.Ta'; ...
                  obj.joint.actuator.Tb'; obj.joint.actuator.Tck'];
            end
        end
        
        function obj = updateRobotParams(obj, params, varargin)
            %%
            % UPDATEROBOTPARAMS
            %
            % Inputs:
            %   robot  - MATLAB rigidBodyTree struct.
            %   params - Links inertia paramter vector ( 7 * ndof ).
            %%  
            n = length(params);
            if isempty(varargin)
                assert(n == 224, "Parameter vector length should be 224");
                for i=1:obj.ndof
                    baseIndex = (i-1)*7 + 1;
                    mi = params(baseIndex);
                    Ii = params(baseIndex+1:baseIndex+6);
                    obj = updateLinkMass(obj, i, mi);
                    obj = updateLinkInertia(obj, i, Ii);
                    obj.friction.lugre.Fc(i) = params(baseIndex + 7);
                    obj.friction.lugre.Fs(i) = params(baseIndex + 8);
                    obj.friction.lugre.sigma0(i) = params(baseIndex + 9);
                    obj.friction.lugre.sigma1(i) = params(baseIndex + 10);
                    obj.friction.lugre.sigma2(i) = params(baseIndex + 11);
                    obj.friction.lugre.z0(i) = params(baseIndex + 12);
                    obj.friction.viscous.Fc(i) = params(baseIndex + 13);
                    obj.friction.viscous.Fs(i) = params(baseIndex + 14);
                    obj.friction.maxwell.n = params(baseIndex + 15);
                    obj.friction.maxwell.k = params(baseIndex + 16);
                    obj.friction.maxwell.sigma0 = params(baseIndex + 17);
                    obj.joint.stiffness(i) = params(baseIndex + 18);
                    obj.joint.actuator.reductionRatio(i) = params(baseIndex + 19);
                    obj.joint.actuator.inertia(i) = params(baseIndex + 20);
                    obj.joint.actuator.torqueConstant(i) = params(baseIndex + 21);
                    obj.joint.actuator.damping(i) = params(baseIndex + 22);
                    obj.joint.actuator.Ta(i) = params(baseIndex + 23);
                    obj.joint.actuator.Tb(i) = params(baseIndex + 24);
                    obj.joint.actuator.Tck(i) = params(baseIndex + 25);
                end
            else
                linkId = varargin{1};
                assert(n == 32, "Parameter vector length should be 32");
                baseIndex = (linkId-1)*7 + 1;
                mi = params(baseIndex);
                Ii = params(baseIndex+1:baseIndex+6);
                updateLinkMass(obj.robot, linkId, mi);
                updateLinkInertia(obj.robot, linkId, Ii);
                obj.friction.lugre.Fc(linkId) = params(baseIndex + 7);
                obj.friction.lugre.Fs(linkId) = params(baseIndex + 8);
                obj.friction.lugre.sigma0(linkId) = params(baseIndex + 9);
                obj.friction.lugre.sigma1(linkId) = params(baseIndex + 10);
                obj.friction.lugre.sigma2(linkId) = params(baseIndex + 11);
                obj.friction.lugre.z0(linkId) = params(baseIndex + 12);
                obj.friction.viscous.Fc(linkId) = params(baseIndex + 13);
                obj.friction.viscous.Fs(linkId) = params(baseIndex + 14);
                obj.friction.maxwell.n = params(baseIndex + 15);
                obj.friction.maxwell.k = params(baseIndex + 16);
                obj.friction.maxwell.sigma0 = params(baseIndex + 17);
                obj.joint.stiffness(linkId) = params(baseIndex + 18);
                obj.joint.actuator.reductionRatio(linkId) = params(baseIndex + 19);
                obj.joint.actuator.inertia(linkId) = params(baseIndex + 20);
                obj.joint.actuator.torqueConstant(linkId) = params(baseIndex + 21);
                obj.joint.actuator.damping(linkId) = params(baseIndex + 22);
                obj.joint.actuator.Ta(linkId) = params(baseIndex + 23);
                obj.joint.actuator.Tb(linkId) = params(baseIndex + 24);
                obj.joint.actuator.Tck(linkId) = params(baseIndex + 25);
            end
        end

        
        function obj = updateLinkMass(obj, linkNum, m)
            %% 
            % Updates the robot mass paramters value for a given link.
            %
            % Returns:
            %   error - 1 if the update failed else 0.
            %% 
            assert(length(m)== 1,"Input mass should be a positive scalar ");
            linkCellArray = obj.rigidBodyTree.Bodies(linkNum);
            linkRigidBody = linkCellArray{1};
            try
                linkRigidBody.('Mass') = m;
            catch
                
            end
        end
        
        function  obj= updateLinkInertia(obj, linkNum, inertiaVector)
            %% 
            % Update the robot inertia paramters values for a given link.
            %
            % Returns:
            %   error - 1 if the update failed else 0.
            %%
            assert(length(inertiaVector)== 6,...
                "Inertia vector length should be equal to 6");
            linkCellArray = obj.rigidBodyTree.Bodies(linkNum);
            linkRigidBody = linkCellArray{1};
            
            try
                linkRigidBody.('Inertia') = inertiaVector;
            catch
                
            end
        end

        function ndof = getRobotDOF(obj)
            %%
            %  Get the Number of Freedom Degrees from a rigidBodyTree
            %  model.
            %  Any joint within the URDF file with the pattern 'joint_X'
            %  where X = 1,2,... and of type revolute is considered a DOF.
            %%
            ndof = 0;
            for i =1:obj.rigidBodyTree.NumBodies
                jointType = obj.rigidBodyTree.Bodies{i}.Joint.Type;
                jointName = obj.rigidBodyTree.Bodies{i}.Joint.Name;
                jointPattern = '^joint_\d+$';
                if strcmp(jointType, 'revolute') && ...
                    ~isempty(regexp(jointName, jointPattern, 'once'))
                     ndof = ndof + 1;
                end
            end
        end
        
        function p = getLinkPosition(obj,LinkNum,varargin)
            %% 
            % Returns the 
            %%
            if numel(varargin)== 0
                
            else
                
            end
            
        end
        
        
        
        
        
        function R = getLinkRotation(obj,LinkNum,varargin)
            %%
            % Returns the 3x3 euler rotation matrix of the manipulator 
            % link given by LinkNum 
            %%
            if numel(varargin)== 0
                HT = obj.getRobotTransformation(obj,LinkNum,...
                    LinkNum + 1);
                R = HT(1:3,1:3);
            elseif numel(varargin) == 1
                jointsPosition = varargin{1};
                HT = obj.getRobotTransformation(obj,LinkNum,...
                    LinkNum + 1,jointsPosition);
                R = HT(1:3,1:3);
            end  
        end   
        
        function transformationMatrix = getRobotTransformation(obj,...
                startLinkNum, endLinkNum, varargin)
            %%
            % Returns the 4x4 homogeneous transformation matrix from the 
            % robot URDF file description.
            %
            % GETROBOTTRANSFORMATION(ROBOT, I, K) : Computes the 
            % transformation between link I and K for the home configuration.
            %
            % GETROBOTTRANSFORMATION(ROBOT, I, K, X) : Computes the 
            % transformation between link I and K for the joint configuration
            % given by the vector X.
            %
            % Inputs:
            %   startLinkNum  - The number (or index) of the starting link 
            %                   in the robot's
            %                   kinematic chain, from which the transformation 
            %                   is calculated.
            %   endLinkNum    - The number (or index) of the ending link in 
            %                   the robot's
            %                   kinematic chain, to which the transformation 
            %                   is calculated.
            %   X             - (Optional) A vector representing the joint 
            %                   configuration. If not
            %                   provided, the home configuration is assumed.
            %
            % Outputs:
            %   T             - A 4x4 homogeneous transformation matrix representing the
            %                   transformation from the start link to the end link in the
            %                   robot's kinematic chain.
            %%
            if numel(varargin) == 0
                config = homeConfiguration(obj.rigidBodyTree);
                transformationMatrix = getTransform(obj.rigidBodyTree,config,...
                    string(obj.rigidBodyTree.BodyNames(startLinkNum))...
                    ,string(obj.rigidBodyTree.BodyNames(endLinkNum)));
            elseif numel(varargin) == 1
                jointsPosition = varargin{1};
                config = homeConfiguration(obj.rigidBodyTree);
                if length(jointsPosition) ~= length(config)
                    error(...
                   'The number of joint angles must match the robot joints.');
                end
                for i = 1:length(config)
                    config(i)= jointsPosition(i);
                end
                transformationMatrix = getTransform(obj.rigidBodyTree,config,...
                    string(obj.rigidBodyTree.BodyNames(startLinkNum)),...
                    string(obj.rigidBodyTree.BodyNames(endLinkNum)));
            else
                error("Unsupported number of inputs!");
            end
        end

        function m = getLinkMass(obj, linkNum)
            %%
            % Return the link mass given by link number.
            %%
            linkRigidBody = obj.rigidBodyTree.Bodies{linkNum};
            m = linkRigidBody.Mass;
        end
      
        function I = getLinkInertia(obj, linkNum)
            %%
            % Returns link inertia elements I = [Ixx Iyy Izz Iyz Ixz Ixy].
            %%
            linkRigidBody = obj.rigidBodyTree.Bodies{linkNum};
            I = linkRigidBody.Inertia;
        end
    end
end