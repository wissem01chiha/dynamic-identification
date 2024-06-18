function Q = recursiveNewtonEuler(robot, Q_t, dQ_dt, d2Q_d2t,gravity)
%%  
% RECURSIVENEWTONEULER( ROBOT ) 
% 
% RNE implementation for multibody robotic systems.
%
% Inputs:
%   robot   -  robot structure model 
%   q       -  joints position vector      (1 * n) 
%   qdot    -  joints velocities vector    (1 * n)
%   qddot   -  joints accelerations vector (1 * n)
%   gravity -  gravity acceleration value.
%
% Returns:
%   Q     - The computed torque
%
% Ref:
%
%% 
ndof = robot.ndof;
Q    = zeros(ndof,1);
w    = zeros(3,ndof);
wdot = zeros(3,ndof);
vdot = zeros(3,ndof);
z0 = [0; 0; 1];
for i = 1 : ndof
        R = robot.getLinkRotation(i,Q_t);
        p = robot.getLinkPosition(i);
        
        if (i > 1)
            w(:, i) =  R'*(w(:, i-1) + z0.*dQ_dt(i));
            wdot(:, i) = R'*(wdot(:, i-1) +  z0.*d2Q_d2t(i) + ...
                cross(w(:, i-1), z0.*dQ_dt(i)));
            vdot(:,i) = R'*vdot(:,i-1) + cross(wdot(:,i), p) + ...
                cross(w(:,i), cross(w(:,i),p));
        else
            w(:, i) =  R'*(z0.*dQ_dt(i));
            wdot(:, i) = R'*(z0.*d2Q_d2t(i));
            vdot(:,i) = R'*gravity + cross(wdot(:,i), p) + ...
                cross(w(:,i), cross(w(:,i),p));
        end
end

for i = ndof:-1:1
        p = robot.getLinkPosition(i);
        I = robot.getLinkInertia(i);
        
        vcdot = vdot(:,i) + cross(wdot(:,i),...
            robot.getLinkCOM(i)) + ...
            cross(w(:,i),cross(w(:,i),...
            robot.getLinkCOM(i)));
        
        F = robot.links{i}.mass * vcdot;
        N = I* wdot(:,i)+ cross(w(:,i), I * w(:,i));
        
        if i < ndof
            R = robot.getLinkRotation(i+1, Q_t);
            n(:,i) = R*(n(:,i+1) + cross(R'*p, f(:,i+1))) + ...
                cross(robot.getLinkCOM(i)+p, F) + N;
            f(:,i) = R*f(:,i+1) + F;
        else
            n(:,i) = cross(robot.getLinkCOM(i,Q_t) + p, F) + N;
            f(:,i) = F;
        end
        R = robot.getLinkRotation(i, Q_t);
        if robot.links{i}.type == "prismatic"
            Q(i) = f(:,i)'*R'*z0;
        elseif robot.links{i}.type == "revolute"
            Q(i) = n(:,i)'*R'*z0;
        end
end
end


