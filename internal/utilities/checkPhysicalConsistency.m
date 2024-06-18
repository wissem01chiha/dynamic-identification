function status = checkPhysicalConsistency(robot)
%% ------------------------------------------------------------------------ 
% CHECKPHYSICALCONSISTENCY  
%  Check the physical consistency of the robot parameters
%
% Author : Wissem CHIHA
%% ------------------------------------------------------------------------
status = true;
for i = 1:robot.NumBodies
    if robot.Bodies{i}.Mass <= eps
        disp(['Invalid mass for Body ', num2str(i), ': ', robot.Bodies{i}.Name]);
        disp(['Mass: ', num2str(robot.Bodies{i}.Mass)]);
        status = false;
    else
        bodyInertia = robot.Bodies{i}.Inertia;
        XX = bodyInertia(1);
        YY = bodyInertia(2);
        ZZ = bodyInertia(3);
        YZ = bodyInertia(4);
        XZ = bodyInertia(5);
        XY = bodyInertia(6);
        I = [XX XY XZ; XY YY YZ; XZ YZ ZZ];
        if ~isequal(I, I') || ~all(eig(I) >= eps)
            disp(['Invalid inertia matrix for Body ', num2str(i),...
                ': ', robot.Bodies{i}.Name]);
            status = false;
        end
    end
end
end
