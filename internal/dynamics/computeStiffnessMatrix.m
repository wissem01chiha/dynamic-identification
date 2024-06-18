function matrix = computeStiffnessMatrix(robot)
%%  
%
%
%
%% 
matrix = zeros(robot.ndof);
for i=1:robot.ndof
    matrix(i,i) = robot.joint.stiffness(i);
end
end