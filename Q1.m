% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = Q1(rob,q1,q2,sphereCenter,sphereCenter2,sphereCenter3,r)
 vec = 0:0.01:1;
    m = size(vec,2);
q=repmat(q2'-q1',1,m).* repmat(vec,6,1) + repmat(q1',1,m);
for i=1:m
    a(i)=robotCollision(rob,q(:,i),sphereCenter,r);
end 
for i=1:m
    b(i)=robotCollision(rob,q(:,i),sphereCenter2,r);
end
for i=1:m
    c(i)=robotCollision(rob,q(:,i),sphereCenter3,r);
end
    if sum(a+c+b) == 0
        collision = 0;
    else
        collision = 1;
    end
end

