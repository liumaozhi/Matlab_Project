% Smooth path given in qMilestones
% input: qMilestones -> nx4 vector of n milestones. 
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xm vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You should output a number of
%                    milestones m<=n.
function qMilestonesSmoothed = smooth(rob,qMilestones,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius)
%to find short path from end to beginning

s=size(qMilestones,1);

loc=[];
while s>1
    for i=1:s-1
       
        if Q1(rob,qMilestones(i,:),qMilestones(s,:),sphereCenter,sphereCenter2,sphereCenter3,sphereRadius)==0
            break 
        end 
    end 
    loc(end+1)=i;
    s=i;
end 

loc= sort(loc);

for n=1:length(loc)
    P_sh(n,:) = qMilestones(loc(n),:);
end

qMilestonesSmoothed=P_sh;
qMilestonesSmoothed(end+1,:)=qMilestones(end,:);
        

end