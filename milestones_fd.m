% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.

function qMilestones = milestones_fd(rob,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius,qStart,xGoal)
R=[1 0 0; 0 1 0; 0 0 1];
Tg=[R xGoal;0 0 0 1];
qGoal=rob.ikine(Tg,qStart,[1,1,1,0,0,0]);
qMilestones = [];
T = {};
T = Addnode(qStart,0,T);

for i=1:1:inf
    T=extend(rob,T,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius);
    if Q1(rob,T{end}.c,qGoal,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius)==0;
      T = Addnode(qGoal,length(T),T);
      m = length(T);
      while (m>0)
          qMilestones = [T{m}.c;qMilestones];
          m = T{m}.iPrev;
          if m<=0
              return
          end
      end
    end
end

    function T=Addnode(config,iPrev,T)
        node.c=config;
        node.iPrev=iPrev;
        T{end+1}=node;
    end
    function qrand=randconfig()
        dq=-pi+rand(1,6)*2*pi;
        if robotCollision(rob,dq,sphereCenter3,sphereRadius)+robotCollision(rob,dq,sphereCenter2,sphereRadius)+robotCollision(rob,dq,sphereCenter,sphereRadius)==0;
            qrand=dq;
        else
            qrand=randconfig();
        end
    end
    function T=extend(rob,T,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius)
        n=length(T);
        qtarget=randconfig();
        
        for m=1:1:n
            d(m)=distance(T{m}.c,qtarget);
        end
        [x,y]=min(d);
        mini=y;
        qnew=T{mini}.c+0.2*(qtarget-T{mini}.c)/norm(qtarget-T{mini}.c);
              
        if Q1(rob,T{mini}.c,qnew,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius)==0
            T=Addnode(qnew,mini,T);
        end
    end
    function d=distance(q1,q2)
    q=q2-q1;
    d=sqrt(sum(q.^2));
    end
end
