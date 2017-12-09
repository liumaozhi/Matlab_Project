function P_pre = short_path(rob,qi, xGoal, k, G,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius,n)
V=G{1};Ed=G{2};
R=[1 0 0; 0 1 0; 0 0 1];
Tg=[R xGoal;0 0 0 1];
qg=rob.ikine(Tg,qi,[1,1,1,0,0,0]);
Nq_init = neighbor(V,qi,5);
Nq_goal = neighbor(V,qg,5);
for i=1:5
    
    if Q1(rob,qi,Nq_init{1,1}.ch{i},sphereCenter,sphereCenter2,sphereCenter3,sphereRadius) == 0
       Ed=Add(qi, Nq_init{1,1}.ch{i},Ed);
     
    end 
    
    if Q1(rob,qg,Nq_goal{1,1}.ch{i},sphereCenter,sphereCenter2,sphereCenter3,sphereRadius) == 0
       Ed=Add(qg, Nq_goal{1,1}.ch{i},Ed);
    end 
    
end 
V=V_Addnode(qi,V);
V=V_Addnode(qg,V);

E=transfer(Ed,V);

E1=E(1,:);
E2=E(2,:);

    
R=graph(E1,E2);

P_path=shortestpath(R,n+1,n+2);
P_pre={P_path, V};

    
    
    
        
    
  

   

function Ed=Add(q,q_dot,Ed)
        node.a=q;
        node.b=q_dot;
        Ed{end+1}=node;

    end

function V=V_Addnode(config,V)
        
        V{end+1}=config;
end 

end 