function G = roadmap(rob,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius,n,k)
V={};
Ed={};

while length(V) < n
    qrand=randconfig();
    if robotCollision(rob,qrand,sphereCenter,sphereRadius) + robotCollision(rob,qrand,sphereCenter2,sphereRadius) + robotCollision(rob,qrand,sphereCenter3,sphereRadius) == 0
        V = V_Addnode(qrand, V);                                                            
    end 
end 

Ed=Add(zeros (1,6),zeros (1,6),Ed);

for i = 1: n
    
    q = V{i};
    Nq = neighbor(V,q,k);
        
    
    for m =1:3
        q_dot=Nq{1,1}.ch{m};
        R = relation(q,q_dot,Ed);
       
        if (R == 0) && (Q1(rob,q,q_dot,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius) == 0)
            
            Ed=Add(q,q_dot,Ed);
            
        end
    
    end
end

Ed=Ed(2:length(Ed));




        
                
G={V,Ed};
    





% the function used for build random configuration
    function qrand=randconfig()
        dq=-pi+rand(1,6)*2*pi;
        if (robotCollision(rob,dq,sphereCenter3,sphereRadius)+robotCollision(rob,dq,sphereCenter2,sphereRadius)+robotCollision(rob,dq,sphereCenter,sphereRadius))==0;
            qrand=dq;
        else
            qrand=randconfig();
        end
    end

% these three functions are built for adding three different nodes
    function V=V_Addnode(config,V)
        
        V{end+1}=config;

    end
    function E=Add(q,q_dot,E)
        node.a=q;
        node.b=q_dot;
   
        E{end+1}=node;

    end
    
    
    function R = relation(q,q_dot,E)
        s=[];
        for i=1:length (E)
            a = q - E{i}.b ;
            b = q_dot - E{i}.a;
            s(i)=norm(a+b);
        end
        f=find (s==0);
        if f > 0
                R = 1;
        else
                R = 0;
        end 
            
               
            
    end
        
end 
