function Nq=neighbor(V,q,k)
    N=[];Nq={};
    n=length(V);
    d=zeros(1,n);
    for (i = 1: n)
        d(i) = norm(V{i} - q);
        %calculating the distance among selected configuration and other
        %configurations
    end 
    val = zeros(k+1,1);
    for (m = 1:k+1);
        [val(m),idx{m}]=min(d);
        % remove for the next iteration the last smallest value:
        d(idx{m})=[max(d)];
    end 
    for (a = 1:k)
        
        N{a}=V{idx{a+1}};
    end
    Nq=nodes(N,q,Nq);
    
    function Nq=nodes(children, parent, Nq)
    node.ch=children;
    node.p=parent;
    Nq{end+1}=node;
    end 
end