function E=transfer(Ed,V)
E=zeros(2,length(Ed));

i=1;
while i<=length(Ed)
       
         C = cellfun(@(a) norm(a)- norm(Ed{i}.a),V);
  
         
         
        for vali=1:length(C)
            
            if C(vali)==0
               E(1,i)= vali ;
            end
        end
        
        D = cellfun(@(a) norm(a)- norm(Ed{i}.b),V);
        
        for valg=1:length(D)
           
            if D(valg)==0
               E(2,i)= valg; 
            end
        end
        i=i+1;
        
end

E;
end
