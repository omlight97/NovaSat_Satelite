function [Access_sum_Times] = GetSumAccess(Access_Inmarsat_Vec)
%sum=zeros(Access_Inmarsat_Vec(:,1),3);
sum=0;
k=0;
i=1;
Access_sum_Times=zeros(length(Access_Inmarsat_Vec(:,1)),3);
for j=1:length(Access_Inmarsat_Vec(1,:))
    while i<length(Access_Inmarsat_Vec(:,1))-1
         while Access_Inmarsat_Vec(i,j)==1 && i<length(Access_Inmarsat_Vec(:,1))-1
             sum=sum+1;
             k=k+1;
             i=i+1;
             if i==length(Access_Inmarsat_Vec(:,1))
                 break;
             end
         end
         if k>1 && i>1
         Access_sum_Times(i-k:i-1,j)=sum;
         end
         sum=0;
         while Access_Inmarsat_Vec(i,j)==0 && i<length(Access_Inmarsat_Vec(:,1))-1
             i=i+1;
             if i==length(Access_Inmarsat_Vec(:,1))
                 break;
             end
         end
       
        k=1;
        
    end
    i=1;
end

end