function [position_com_time] = Get_position_SumAccess(Access_sum_Time,Inmarsat_postion)
max_comm=zeros(1,length(Access_sum_Time(:,1)));
position_com_time=zeros(length(Access_sum_Time(:,1)),6);
for i=1:length(Access_sum_Time(:,1))
    b=[Access_sum_Time(i,1),Access_sum_Time(i,2),Access_sum_Time(i,3)];
    max_comm(i)=max([Access_sum_Time(i,1),Access_sum_Time(i,2),Access_sum_Time(i,3)]);
    index=find(b==max_comm(i),1);
    position_com_time(i,1)=index;
    position_com_time(i,2:5)=[max_comm(i),Inmarsat_postion(i,3*index-2:3*index)];
    position_com_time(i,6)=sqrt(position_com_time(i,3)^2+position_com_time(i,4)^2+position_com_time(i,5)^2);
end
end