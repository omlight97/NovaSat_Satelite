function [Torque]=Allowed_torque(Commanded_Torque, Stored_Angular, TorqueBox_data)
n=size(TorqueBox_data,1); %number of rows in our TorqueBox_data
target_i_max=1; %dummy starter i max
target_max=abs(TorqueBox_data(1,2)-Stored_Angular); %dummy max of the function
target_i_min=1; %dummy starter i min
target_min=abs(TorqueBox_data(1,3)-Stored_Angular); %dummy min of the function

%linear search for upper and lower bounds in the data for a given stored angular momentum

for i=1:n
    if(abs(TorqueBox_data(i,2)-Stored_Angular)<target_max && TorqueBox_data(i,3)>0)
        target_i_max=i;
        target_max=abs(TorqueBox_data(i,2)-Stored_Angular);
    end
    if(abs(TorqueBox_data(i,2)-Stored_Angular)<target_min && TorqueBox_data(i,3)<0)
        target_i_min=i;
        target_min=abs(TorqueBox_data(i,2)-Stored_Angular);
    end
end
%if the commanded torque is between the bounds the return the commanded
if(Commanded_Torque<TorqueBox_data(target_i_max,3)&&Commanded_Torque>TorqueBox_data(target_i_min,3))
    Torque= Commanded_Torque;
else
    %if is not between the bounds return the upper bound if its commanded torque is positive or the lower bound if negative
    if(Commanded_Torque<0)
        Torque= TorqueBox_data(target_i_min,3);
    else
        Torque= TorqueBox_data(target_i_max,3);
    end
end
return
end