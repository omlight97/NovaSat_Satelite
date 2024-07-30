function GRB_Alert= Get_GRB_Alert(duration)

% This function creates a vector of zeros based on the duration and 1 in
% the vector location that matches our wanted GRB alert time.

% Inputs:
% Time vector of simulation

% Outputs:
% A vector of zeros and ones that indicates the time of the GRB event

GRB_Start_time=60*60; % Default GRB alert time - after 60 minutes

GRB_Index=find(duration>=GRB_Start_time,1); % Finding index of wanted time

GRB_Alert=zeros(size(duration));
while GRB_Index<length(duration)-10
GRB_Alert(GRB_Index:GRB_Index+10)=1;% sets GRB alert time to 1
GRB_Index=GRB_Index+400;
end

end