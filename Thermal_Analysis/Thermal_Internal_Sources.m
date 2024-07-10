function [Internal_Q_total,Internal_Q_component] = Thermal_Internal_Sources(Power,Params)

eta = Params.Efficiency; %Constant and equal efficiency for all component. This should be updated! (here and on JeriParams) 

list_of_fields = fieldnames(Power);
ind = find(strcmp(list_of_fields,'Total_Power'));
list_of_fields(ind) = [];
ind = find(strcmp(list_of_fields,'Production'));
list_of_fields(ind) = [];

for i = 1:length(list_of_fields)
    component = list_of_fields(i);
    Internal_Q_component.(component{1}) = Power.(component{1})*eta;
    Internal_Q_component_to_sum(i) = Internal_Q_component.(component{1});
end

Internal_Q_total = sum(Internal_Q_component_to_sum);

% Internal_Q_total_check = -Power.Total_Power*Params.Efficiency;

end