function [Power, next_charge] = Nova_Calc_Power(Params, theta_angle, current_charge, dt, Flags)
%--------------------------------------------------------------------------
% PURPOSE: This function calculates the total power available to charge the batteries, 
%          the state of charge of the batteries and the power produced by the solar panels
%--------------------------------------------------------------------------
% INPUT: Flags for day\nigth, Modes states, theta angle for power production calculations,
%        the current charge of the batteries, and time constant for next charge
%        calculation
%--------------------------------------------------------------------------
% OUTPUT: Total power, power production from the solar panels, 
%         and charge state of the batteries
%--------------------------------------------------------------------------

Day_state = Flags.IsDay;
Comm_state = Flags.Communication;

% according to the given state (day\night\modes), the relevant column from
% the power budget will be extracted for the power and charge calculations

if Day_state 
    Mode_data = Params.power_budget('Sun Cruise',:);

elseif Day_state && Comm_state
    Mode_data = Params.power_budget('Communication',:);

elseif not(Day_state) && Comm_state
    Mode_data = Params.power_budget('Communication',:); 

else 
    Mode_data = Params.power_budget('Night Cruise',:);
end

% switch Logic_state
% 
% 
%     case 'SafeMode'
%         if is_it_day
%             Mode_data = Params.power_budget('Sun Cruise',:);
%         else
%             Mode_data = Params.power_budget('Night Cruise',:);
%         end
% 
% 
%     case 'operational'
% 
%         switch Control_state
% 
%             case 'Communication'
%                 Mode_data = Params.power_budget('Communication',:);
% 
%             case 'GRB event'
%                 Mode_data = Params.power_budget('Payload_Mission',:);
% 
%             case 'Detumbling'
%                 Mode_data = Params.power_budget('Detumbling',:);
% 
%             case 'Commissioning'
%                 Mode_data = Params.power_budget('Commissioning',:);
% 
%             case 'Decomissioning'
%                 Mode_data = Params.power_budget('Decomissioning',:);
% 
%         end
% 
% end

for par = Params.power_budget.Properties.VariableNames
    Power.(char(par)) =  Mode_data(:,par).Variables;
end

% total power column from power budget 
total_power_current_mode = Mode_data(:,'Total_Power').Variables;

% power production calculation
if Flags.IsDay
    Power_Production = Params.vertical_power_produc*cos(theta_angle);
else
    Power_Production = 0;
end

Total_Power = Power_Production - total_power_current_mode; %[w] Power available to charge the batteries


if current_charge + Total_Power*dt*Params.sec_to_hour < Params.Max_capacity % check for overcharge
    next_charge =   Total_Power*dt*Params.sec_to_hour;
else
    next_charge = Params.Max_capacity;
end

Power.Total_Power = Total_Power;
Power.Production  = Power_Production;
end
