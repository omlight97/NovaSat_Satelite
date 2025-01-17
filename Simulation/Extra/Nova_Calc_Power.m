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
power_consumption = Mode_data(:,'Total_Power').Variables;

% power production calculation

if Flags.IsDay
    Power_Production = Params.vertical_power_produc*abs(cos(theta_angle));
    Total_Power = Power_Production - power_consumption/60; %[w] Power available to charge the batteries

    current_charge_next = Total_Power*dt/3600;

    if current_charge + current_charge_next < Params.Max_capacity % check for overcharge
        next_charge = current_charge +  current_charge_next;
    else
    next_charge = Params.Max_capacity;
    end

else
    Power_Production = 0;
   if current_charge == Params.Max_capacity
    % if current_charge < 35
        Total_Power = 180 - power_consumption; %[w] Power available to charge the batteries
    else
         % Total_Power =current_charge/(dt/3600) - power_consumption; %[w] Power available to charge the batteries
         Total_Power =current_charge - power_consumption/60; %[w] Power available to charge the batteries

    end

    current_charge_next = Total_Power*dt/3600;

    if current_charge - current_charge_next < Params.Max_capacity % check for overcharge
        next_charge = current_charge -  current_charge_next;
    else
    next_charge = Params.Max_capacity;
    end

end

Power.Total_Power = Total_Power;
Power.Production  = Power_Production;
end


