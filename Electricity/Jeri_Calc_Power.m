function [Power, next_charge] = Jeri_Calc_Power(Params, Alpha_angle, States, current_charge, dt, Flags)

Logic_state = States.Logic;
Control_state = States.Control;
is_it_day = Flags.Day;


switch Logic_state
    
    
    case 'SafeMode'
        if is_it_day
            Mode_data = Params.power_budget('cruise_safe_day',:);
        else
            Mode_data = Params.power_budget('cruise_safe_night',:);
        end
        
        
    case 'operational'
        
        switch Control_state
            
            case 'Communication'
                Mode_data = Params.power_budget('Communication',:);
                
            case 'Max_Sun'
                Mode_data = Params.power_budget('Max_Sun',:);
                
            case 'Payload_Mission'
                Mode_data = Params.power_budget('Payload_Mission',:);
                
            case 'Orbit_Correction'
                Mode_data = Params.power_budget('Orbit_Correction',:);
                
            case 'Attitude_Correction'
                Mode_data = Params.power_budget('Attitude_Correction',:);
                
            case 'Momentum_Unload'
                Mode_data = Params.power_budget('Momentum_Unload',:);
                
            case 'Eco'
                Mode_data = Params.power_budget('Eco',:);
        end
        
end

for par = Params.power_budget.Properties.VariableNames
    Power.(char(par)) =  Mode_data(:,par).Variables;
end

total_power_current_mode = Mode_data(:,'Total_Power').Variables;

if is_it_day
    Power_Production = Params.vertical_power_produc*cos(Alpha_angle);
else
    Power_Production = 0;
end
Total_Power = Power_Production - total_power_current_mode; %[w] Power available to charge the batteries


if current_charge + Total_Power*dt*Params.sec_to_hour < Params.Max_capacity % check for overcharge
    next_charge = current_charge + Total_Power*dt*Params.sec_to_hour;
else
    next_charge = Params.Max_capacity;
end

Power.Total_Power = Total_Power;
Power.Production  = Power_Production;
end
