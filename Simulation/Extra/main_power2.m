function [Power, next_charge, DOD] = main_power(Params,Current_Step_Angular,current_charge,dt,Flags)

% Electricity parameters for the power bugdet 
[Params] = PowerBudget(Params);

theta_angle = Calc_theta(Params.SunPosition, Current_Step_Angular ,Params.SatPosition);
 if not(Flags.Communication) && not ( Flags.GRB)
    theta_angle=0;
    end
max_charge = Params.Max_capacity; %[Wh]

[Power, next_charge] = Nova_Calc_Power(Params,theta_angle, current_charge, dt, Flags);

DOD = ( max_charge-current_charge)/max_charge;


end