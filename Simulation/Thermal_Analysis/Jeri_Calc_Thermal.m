function Thermal = Jeri_Calc_Thermal(World_Model, Current_Step_Angular, Current_Step_Orbit, Power, Flags, dt, Params, Thermal)

External_Q_total = Thermal_External_Sources(World_Model, Current_Step_Angular, Current_Step_Orbit, Flags, Params);
[Internal_Q_total,Internal_Q_component] = Thermal_Internal_Sources(Power,Params);


%% Final Temperature Calculation

% q_radiator = Params.A_rad * Params.radiator_emissivity * Params.sigma * Thermal.Average_Temperature^4;
% q_net = External_Q_total + Internal_Q_total - q_radiator;

Q_ext_and_int = External_Q_total + Internal_Q_total;
q_radiator = Params.A_rad * Params.radiator_emissivity * Params.sigma * Thermal.Average_Temperature^4;

Thermal.Delta_Average_Temperature = (Q_ext_and_int - q_radiator)*dt/(Params.mass * Params.CP_AL);

% Thermal.Delta_Average_Temperature = q_net *dt /(Params.mass * Params.CP_AL);

Thermal.Average_Temperature = Thermal.Average_Temperature + Thermal.Delta_Average_Temperature;
Thermal.q_net = Q_ext_and_int - q_radiator;
Thermal.Internal_Q_component = Internal_Q_component;
end