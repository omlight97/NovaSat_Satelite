function [Numeric_properties] = Define_Numeric_properties(SunTimes)

% Numeric Properties

Numeric_properties.dt = SunTimes(2)- SunTimes(1); %[s]
Numeric_properties.opts = odeset('Reltol',1e-10,'AbsTol',1e-10,'Stats','off');% tolerance

% Numeric_properties.dt_for_thermal = 5*60; % [s]
Numeric_properties.dt_for_thermal = Numeric_properties.dt;

Numeric_properties.Tol_km = 1; % [km]

end
