function PowerProduction = solar_power_prod_calc(Solar_Area, D, years, theta, Id, I0, mu, Xe, Xd)
%--------------------------------------------------------------------------
% PURPOSE: This function calculate the power production as per the solar
%          panels area and a few other factors and efficiencies
%--------------------------------------------------------------------------
% INPUT: solar panel area, degradation factor, duration of life (years),
%        theta angle (between the pannels and the sun), 
%        Id - inherent degradation, 
%        I0 - total solar irradiance around earth, Solar panel efficiency, 
%        Xe - efficiency of the path eclipse - from the book,
%        Xd - efficiency of the path daylight - from the book
%--------------------------------------------------------------------------
% OUTPUT: Power production for the day and night (assuming they are
%         approximatly equal)
%--------------------------------------------------------------------------


Po_Si = I0*mu ;     %[W/m^2]    I0 = total solar irradiance around earth
%                               mu = Solar panel efficiency

P_BOL = Po_Si*Id*cos(theta);       % [W/m^2]    Id = inherent degradation

P_EOL = P_BOL*(1-D)^years;    % [W/m^2]

P_sa = Solar_Area*P_EOL;    % [W]

PowerProduction = P_sa*Td*(Te/Xe+Td/Xd)^(-1);   % [W]

end
