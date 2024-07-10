function External_Q_total = Thermal_External_Sources(World_Model, Current_Step_Angular, Current_Step_Orbit, Flags, Params)

psi   = Current_Step_Angular.angles(1);
theta = Current_Step_Angular.angles(2);
phi   = Current_Step_Angular.angles(3);
r = Current_Step_Orbit.r;
v = Current_Step_Orbit.v;

sun_vec = World_Model.SunPosition; %TOD
sun_vec = ToD2LVLH(sun_vec,r,v);   %LVLH

Day_flag = Flags.Day;

%% Solar

Q_solar = nan(1,6);
assumed_area = Params.assumed_area;
solar_flux = Params.q_solar_flux;

if Day_flag
    for i =1:6
        normal = Body2LVLH(Params.Normals(:,i), phi, theta, psi);
        normal = make_it_row(normal); %LVLH
        sun_vec = make_it_col(sun_vec);
        sun_vec = sun_vec - make_it_col(r);
        cos_b = normal*sun_vec/(norm(normal)*norm(sun_vec));
        
        Absorptivity = Params.Absorptivity_per_face(i);
        
        if(cos_b > 0)
            Q_solar(i) = Absorptivity * assumed_area * cos_b * solar_flux;
        else
            Q_solar(i) = 0;
        end
    end
    
else
    Q_solar = zeros(1,6);
end

%% IR & Albedo

F_IR_day   = Params.sigma * Params.Temperature_day^4;     % heat flux on the Moon's surface     [Watt/m^2]
F_IR_night = Params.sigma * Params.Temperature_night^4;   % heat flux on the Moon's surface, ~0 [Watt/m^2]
dist       = (Params.R/norm(r))^2;                        % Normalized distance
q_IR_day   = F_IR_day * dist;                             % heat flux on distance r from the center of the Moon [Watt/m^2]
q_IR_night = F_IR_night * dist;                           % heat flux on distance r from the center of the Moon [Watt/m^2]
q_albedo_day   = Params.q_albedo_day;                               % fractional solar flux reflected by the Moon's surface [Watt/m^2]
q_albedo_night = Params.q_albedo_night;

r_temp      = ToD2LVLH(-r,r,v);                           % pointing from the satellite toward the Moon (minus sign)
r_sat2Moon  = r_temp; 

% surface_from_moon = Calc_Surf(Params, sun_vec_in_body, phi, theta, psi);  warning('???')
assumed_area = Params.assumed_area; % TEMP! Calc_Surf replaces this parameter

Q_Lunar = nan(1,6);    % Includes IR and lunar albedo, both reflected\ emitted from the same direction

for i=1:6
    normal = Body2LVLH(Params.Normals(:,i), phi, theta, psi);
    normal = make_it_row(normal); %LVLH
    r_sat2Moon = make_it_col(r_sat2Moon);
    cos_a = normal*r_sat2Moon/(norm(normal)*norm(r_sat2Moon));
        
    if(cos_a > 0) 
        if Day_flag
            q_IR_now = q_IR_day;
            q_albedo = q_albedo_day;
        else
            q_IR_now = q_IR_night;
            q_albedo = q_albedo_night;
        end
        Q_Lunar(i) = Params.Absorptivity_per_face(i) * assumed_area * cos_a * (q_albedo + q_IR_now);
    else
        Q_Lunar(i) = 0;
    end
end

External_Q_total = sum(Q_solar + Q_Lunar);

end