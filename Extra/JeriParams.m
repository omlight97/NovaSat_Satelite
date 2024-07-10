function [Params, Flags] = JeriParams(Params, Flags)

%% General Properties
% Coordinates Conversion

% clear pi
% d2r = pi/180;
Params.sec_to_hour = 1/3600; %second to hour convertion

% General Properties of earth
Params.mu = 3.986004e5; % Standard Gravitational parameter [km^3/s^2]
Params.R  = 6371;       % Mean Radius [km]

Params.Duration = 1*3600; % [s]

%% Orbit
Params.min_ecc = 1e-6; % below this value we assume circular orbit

Params.Altitude0      = 590; % [km]
Params.Peri_Altitude0 = 590; % [km]
Params.Apo_Altitude0  = 590; % [km]

Params.SMA0 = 0.5*(Params.Peri_Altitude0 + Params.Apo_Altitude0 + 2*Params.R); % [km]
Params.theta0 = deg2rad(0);  % True Anomaly [rad]
Params.omega0 = deg2rad(0);  % Argument of Perilune [rad]
Params.inc0   = deg2rad(45); % Inclination [rad]
Params.RAAN0  = deg2rad(0);  % Right Ascension of the Ascending Node [rad]
Params.ecc0   = 0;           % eccentricity

Params.Mission_Start_Date = datetime('01-Jan-2026');
Params.Mission_End_Date   = datetime('01-Jan-2029');
Params.Scenario_Duration_days  = 1*3600; % deafult duration of scenario [s]

elements0.a     = Params.SMA0;
elements0.theta = Params.theta0;
elements0.omega = Params.omega0;
elements0.inc   = Params.inc0;
elements0.RAAN  = Params.RAAN0;
elements0.ecc   = Params.ecc0;

r_and_v = OrbitalElements2StateVec(elements0,elements0.theta, Params);
%[r,v] = OrbitalElements2StateVec(Params,elements0);
%r_and_v = [r,v];
Params.r0 = r_and_v.r; 
Params.v0 = r_and_v.v; 

Params.n = sqrt(Params.mu/Params.SMA0^3); % [rad/s]
Params.T = 2*pi/Params.n;

mission_duration = hours(Params.Mission_End_Date - Params.Mission_Start_Date); % hours
T_hours = Params.T/3600;

% Number of orbits during missions time
periods = mission_duration/T_hours;

%% Flags

if Params.ecc0 <= Params.min_ecc % circular orbit
    Flags.Circular_Orbit = 1;
else
    Flags.Circular_Orbit = 0;
    warning('Orbit Is not Circular at t = 0')
end

Flags.Start_at_RAAN = 1; % default

%% System



%% Electricity

Params.Max_capacity = 154; % Maximum allowed power [Wh] 
Params.vertical_power_produc = 133.2847; % maximum solar power production [w]

% everything is in [w]
Modes = {'Communication', 'Max_Sun', 'Payload_Mission', 'Orbit_Correction'...
        , 'Attitude_Correction', 'cruise_safe_day', 'cruise_safe_night', 'Momentum_Unload', 'Eco'}';

EPS                   =  [1.94*ones(1,5), 0.574, 0.574, 1.94, 0.574]';
TX_x_band             =  [5.1, zeros(1,8)]';
RX_S_band             =  [2.2*ones(1,5), zeros(1,2), 2.2*ones(1,2)]';
Antenna               =  [26.6, zeros(1,8)]';
OBC                   =  0.7*ones(1,9)';
Star_Tracker          =  [2.2, 1.4,	2.2, 2.2, 2.2, 0.74, 0.74, 0.74, 1.6]';
Sun_Sensor            =  [0.1518*ones(1,6),	0.1386,	0.1518,	0.1386]';
IMU                   =  [4, 3, 4, 4, 4, 3, 3, 4, 3]';
Prop_System           =  [zeros(1,3), 3,zeros(1,3), 1, 0]';
Reaction_Wheels       =  [7.2, 1.2, 7.2, 7.2, 7.2, 1.2, 1.2, 0.4, 1.2]';
Heaters               =  zeros(1,9)';
Electro_Optic_Payload =  [0, 0, 3.5, zeros(1,6)]';

All_components        = [EPS, TX_x_band, RX_S_band, Antenna, OBC,...
                        Star_Tracker, Sun_Sensor, IMU, Prop_System, ...
                        Reaction_Wheels, Heaters, Electro_Optic_Payload];

Total_Power           = sum(All_components,2);

Params.power_budget   = table(EPS, TX_x_band, RX_S_band, Antenna, OBC,...
                              Star_Tracker, Sun_Sensor, IMU, Prop_System, ...
                              Reaction_Wheels, Heaters, Electro_Optic_Payload, Total_Power, 'RowNames', Modes);

%% Communication



%% Thermal analysis

Params.Temperature_max       = Cel2Kel(50);                   % Highest temperature allowed [K]
Params.Temperature_min       = Cel2Kel(-10);                  % Highest temperature allowed [K]
Params.sigma                 = 5.670374*1e-8;                 % Boltzman constant [Watt/m^2*K^4]
Params.radiator_emissivity   = 0.82;                          % Epsilon (1 for perfect radiator)
Params.A_rad                 = 0.06;                          % total area of radiator [m^2]
Params.q_solar_flux          = 1346.13;                       % heat flux emitted from sun [Watt/m^2]
Params.Lunar_Albedo          = 0.12;                          % Average fractional reflectivity of the Moon [0 for black body]
Params.Absorptivity_radiator = linspace(0.08, 0.1, periods);  % Absorptivity, assume linear degredation
Params.Initial_Temperature   = Cel2Kel(30);                   % [K]
Params.Temperature_day       = Cel2Kel(123);                  % [K]
Params.Temperature_night     = Cel2Kel(-253);                 % [K]
Params.Efficiency            = 0.5;  
Params.q_albedo_day   = Params.q_solar_flux * Params.Lunar_Albedo;  % fractional solar flux reflected by the Moon's surface [Watt/m^2]
Params.q_albedo_night = 0;
Params.Normals = [ [1;0;0] [0;0;1] [0;1;0] [0;0;-1] [0;-1;0] [-1;0;0] ]; %Body Frame

Params.Absorptivity_per_face   = [0.89 , 0.44 , 0.44 , 0.44 , 0.44 , 0.26];
Params.emissivity_per_face     = [0.85 , 0.08 , 0.45 , 0.05 , 0.45 , 0.45];
Params.Faces_of_sat            = []; %[m^2]
Params.assumed_area            = 0.5*(0.06*4+0.04*2); % [m^2]
Params.Faces_Area              = [0.06, 0.04, 0.06, 0.06, 0.04, 0.06]; %No.2 and No.5 are the small faces

% Params.q_net_radiator = Params.A_rad * Params.radiator_emissivity * Params.sigma * Params.Temperature_radiator^4;

% Params.emissivity_per_face.*Params.Faces_Area
%% Structure
Params.mass  = 15.3;     % mass [kg]
Params.CP_AL = 921.096;  % Aluminum heat capacity [J/kg*K]

%% Attitude Control
Params.Attitude_Angles0.Psi   = 5; % Initial attitude [deg]
Params.Attitude_Angles0.Theta = 10; % Initial attitude [deg]
Params.Attitude_Angles0.Phi   = 12; % Initial attitude [deg]

Params.Attitude_rate0.P   = 0.1;   % Initial angular rate [deg/s]
Params.Attitude_rate0.Q   = 0;   % Initial angular rate [deg/s]
Params.Attitude_rate0.R   = 0;   % Initial angular rate [deg/s]

Params.p_max = 0.05;          % Maximum roll rate  [rad\s]
Params.q_max = Params.p_max;  % Maximum pitch rate [rad\s]
Params.r_max = Params.p_max;  % Maximum yaw rate   [rad\s]

% Controller Gains
Params.gains.kpx = 6.102e-02; 
Params.gains.kdx = 1.220e-01;
Params.gains.kpy = 2.75e-02;
Params.gains.kdy = 4.151e-02;
Params.gains.kpz = 5.390e-02;
Params.gains.kdz = 1.078e-01;

% Inertia matrix [kg*m^2]
Params.J = 1e-9 * [[244079928.67,539549.34,5734372.12];...
                   [539549.34,83010901.99,317734.95];...
                   [5734372.12,317734.95,215583426.01]];

end

function temperature_in_Kelvin = Cel2Kel(temperature_in_Celsius)
temperature_in_Kelvin = temperature_in_Celsius + 273.15;
end