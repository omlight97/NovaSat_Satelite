function Params = PowerBudget(Params)
%--------------------------------------------------------
% PURPOSE: This function defines the necessary parameters
%          for the electricity system for easy and
%          clean access throughout the simulation
%--------------------------------------------------------
% INPUT: an empty list by the name params (given there is non)
%--------------------------------------------------------
% OUTPUT: The list params full with all the necessary parameters 
%         for the simulation
%--------------------------------------------------------

%-------------! keep in mind !---------------------------
% Modes should be reviewed to make sure that it's corrert
% The commponent list is not updated!!!!
%--------------------------------------------------------
%% General Properties
% Coordinates Convertion
clear pi
Params.sec_to_hour = 1/3600; %second to hour convertion

% General Properties of the Moon
Params.mu = (4.9048695)*10^3; % Standard Gravitational parameter [km^3/s^2]
Params.R  = 1737.4;           % Mean Radius [km]

%% Electricity

Params.Max_capacity = 154; % Maximum allowed power for the batteries [Wh] 
Params.vertical_power_produc = 133.2847; % maximum solar power production [w]

% everything is in [w]

Modes = {'Sun Cruise','Night Cruise','GRB event','Communication','Detumbling','Commissioning','Decomissioning'};

EPS                   =  [1.94*ones(1,3), 0.574, 0.574, 1.94, 0.574]';
TX_x_band             =  [5.1, zeros(1,6)]';
RX_S_band             =  [2.2*ones(1,3), zeros(1,2), 2.2*ones(1,2)]';
Antenna               =  [26.6, zeros(1,6)]';
Transmitter           =  [zeros(1,7)]';
batteries             =  [zeros(1,7)]';
Solar_Panels          =  [zeros(1,7)]';
OBC                   =  0.7*ones(1,7)';
Data_storage          =  [zeros(1,7)]';
Star_Tracker          =  [2.2, 1.4,	2.2, 2.2, 2.2, 0.74, 1.6]';
Sun_Sensor            =  [0.1518*ones(1,4),	0.1386,	0.1518,	0.1386]';
IMU                   =  [4, 3, 4, 4, 3, 4, 3]';
Magnet_Torquer        =  [zeros(1,7)]';
Magnometer            =  [zeros(1,7)]';
Reaction_Wheels       =  [7.2, 1.2, 7.2, 1.2, 1.2, 0.4, 1.2]';
Heaters               =  zeros(1,7)';
Payload               =  [0, 0, 3.5, zeros(1,4)]';

All_components        = [EPS, TX_x_band, RX_S_band, Antenna, Transmitter,batteries,Solar_Panels,OBC,...
                        Data_storage,Star_Tracker, Sun_Sensor, IMU, Magnet_Torquer,Magnometer, ...
                        Reaction_Wheels, Heaters, Payload];

Total_Power           = sum(All_components,2);


Params.power_budget   = table(EPS, TX_x_band, RX_S_band, Antenna, Transmitter,batteries,Solar_Panels,OBC,...
                        Data_storage,Star_Tracker, Sun_Sensor, IMU, Magnet_Torquer,Magnometer, ...
                        Reaction_Wheels, Heaters, Payload, Total_Power, 'RowNames', Modes);



end