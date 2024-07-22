function Params = PowerBudget(Params)
%--------------------------------------------------------
% PURPOSE: This function defines the necessary parameters
%          for the electricity system for easy and
%          clean access throughout the simulation and forming 
%          power budget for power and charge calculations
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


Params.Max_capacity = 43; % Maximum allowed power for the batteries [Wh] 
Params.vertical_power_produc = 4*45; % maximum solar power production [w]

% everything is in [w]

% Modes = {'Sun Cruise','Night Cruise','GRB event','Communication','Detumbling','Commissioning','Decomissioning'};

Modes = {'Sun Cruise','Night Cruise','GRB event','Communication'};
n=length(Modes);

% Avionics
EPS                   =  0.6.*ones(1,n)';
batteries             =  0.1.*ones(1,n)';
Solar_Panels          =  [zeros(1,n)]'; %TBD

% Communication
Antenna_L               =  [zeros(1,n)]'; %TBD
Antenna_S               =  2*[0,0,ones(1,2)]';
Transmitter_L           =  18*ones(1,n)';
Transmitter_S           =  13*[0,0,ones(1,2)]';

% Computers & Data
OBC                   =  2*ones(1,n)';

% Control
Star_Tracker          =  3*1.5*[0,ones(1,3)]';
Sun_Sensor            =  3*0.35*[1,0,0,1]';
IMU                   =  2*1.5*[0,0,1,0]';
GPS                   =  0.05*ones(1,n)';
Magnet_Torquer        =  3*0.2*[1,1,0,1]';
Magnometer            =  3*0.05*ones(1,n)';
Reaction_Wheels       =  4*4*ones(1,n)';

% Structure & Deep Space
Heaters               =  zeros(1,n)'; %TBD

%Payload
Payload               =  22*ones(1,n)';


All_components        = [EPS, batteries,Solar_Panels , Antenna_L, Antenna_S, Transmitter_L,Transmitter_S,OBC,...
                        Star_Tracker, Sun_Sensor, IMU, GPS,Magnet_Torquer,Magnometer, ...
                        Reaction_Wheels, Heaters, Payload];

Total_Power           = sum(All_components,2);


Params.power_budget   = table(All_components, Total_Power, 'RowNames', Modes);



end