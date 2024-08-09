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
% The commponent list is updated for spring semester 2024
%--------------------------------------------------------



% everything is in [w]

Modes = {'Sun Cruise','Night Cruise','GRB event','Communication'};

% Avionics
ACU                   =  [zeros(1,4)]'; %TBD
PDU                   =  [zeros(1,4)]'; %TBD
batteries             =  6.*ones(1,4)';
Solar_Panels          =  [zeros(1,4)]'; %TBD

% Communication
Antenna_S                     =  2*[0,0,ones(1,2)]';
Transmitter_and_Antenna_L     =  [0 0 0 18/2]';
Transmitter_S                 =  [1.5,1.5,1.5,13/2]';

% Computers & Data
OBC                   =  2*ones(1,4)';

% Control
Star_Tracker          =  1*0.165*[0,ones(1,3)]';
Sun_Sensor            =  1*0.1*[1,0,0,1]';
IMU                   =  2*1.5*[0,0,1,0]';
GPS                   =  0.05*ones(1,4)';
Magnet_Torquer        =  3*0.68*[1,1,0,1]';
Magnometer            =  3*0.05*ones(1,4)';
Reaction_Wheels       =  4*2*ones(1,4)';

% Structure & Deep Space
Heaters               =  zeros(1,4)'; %TBD

%Payload
Payload               =  18*ones(1,4)';


All_components        = [ACU, PDU, batteries,Solar_Panels, Antenna_S, Transmitter_and_Antenna_L,Transmitter_S,OBC,...
                        Star_Tracker, Sun_Sensor, IMU, GPS,Magnet_Torquer,Magnometer, ...
                        Reaction_Wheels, Heaters, Payload];

Total_Power           = sum(All_components,2);


Params.power_budget   = table(ACU, PDU, batteries,Solar_Panels, Antenna_S, Transmitter_and_Antenna_L,Transmitter_S,OBC,...
                        Star_Tracker, Sun_Sensor, IMU, GPS,Magnet_Torquer,Magnometer, ...
                        Reaction_Wheels, Heaters, Payload, Total_Power, 'RowNames', Modes);



end