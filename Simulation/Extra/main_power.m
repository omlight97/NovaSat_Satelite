%% Simulation Main File - Avionics Team 
% Nehama Kaplan & Or Yehezkel

%% Initialization

close;clc;clear;

% unless recieving 'Params' from simulation team, Params is
% set to be an empty array
if ~exist('Params')
    Params = [];
end

% Electricity parameters for the power bugdet 
[Params] = PowerBudget(Params);

%% 

% given world model (sun position,earth position ets.) and angular state
% the calc_theta function will be implemented

theta_angle = Calc_theta(Parms.SunPosition, Current_Step_Angular ,Param.SatPosition);

%% Jerico Part

% Flags.stop_simulation = 0;
% 
% states.Logic = 'operational';

% Flags.Day = Day_or_Night(Params, Current_Step_Orbit.r, World_Model.SunPosition);

% Flags.Day = 1; % it's day time
% 
% if strcmp(states.Logic,'operational')
%     if Flags.Day
%         states.Control = 'Communication';
%     else
%         states.Control = 'GRB event';
%     end
% elseif strcmp(states.Logic,'SafeMode')
%     if Flags.Day
%         states.Control = 'Sun Cruise';
%     else
%         states.Control = 'Night Cruise';
%     end
% end

%% Arbitrarily set in order to run the power simulation separately

% flag parameters
Flags.IsDay = 1;
Flags.Comm = 0;

% theta angle parameter
theta_angle = 0.6435; 
theta_angle_night = 0;

% simulation parameters
dt = 1; %[sec]
t = 0:dt:24*60*60; %[sec]

%% Main Power Simulation

% initialization
current_charge = zeros(1,length(t));    %[Wh]
Total_Power = zeros(1,length(t));       %[W]
Power_production = zeros(1,length(t));  %[W]
DOD = zeros(1,length(t));               %[Wh]

current_charge(1) = 77; %[Wh]
max_charge = Params.Max_capacity; %[Wh]

% simulation loop
for i=1:length(t)-1

[Power, next_charge] = Nova_Calc_Power(Params, theta_angle, current_charge(i), dt, Flags);
current_charge(i+1) = next_charge;

DOD(i) = 1 - (max_charge - current_charge(i))/max_charge;
Total_Power(i) = Power.Total_Power;
Power_production(i) = Power.Production;

% stop\warning condition for the overall simulation 
% could be implemented elsewere - TBD by simulation team
if current_charge(i) == 0
   break
end

end

%% ploting

figure(1)
plot(t./60^2,current_charge)
xlabel("time [hr]")
ylabel("Charge [wh]")

figure(2)
plot(t./60^2,Total_Power)
xlabel("time [hr]")
ylabel("Power [w]")
title("Total Power")

figure(3)
plot(t./60^2,Power_production)
xlabel("time [hr]")
ylabel("Power [w]")
title("Power Production")




