close all
clear 
clc

day_or_night = load('day_or_night.mat');
earth_position = load('earth_position.mat');
magnetic_field = load('magnetic field.mat');
sat_position = load('sat_position.mat');
States = load('States.mat');
Params = load('Params.mat');
Flags.day_or_night = day_or_night.SimData.Day_Or_Night;
Current_Step_Angular = day_or_night.SimData.Angular_State';

t_len = size(Current_Step_Angular,2);
initial_psi = 0;
initial_tet = 0;
initial_phi = 0;
initial_p = 0;
initial_q = 0;
initial_r = 0;

Current_Step_Angular = zeros(6,t_len);
States = zeros(1,t_len);

Current_Step_Angular(:,1) = [initial_psi;initial_tet;initial_phi;initial_p;initial_q;initial_r];
for idx=1:t_len-1
    [Next_Angular_State,Attitude_Control_Data,Flags] = Nova_Attitude_Prop(States(idx),Current_Step_Angular(:,idx),Flags,Params);
    Current_Step_Angular(1:3,idx+1) = Next_Angular_State.angles;
    Current_Step_Angular(4:6,idx+1) = Next_Angular_State.rates;
end


