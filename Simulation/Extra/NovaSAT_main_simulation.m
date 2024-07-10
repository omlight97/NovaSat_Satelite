%function SimData = NovaSAT_main_simulation(Params,Flags)
% NovaSat MAIN SIMULATION
% Originally by: May Alon (Jericco)
% NovaSAT editors: Yarden Milshtein & Benny Muchnick
clc;
close all;
clear all;
tic
file_names = dir;
Flags.Can_Continue = 0;
Flags.Mission_Now = 0;
Flags.Comms_Now = 0;
[g, h] = IGRF13;

% Checks if the GUI file is in the current directory
% for k = 1:length(file_names)
%     if strcmp(file_names(k).name,'NOVASAT_MAIN.mlapp')
%         Flags.Can_Continue = 1;
%         break
%     end
% end
% clear file_names k
% 
% %Adding all folders and sub-folders to path
% if Flags.Can_Continue
%     addpath(genpath(pwd))
% else
%     error('Change directory! (NOVASAT_MAIN.mlapp file is missing)')
% end

% [DataBase2.SunTimes, DataBase2.SunPosition]     = Read_Data_From_STK([pwd,'\NOVASAT-16U_MatlabReport_-_SunPosition']);
% [DataBase.EarthTimes, DataBase.EarthPosition] = Read_Data_From_STK([pwd,'\NOVASAT-16U_MatlabReport_-_EarthPosition']);

[DataBase.SunTimes, DataBase.SunPosition]     = Read_Data_From_STK2([pwd,'\NOVASAT-16U_MatlabReport_-_SunPosition - Copy']);
[DataBase.SatTimes, DataBase.SatProperties]   = Read_Data_From_STK2([pwd,'\NOVASAT-16U_FullSimulation']);
[DataBase.EarthTimes, DataBase.EarthPosition] = Read_Data_From_STK2([pwd,'\NOVASAT-16U_MatlabReport_-_EarthPosition - Copy']);
e     = DataBase.SatProperties(:,9);
i2    = DataBase.SatProperties(:,10);
a     = DataBase.SatProperties(:,8);
beta=DataBase.SatProperties(:,3);
Time_vec=DataBase.SatTimes;
%%
% Gets vector of Communication time for 3 Satellites(0-No Comm, 1-Comm)
AccessInmarsatVec = GetAccess(Time_vec,'Inmarsat');
AccessSumTime = GetSumAccess(AccessInmarsatVec);
DayOrNight_vec = GetAccess(Time_vec,'Sun');
InmarsatPostion=table2array(DataBase.SatProperties(:,22:30));
PositionCommTime=Get_position_SumAccess(AccessSumTime,InmarsatPostion);
%%
% Creates 'Params'
if ~exist('Params')
    Params = [];
end

[Params, Flags] = JeriParams(Params,Flags);
[Numeric_properties] = Define_Numeric_properties(DataBase.SunTimes);

if isfield(Params,'Duration')
    Duration = Params.Duration;
else
    Duration = 1*3600; % [s]
    disp(['Default Scenario: ', num2str(Params.Scenario_Duration_days), ' Hours,'])
end

% Finding index of start and end time in exported data from STK
ind_initial = 1;
ind_final   = find(DataBase.SunTimes > Duration,1);
World_Model_All.SunTimes      = DataBase.SunTimes(ind_initial:ind_final);
World_Model_All.SunPosition   = DataBase.SunPosition(ind_initial:ind_final,:);

ind_initial = 1;
ind_final   = find(DataBase.EarthTimes > Duration,1);
World_Model_All.EarthTimes    = DataBase.EarthTimes(ind_initial:ind_final);
World_Model_All.EarthPosition = DataBase.EarthPosition(ind_initial:ind_final,:);

OneDay = seconds(hours(24)); %[s]
%Current_Date = initial_date;
World_Model.SunPosition0   = World_Model_All.SunPosition(1,:);
World_Model.EarthPosition0 = World_Model_All.EarthPosition(1,:);

%% Initial Orbital Elements

if ~isfield(Params,'OrbitalElements0')
    Current_Step_Orbit.r = Params.r0;
    Current_Step_Orbit.v = Params.v0;
    
elseif isnan(Params.OrbitalElements0)
    Current_Step_Orbit.r = Params.r0_and_v0(1:3);
    Current_Step_Orbit.v = Params.r0_and_v0(4:6);
    
elseif isnan(Params.r0_and_v0)
    elements.ecc = Params.OrbitalElements0(4);
    elements.omega = deg2rad(Params.OrbitalElements0(2));
    elements.inc = deg2rad(Params.OrbitalElements0(6));
    elements.RAAN = deg2rad(Params.OrbitalElements0(5));
    elements.a = Params.OrbitalElements0(1);
    theta = deg2rad(Params.OrbitalElements0(3));
    
    r_and_v = OrbitalElements2StateVec(elements, theta, Params);
    Current_Step_Orbit.r = r_and_v.r;
    Current_Step_Orbit.v = r_and_v.v;
end

%%
% if Flags.Start_at_RAAN
%     Current_Step_Orbit = Prop_to_RAAN(Current_Step_Orbit, Numeric_properties, Params);
% end
% 
t0 = 0;
Flags.Skip = 0;

if ~isfield(Params,'Attitude_Angles')
    Current_Step_Angular.Psi = Params.Attitude_Angles0.Psi;
    Current_Step_Angular.Theta = Params.Attitude_Angles0.Theta;
    Current_Step_Angular.Phi = Params.Attitude_Angles0.Phi;
    Current_Step_Angular.P = Params.Attitude_rate0.P;
    Current_Step_Angular.Q = Params.Attitude_rate0.Q;
    Current_Step_Angular.R = Params.Attitude_rate0.R;

    % Current_Step_Angular  =  Params.Attitude_rate0;
else
        Current_Step_Angular.Psi = Params.Attitude_Angles.Psi;
    Current_Step_Angular.Theta = Params.Attitude_Angles.Theta;
    Current_Step_Angular.Phi = Params.Attitude_Angles0.Phi;
    Current_Step_Angular.P = Params.Attitude_Angles0.P;
    Current_Step_Angular.Q = Params.Attitude_Angles0.Q;
    Current_Step_Angular.R = Params.Attitude_Angles0.R;
    % Current_Step_Angular = deg2rad(Params.Attitude_Angles);
    % Current_Step_Angular = deg2rad(Params.Attitude_rate);
end

%dt = Numeric_properties.dt;
dt=Time_vec(2)-Time_vec(1);
i = 1;

Alpha_angle = deg2rad(20); 
states.Logic = 'operational';
current_charge = 20; % [Wh]
Thermal.Average_Temperature = Params.Initial_Temperature;


T_final = Time_vec(end); % [s]
psi_vec=zeros(1,1241);
Theta_vec=zeros(1,1241);
phi_vec=zeros(1,1241);
Bx=zeros(3,1241);
By=zeros(1,1241);
Bz=zeros(1,1241);
%%
while  i <  length(DataBase.SunTimes)
    
    alpha_G_0=10;
    n=6;
    Params.SunPosition=[DataBase.SunPosition.Var5(i),DataBase.SunPosition.Var6(i),DataBase.SunPosition.Var7(i)];
    % External calculations (Alpha_angle, Date, Day or Night)
    if i==1
    SatPosition=[Current_Step_Orbit.r(1),Current_Step_Orbit.r(2),Current_Step_Orbit.r(3)];
    else
        SatPosition=table2array(Current_Step_Orbit.r);
    end
    Params.SatPosition=SatPosition;
    Flags.Day = Day_or_Night(Params, Params.SatPosition,Params.SunPosition);
    Flags.IsDay =DayOrNight_vec(i);
    SimData.Day_Or_Night(i) = DayOrNight_vec(i);
    [Bx(:,i)]  = earthmagfield13(SatPosition', Time_vec(i), g, h, alpha_G_0, n);
    Params.MagneticField=Bx;
    Params.Communication=PositionCommTime;
%     if Flags.Skip
    if PositionCommTime(i,2)>15
        Flags.Communication=1;
    else
        Flags.Communication=1;
    end
    if strcmp(states.Logic,'operational')
        if Flags.Day
            states.day = 1;
            states.Control='sun search';
        else
            states.day = 0;
            states.Control='min drag';
        end
    % elseif strcmp(states.Logic,'SafeMode')
    %     if Flags.Day
    %         states.Control = 'cruise_safe_day';
    %     else
    %         states.Control = 'cruise_safe_night';
    %     end
     end
    
%     if (~mod(time_vec(i),OneDay)) && time_vec(i)
%         Current_Date = Current_Date + duration(hours(24));
%         Ind_Today = find(strcmp(string(World_Model_All.SunTimes), string(Current_Date)));
%         World_Model.SunPosition0   = World_Model_All.SunPosition(Ind_Today,:);
%         World_Model.EarthPosition0 = World_Model_All.EarthPosition(Ind_Today,:);
%     end
    
    %     Alpha_angle = Calc_Alpha(World_Model, Current_Step_Angular, Current_Step_Orbit);
    SimData.Alpha_angle(i,:) = Alpha_angle;
   % SimData.World_Model(i,:) = World_Model_All;
    
    %% Power
%     % if i==500
%     %     Alpha_angle=pi/2;
%     % end
%     [Power, next_charge] = Jeri_Calc_Power(Params, Alpha_angle, states, current_charge, dt, Flags);
%     if next_charge<=0
%         display('simulation stops');
%         break
%     end
%     SimData.Power.Production(i,:)  = Power.Production;
%     SimData.Batteries_Electric_Charge(i,:) = next_charge;
% 
%     for par = Params.power_budget.Properties.VariableNames
%         SimData.Power.(char(par))(i) =  Power.(char(par));
%     end
%     SimData.Power.Total_Power(i,:) = -Power.Total_Power;
% 
%     current_charge = next_charge;
% 
% %     %% Thermal
% %     if ~mod(time_vec(i),Numeric_properties.dt_for_thermal)
% % %         Thermal = Jeri_Calc_Thermal(World_Model, Current_Step_Angular, Current_Step_Orbit, Power, Flags, dt, Params, Thermal);
% %           Thermal = Jeri_Calc_Thermal(Params, Current_Step_Angular, Flags, dt);
% % 
% %     end
% %     SimData.Thermal.Average_Temperature(i,:) = Thermal.Average_Temperature;
% %     SimData.Thermal.q_net(i,:) = Thermal.q_net;
% %     
% %     for par = fieldnames(Thermal.Internal_Q_component)'
% %         SimData.Thermal.Internal_Q_component.(char(par))(i) =  Thermal.Internal_Q_component.(char(par));
% %     end
% %     
% %     %     SimData.Thermal.Internal_Q_component(i,:) = Thermal.Internal_Q_component;
% %     
    %% System
    
    %if Flags.Skip
        
        % Control:
        % Communication         % Max Sun Exposure   % Payload Mission   % Orbit Correction
        % Attitude Correction   % Cruise Safe        % Momentum Unload   % Eco
        
        % Logic  (Ready, Safe Mode)
        % 
        % psi   = Current_Step_Angular.angles(1);
        % theta = Current_Step_Angular.angles(2);
        % phi   = Current_Step_Angular.angles(3);
        % p     = Current_Step_Angular.rate(1);
        % q     = Current_Step_Angular.rate(2);
        % r     = Current_Step_Angular.rate(3);
        
        % if Thermal_params.aveT >= Params.T_max ||...
        %         abs(p) >=  Params.p_max ||...
        %         abs(q) >=  Params.q_max ||...
        %         abs(r) >=  Params.r_max ||...
        %         max(Power_All_Comp) >= Params.Max_Power % Add more to this list!
        %     states.Logic = 'Safe Mode';
        % else % Operational
        %     states.Logic = 'Operational';
        % end
        % 
        % SimData.Extream_Events(i,:) = Extream_Events;
        % SimData.States(i,:) = States;
        % 
        
        %% Comms
%         if isfield(Flags, User_Comms) && Flags.User_Comms && Flags.Comms_Now
%             [States, Data_Storage] = Jeri_Calc_Comms(World_Model, Current_Step_Angular ,Current_Step_Orbit, Flags);
%             SimData.Data_Storage(i,:) = Data_Storage;
%         end
        
        %% Mission
%         if isfield(Flags, User_Mission) && Flags.User_Mission && Flags.Mission_Now
%             [States, Data_Storage] = Jeri_Mission(World_Model, Current_Step_Angular, Current_Step_Orbit, Flags);
%             SimData.Data_Storage(i,:) = Data_Storage;
%         end
        
        
        %% Attitude
        [Next_Angular_State,Attitude_Control_Data,Flags2] = Nova_Attitude_Prop(Current_Step_Angular,Flags, Params);
        SimData.Angular_State(i,:) = Next_Angular_State;
       % SimData.Wheels_Data(i,:) = Wheels_Data;
        %SimData.More_Data_Attitude_Control(i,:) = More_Data_Attitude_Control; %Name should be given!
    %end
       psi_vec(i) = Next_Angular_State.Psi;
       Theta_vec(i)=Next_Angular_State.Theta;
       phi_vec(i)=Next_Angular_State.Phi;
       Current_Step_Angular(i) = Next_Angular_State;
       % 
       % Current_Step_Angular.Psi=Next_Angular_State.Psi;
       % Current_Step_Angular.angles(2)=i.Theta;
       % Current_Step_Angular.angles(3)=Next_Angular_State.Phi;
       % Current_Step_Angular.rate(1)=Next_Angular_State.P;
       % Current_Step_Angular.rate(2)=Next_Angular_State.Q;
       % Current_Step_Angular.rate(3)=Next_Angular_State.R;

    %% Orbit

    % [Next_step_Orbit,t_orbit] = Orbit_Prop(Current_Step_Orbit, Numeric_properties, Params, time_vec(i));
    % len = length(t_orbit);
    % SimData.t_orbit(j:(len+j-1),:) =  t_orbit;
    % SimData.orbit.r(j:(len+j-1),:) = Next_step_Orbit.r;
    % SimData.orbit.v(j:(len+j-1),:) = Next_step_Orbit.v;
    % len_t_so_far = length(SimData.t_orbit);
    % j = len_t_so_far+1;
    Current_Step_Orbit.r = [DataBase.SatProperties(i,1),DataBase.SatProperties(i,2),DataBase.SatProperties(i,3)];
    Current_Step_Orbit.v = [DataBase.SatProperties(i,4),DataBase.SatProperties(i,5),DataBase.SatProperties(i,6)];
    SimData.orbit.r(i,:) = table2array(Current_Step_Orbit.r);
    SimData.orbit.v(i,:) = table2array(Current_Step_Orbit.v);
%     end

    %% Time
    %time_vec(i+1) = time_vec(i) + dt;
    i = i+1;
end

% r = Norm_Each_Row(SimData.orbit.r);
% if ~Flags.Circular_Orbit
% 
%     for i = 1:length(r)
%         Next_step_Orbit.r = SimData.orbit.r(i,:);
%         Next_step_Orbit.v = SimData.orbit.v(i,:);
% 
        % Next_elements = StateVec2OrbitalElements(Next_step_Orbit,Params);
%         SimData.orbit.omega(i,:) = Next_elements.omega;
%         SimData.orbit.TrueAnomaly(i,:) = Next_elements.theta;
%         SimData.orbit.SMA(i,:)  = Next_elements.a;
%         SimData.orbit.RAAN(i,:) = Next_elements.RAAN;
%         SimData.orbit.ecc(i,:)  = Next_elements.ecc;
%         SimData.orbit.inc(i,:)  = Next_elements.inc;
%     end
% end

% SimData.orbit.ecc = (SimData.orbit.ecc);
% warning('temp!!')
SimData.Time_vec= Time_vec;
% SimData.orbit.Peri_Altitude = SimData.orbit.SMA.*(1-SimData.orbit.ecc) - Params.R;
% SimData.orbit.Apo_Altitude  = SimData.orbit.SMA.*(1+SimData.orbit.ecc) - Params.R;
% % SimData.orbit.Altitude      = r - Params.R; %r = Norm_Each_Row(SimData.orbit.r);
% SimData.orbit.Altitude = (1/2)*(SimData.orbit.Peri_Altitude + SimData.orbit.Apo_Altitude);
% toc
disp('Simulation has completed')

%end
%% plot

figure()
subplot(2,3,1);
%sgtitle(['Orbit For a=', num2str(a(1)), ' [km], i=', num2str(i(1)), '[deg]'])
a=table2array(a);
e=table2array(e);
i2=table2array(i2);
plot(Time_vec/60,(a)-6378);
xlabel('time [min]');
ylabel('h [KM]');
title('h(t)');
grid on;
ylim([min(a)-6400 max(a)-6340])

subplot(2,3,2);
plot(Time_vec/60,e);
xlabel('time [min]');
ylabel('e'); ylim([-0.02,0.05]);
title('e(t)');
grid on;

subplot(2,3,3);
plot(Time_vec/60,i2);
xlabel('time [min]');
ylabel('i [deg]');
title('i(t)');
grid on;
subplot(2,3,4);
hold on;
plot(Time_vec(1:1241)/60,psi_vec);
xlabel('time [min]');
ylabel('psi [deg]');
title('psi vs time');
grid on;
subplot(2,3,5);
hold on;
plot(Time_vec(1:1241)/60,Theta_vec);
xlabel('time [min]');
ylabel('theta [deg]');
title('theta vs time');
grid on;
subplot(2,3,6);
hold on;
plot(Time_vec(1:1241)/60,phi_vec);
xlabel('time [min]');
ylabel('phi [deg]');
title('phi vs time');
grid on;