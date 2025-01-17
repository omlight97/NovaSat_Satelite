%function SimData = NovaSAT_main_simulation(Params,Flags)
% NovaSat MAIN SIMULATION
% Originally by: May Alon (Jericco)
% NovaSAT editors: Yarden Milshtein & Benny Muchnick
clc;
close all;
clear all;

tic
file_names = dir;
Flags2.sun_search.initial_flag = 1;
Flags2.Can_Continue = 0;
Flags2.Mission_Now = 0;
Flags2.Comms_Now = 0;
addpath(genpath(pwd));
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
try
    [DataBase.SunTimes, DataBase.SunPosition]     = Read_Data_From_STK2([pwd,'\3 days/NOVASAT-16U_MatlabReport_-_SunPosition']);
    [DataBase.SatTimes, DataBase.SatProperties]   = Read_Data_From_STK2([pwd,'\3 days/NOVASAT-16U_FullSimulation']);
    [DataBase.EarthTimes, DataBase.EarthPosition] = Read_Data_From_STK2([pwd,'\3 days/NOVASAT-16U_MatlabReport_-_EarthPosition']);
catch
    [DataBase.SunTimes, DataBase.SunPosition]     = Read_Data_From_STK2([pwd,'/3 days/NOVASAT-16U_MatlabReport_-_SunPosition']);
    [DataBase.SatTimes, DataBase.SatProperties]   = Read_Data_From_STK2([pwd,'/3 days/NOVASAT-16U_FullSimulation']);
    [DataBase.EarthTimes, DataBase.EarthPosition] = Read_Data_From_STK2([pwd,'/3 days/NOVASAT-16U_MatlabReport_-_EarthPosition']);
end

e     = DataBase.SatProperties(:,9);
i2    = DataBase.SatProperties(:,10);
f2    = DataBase.SatProperties(:,13);
r_sat=sqrt(DataBase.SatProperties.Var6.^2+DataBase.SatProperties.Var7.^2+DataBase.SatProperties.Var8.^2);
v_sat=sqrt(DataBase.SatProperties.Var9.^2+DataBase.SatProperties.Var10.^2+DataBase.SatProperties.Var11.^2);
a     = DataBase.SatProperties(:,8);
beta=DataBase.SatProperties(:,3);
Time_vec=DataBase.SatTimes;
%%
% Gets vector of Communication time for 3 Satellites(0-No Comm, 1-Comm)

AccessInmarsatVec = GetAccess(Time_vec,'Inmarsat');
AccessTechnion = GetAccess(Time_vec,'GS');
AccessSumTime = GetSumAccess(AccessInmarsatVec);
DayOrNight_vec = GetAccess(Time_vec,'Sun');
InmarsatPostion=table2array(DataBase.SatProperties(:,22:30));
PositionCommTime=Get_position_SumAccess(AccessSumTime,InmarsatPostion);

%%
% Creates 'Params'
if ~exist('Params')
    Params = [];
end

[Params, Flags2] = JeriParams(Params,Flags2);
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
% Flags.Skip = 0;

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
psi_vec=zeros(size(DataBase.EarthTimes));
Theta_vec=zeros(size(DataBase.EarthTimes));
phi_vec=zeros(size(DataBase.EarthTimes));
B_vec=zeros(3,length(phi_vec));
GRB_Alert= Get_GRB_Alert(Time_vec);
Flags.sun_search.initial_flag = 1;
%%
while  i <=  length(DataBase.SunTimes)
    if GRB_Alert(i)
        Flags.GRB=1;
    else
        Flags.GRB=0;
    end
    alpha_G_0=10;
    n=6;
    Params.SunPosition=[DataBase.SunPosition.Var5(i),DataBase.SunPosition.Var6(i),DataBase.SunPosition.Var7(i)];
    Params.CommsSatPosition = PositionCommTime(i,3:5);
    % External calculations (Alpha_angle, Date, Day or Night)
    if i==1
    SatPosition=[Current_Step_Orbit.r(1),Current_Step_Orbit.r(2),Current_Step_Orbit.r(3)];
    else
        SatPosition=table2array(Current_Step_Orbit.r);
    end
    if i==1
    SatVelocity=[Current_Step_Orbit.v(1),Current_Step_Orbit.v(2),Current_Step_Orbit.v(3)];
    else
        SatVelocity=table2array(Current_Step_Orbit.v);
    end
    Params.SatPosition=SatPosition;
    Flags.Day = Day_or_Night(Params, Params.SatPosition,Params.SunPosition);
    Flags.IsDay =DayOrNight_vec(i);
    SimData.Day_Or_Night(i) = DayOrNight_vec(i);
    [B_vec(:,i)]  = earthmagfield13(SatPosition', Time_vec(i), g, h, alpha_G_0, n);
    Params.MagneticField=B_vec;
    Params.Communication=PositionCommTime;
%     if Flags.Skip
    if PositionCommTime(i,2)>15
        Flags.Communication=1;
    else
        Flags.Communication=0;
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
    [Power, next_charge,DOD] = main_power2(Params,Current_Step_Angular, current_charge, dt, Flags);
    % if next_charge<=0
    %     display('simulation stops');
    %     break
    % end
    SimData.Power.Production(i,:)  = Power.Production;
    SimData.Batteries_Electric_Charge(i,:) = next_charge;

    % for par = Params.power_budget.Properties.VariableNames
    %     SimData.Power.(char(par))(i) =  Power.(char(par));
    % end
    SimData.Power.Total_Power(i,:) = Power.Total_Power;

    current_charge = next_charge;

    DOD_vec(i) = DOD;
    if Power.Total_Power<70
                Flags.Communication=0;
    end

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
        
        
        %% Attitude Control
        Params.orbital_elements=[DataBase.SatProperties(i,8),DataBase.SatProperties(i,9),DataBase.SatProperties(i,10),DataBase.SatProperties(i,11),DataBase.SatProperties(i,12),DataBase.SatProperties(i,13)];
        [Next_Step_Angular,Params,Flags] = Nova_Attitude_Prop(Current_Step_Angular,Params,Flags);
        SimData.Angular_State(i,:) = Next_Step_Angular;
       % SimData.Wheels_Data(i,:) = Wheels_Data;
        %SimData.More_Data_Attitude_Control(i,:) = More_Data_Attitude_Control; %Name should be given!
    %end
       psi_vec(i) = Next_Step_Angular.Psi;
       Theta_vec(i)=Next_Step_Angular.Theta;
       phi_vec(i)=Next_Step_Angular.Phi;
       Current_Step_Angular = Next_Step_Angular;
       %   psi_vec(i) = 0;
       % Theta_vec(i)=0;
       % phi_vec(i)=0;
       % % Current_Step_Angular = Next_Step_Angular;
       % Current_Step_Angular.Psi=0;
       %        Current_Step_Angular.Phi=0;
       %         Current_Step_Angular.Theta=0;


       
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
subplot(3,1,1);
% sgtitle(['Orbit For a=', num2str(a(1)), ' [km], i=', num2str(i(1)), '[deg]'])
a=table2array(a);
e=table2array(e);
i2=table2array(i2);
plot(Time_vec/60,r_sat);
xlabel('time [min]');
ylabel('r [KM]');
title('r(t)');
grid on;
%ylim([min(a)-6400 max(a)-6340])

subplot(3,1,2);
plot(Time_vec/60,e);
xlabel('time [min]');
ylabel('e'); ylim([-0.02,0.05]);
title('e(t)');
grid on;

subplot(3,1,3);
plot(Time_vec/60,i2);
xlabel('time [min]');
ylabel('i [deg]');
title('i(t)');
grid on;
figure;
subplot(3,1,1);
hold on;
plot(Time_vec/60,psi_vec);
xlabel('time [min]');
ylabel('psi [rad]');
title('psi vs time');
grid on;
subplot(3,1,2);
hold on;
plot(Time_vec/60,Theta_vec);
xlabel('time [min]');
ylabel('theta [rad]');
title('theta vs time');
grid on;
subplot(3,1,3);
hold on;
plot(Time_vec/60,phi_vec);
xlabel('time [min]');
ylabel('phi [rad]');
title('phi vs time');
grid on;
%%
figure()
subplot(3,1,1);
hold on;
plot(Time_vec/60,DayOrNight_vec);
xlabel('time [min]');
ylabel('ISDay [rad]');
title('DayOrNight vs time');
grid on;
subplot(3,1,2);
hold on;
plot(Time_vec/60,GRB_Alert);
xlabel('time [min]');
ylabel('GRB ');
title('GRB vs time');
grid on;

imarasat1=zeros(1,length(Time_vec));
imarasat2=zeros(1,length(Time_vec));
imarasat3=zeros(1,length(Time_vec));

for i=1:length(Time_vec)
    if PositionCommTime(i,1)==1 && PositionCommTime(i,2)>2
        imarasat1(i)=PositionCommTime(i,2);
    end
    if PositionCommTime(i,1)==2 && PositionCommTime(i,2)>2
        imarasat2(i)=PositionCommTime(i,2);
    end
    if PositionCommTime(i,1)==3 && PositionCommTime(i,2)>2
        imarasat3(i)=PositionCommTime(i,2);
    end
end

subplot(3,1,3);
hold all;
bar(Time_vec/60,imarasat1);
bar(Time_vec/60,imarasat2);
bar(Time_vec/60,imarasat3);
xlabel('time [min]');
ylabel('communicationtime[min] ');
title('communication vs time');
grid on;
legend('inmarsat1','inmarsat2','inmarsat3');
ylim([1 40]);
%%
figure;
subplot(4,1,1);
hold on;
plot(Time_vec/60,SimData.Power.Total_Power');
xlabel('time [min]');
ylabel('Total Power [w]');
title('Total Power vs time');
grid on;
% ylim([0 200]);
subplot(4,1,2);
hold on;
plot(Time_vec/60,SimData.Power.Production');
xlabel('time [min]');
ylabel('Production [w]');
title('Production vs time');
grid on;
subplot(4,1,3);
hold on;
plot(Time_vec/60,SimData.Batteries_Electric_Charge');
xlabel('time [min]');
ylabel('Batteries ElectricCharge [W]');
title('Batteries ElectricCharge vs time');
grid on;
subplot(4,1,4)
hold on;
plot(Time_vec/60,DOD_vec);
xlabel('time [min]');
ylabel('%');
title('DOD vs time');
grid on;
%%
figure()
axis equal;
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;

% Initialize quiver objects for the principal axes
hX = quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2);
hY = quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2);
hZ = quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2);

% Loop to update the plot
for k = 1:length(Theta_vec)
    % Compute the rotation matrix from Euler angles
   R = eul2rotm([psi_vec(k) Theta_vec(k) phi_vec(k)], 'ZYX');
    
    % Principal axes in the rotated frame
    xAxis = R(:, 1);
    yAxis = R(:, 2);
    zAxis = R(:, 3);
    % Update the quiver objects
    set(hX, 'UData', xAxis(1), 'VData', xAxis(2), 'WData', xAxis(3));
    set(hY, 'UData', yAxis(1), 'VData', yAxis(2), 'WData', yAxis(3));
    set(hZ, 'UData', zAxis(1), 'VData', zAxis(2), 'WData', zAxis(3));
    
    drawnow;

end
