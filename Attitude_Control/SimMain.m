close all
clear all
clc

% Constants
mu = 3.986004418*10^14; % [m^3/sec^2]
R_eq = 6378.137*10^3; % [m]
J2 = 1.08264*10^(-3); % 

% Initial conditions
initial_tet = deg2rad(0);
initial_psi = deg2rad(0);
initial_phi = deg2rad(0);
Params.q0 = eul2quat([pi-initial_phi,-initial_tet,initial_psi]);
% paParams.q0 = [0,0,0,1`]; % Initial quaternion 0 0 0 
Params.w0 = [5,0,0]'; % Angular Velocity [deg/sec]
Params.w0_W = 0*[1,1,1,1]';

%Satellite inertia
I_conv = 1/10^9;
Ixx = 136536680.23*I_conv;
Ixy = -2302943.14*I_conv;
Ixz = -456852.60*I_conv;
Iyx = -2302943.14*I_conv;
Iyy = 102436630.43*I_conv;
Iyz = -1019138.10*I_conv;
Izx = -456852.60*I_conv;
Izy = -1019138.10*I_conv;
Izz = 141529348.05*I_conv;
Params.J = [Ixx,Ixy,Ixz;
            Iyx,Iyy,Iyz;
            Izx,Izy,Izz];

% wheel's inertia [kg*m^2]
m_rw = 0.400;
Dimensions_rw = [67;25;67]./1000; 
Params.Jw = m_rw/12 * (Dimensions_rw(1)^2 + Dimensions_rw(2)^2);

% Wheel configuration
% Pyramid Configuration (the chosen one)
aux = 1/sqrt(3); % Factoring by 1/sqrt(3) to get nicer looking vectors in the code
Params.W1 = aux * [1;-1;1]; % Coordinates of the corresponding wheel
Params.W2 = aux * [-1;1;1]; 
Params.W3 = aux * [-1;-1;1]; 
Params.W4 = aux * [1;1;1]; 
Params.W = [Params.W1,Params.W2,Params.W3,Params.W4]; % Wheel config matrix
Params.Winv = pinv(Params.W); % Inverse of the wheel config matrix

% toquebox contains the maximum allowed torque for given command torque and
% stored torque.
Params.torquebox = load('torquebox_modified.mat'); % Given by the manufacturer, the input is the controller output, the output is the torque working on each wheel 

% Wheel limits 
H_lim = 0.03; % Angular momentum limit [Nms] 
T_lim = 2e-3; % Torque limit [Nm]


utc_time = datetime('now', 'timezone', 'utc');
% lla = eci2lla(Launch_Position,utc);
Launch_latitude = 32.77486454809806;
Launch_longitude = 35.02132419278802;

Pos_I = [7078137;0.100000000000000;0.959931088596881];
alt = 550*10^3; %[m]
a0 = alt + R_eq; %[m]
ecc0 = 0.1; 
inc0 = deg2rad(55);
omega0 = deg2rad(5);
Omega0 = deg2rad(10);
f0 = deg2rad(0);
Theta0 = omega0 + f0;
OE0 = [a0; ecc0; inc0; omega0; Omega0; f0];
epoch_time = 0; %???
E_to_I = calc_E_to_I(epoch_time);
I_to_E = E_to_I';
E_to_L = rotz(-(pi/2 + Launch_longitude))*rotx(-(pi/2 - Launch_latitude));
L_to_E = E_to_L';
I_to_L = E_to_L*I_to_E;
Lv_to_I = rotz(Omega0)*rotx(inc0)*rotz(Theta0);
I_to_Lv = Lv_to_I';

currnet_tet = initial_tet;
currnet_psi = initial_psi;
currnet_phi = initial_phi;
L_to_B = tetpsiphi(currnet_tet,currnet_psi,currnet_phi);
I_to_B = L_to_B*I_to_L;
[uSat2Sun_B,isLOS2Sun,eul2target] = SunSensorModel(Pos_I,I_to_B,utc_time);

% Target conditions
Params.q_T = eul2quat([pi-eul2target(3),-eul2target(1),eul2target(2)]);
% Params.q_T = [0,0,0,1]'; % Target quaternion 0 0 0 

% PD Controller 
zeta = 0.7; % Damping ratio
wn = 0.1; % omega_n - Natural frequency 

% PD Controller gains:
kp = 200*wn*wn*Params.J(1,1); % Proportional controller gain
kd = 200*zeta*wn*10*Params.J(1,1); % Derivative controller gain
% Assigning a gain matrix:
Params.gains.kpx = kp;
Params.gains.kdx = kd;
Params.gains.kpy = kp;
Params.gains.kdy = kd;
Params.gains.kpz = kp;
Params.gains.kdz = kd;

%% Simulate and extract the data
Params.tf = 4000; % Simulation time [sec]
% Outsim = sim('Control_Sim'); % Simulink model
% Data = Outsim.Data.signals.values(:,:,:);
% Data = reshape(Data,[26 length(Data)]);
% q_t = Data(1:4,:); % Quaternion vector in time
% w_t = Data(5:7,:); % Angular velocity vector in time
% q_error = Data(8:11,:); % Quaternion error vector in time (not important for now)
% w_error = Data(12:14,:); % Angular momentum vector in time
% Tc = Data(15:18,:); % Torque controller
% TW = Data(19:22,:); % Torque on wheels
% HW = Data(23:26,:); % Angular momentum on wheels
% t = Outsim.tout;    % Time vector


function E_to_I = calc_E_to_I(epoch_time)
    Theta_Earth = 360.9856123035484*epoch_time + 280.46;
    E_to_I = rotz(Theta_Earth);
end


function mat = tetpsiphi(tet,psi,phi)
    mat = rotx(phi)*rotz(psi)*roty(tet);
end


function mat = rotx(angle)
    mat = [1, 0         , 0;
           0, cos(angle), -sin(angle);
           0, sin(angle), cos(angle)];
end
function mat = roty(angle)
    mat = [cos(angle) , 0, sin(angle);
           0          , 1, 0;
           -sin(angle), 0, cos(angle)];
end
function mat = rotz(angle)
    mat = [cos(angle), -sin(angle), 0;
           sin(angle), cos(angle) , 0;
           0         , 0          , 1];
end



