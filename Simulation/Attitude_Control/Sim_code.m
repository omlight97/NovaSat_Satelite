% NovaSat Attitude SIMULATION
% NovaSAT editors: Shai Peled & Edos Osazuwa
% This function calculates the angular state of the satellite each step including angles, rates, 
% performances of reaction wheels and errors.

clc; 
clear variables;
close all;
set(0,'DefaultAxesFontName','Times','DefaultAxesFontSize',12)
set(0,'defaultLineLineWidth',1.5,'defaultLineMarkerSize',8)
%% Pararmeters
% Inertia matrix [kg*m^2]
% Params.J = [[0.27,0.00,-0.01];...
%            [0.00,0.10,0.00];...
%            [-0.01,0.00,0.24]];

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

% Params.J = [[0.268649,-0.0009353033,-0.0072718662];...
%             [-0.0009353033,0.101493,0.0008260417];...
%             [-0.0072718662,0.0008260417,0.240842]];


% wheel's inertia [kg*m^2]
m_rw = 0.400;
Dimensions_rw = [67;25;67]./1000; 
Params.Jw = m_rw/12 * (Dimensions_rw(1)^2 + Dimensions_rw(2)^2);
% Params.Jw = 5.01e-5;


zeta = 0.7; % Damping ratio
wn = 0.1; % omega_n - Natural frequency 

% PD Controller gains:
gain_mult = 1;
kp = gain_mult*2*wn*wn*Params.J(1,1); % Proportional controller gain
kd = 2*zeta*wn*10*Params.J(1,1); % Derivative controller gain
% Assigning a gain matrix:
Params.gains.kpx = kp;
Params.gains.kdx = kd;
Params.gains.kpy = kp;
Params.gains.kdy = kd;
Params.gains.kpz = kp;
Params.gains.kdz = kd;


% Initial conditions
initial_phi = deg2rad(0);
initial_theta = deg2rad(0);
initial_psi = deg2rad(0);
eul_i = [initial_phi,initial_theta,initial_psi];
% Convert to quaternion
q_i = eul2quat(eul_i);
q_i = flip(q_i);
% Set initial conditions to simulink
Params.q0 = q_i;
Params.w0 = [0;0;5]; % Angular Velocity [deg/sec]
Params.w0_W = 0*[1,1,1,1]';

% Params.q0 = eul2quat([pi-initial_phi,-initial_theta,initial_psi]);
% Params.q0 = [deg2rad(10)  ,  deg2rad(10)  ,  deg2rad(10)  ,  1]; % Initial quaternion
% Params.q0 = [0,0,0,1`]; % Initial quaternion
% Params.q0 = eul2quat(eul_i);
% Params.q0 = flip(Params.q0);

% Target conditions
target_phi = deg2rad(0);
target_theta = deg2rad(0);
target_psi = deg2rad(0);
eul_t = [target_phi,target_theta,target_psi];
% Convert to quaternion
q_t = eul2quat(eul_t);
q_t = flip(q_t);
% Set initial conditions to simulink
Params.q_T = q_t;
% Params.q_T = [0.1767767,0.3061862,0.1767767,0.9185587]'; % Target quaternion (this quaternion sets the satellite to 30 degrees)
% Params.q_T = [0,0,0,1]'; % Target quaternion same as initial quaternion, used for checking detumbling for now


% Wheel configuration
% Pyramid Configuration (the chosen one)
aux = 1/sqrt(3); % Factoring by 1/sqrt(3) to get nicer looking vectors in the code
Params.W1 = aux * [1;-1;1]; % Coordinates of the corresponding wheel
Params.W2 = aux * [-1;1;1]; 
Params.W3 = aux * [-1;-1;1]; 
Params.W4 = aux * [1;1;1]; 

Params.W = [Params.W1,Params.W2,Params.W3,Params.W4]; % Wheel config matrix
Params.Winv = pinv(Params.W); % Inverse of the wheel config matrix

Params.torquebox = load('torquebox_modified.mat'); % Given by the manufacturer, the input is the controller output, the output is the torque working on each wheel 

% Wheel limits 
H_lim = 0.03; % Angular momentum limit [Nms] 
T_lim = 2e-3; % Torque limit [Nm]

ss_eul = 30; % Target Euler angle for cruise control loop [degree]


% External Torque 
SolarRadiation_torque = [0;0;0];
Drag_torque = [0;0;0];
Params.External_Torque  = Drag_torque + SolarRadiation_torque;


% Magnetotourquers  
Magneto_torque = [0;0;0];
Params.Magneto_Torque  = Magneto_torque;

%% Simulate and extract the data

Params.tf = 400; % Simulation time [sec]
Outsim = sim('Control_Sim'); % Simulink model
Data = Outsim.Data.signals.values(:,:,:);
Data = reshape(Data,[26 length(Data)]);
q_t = Data(1:4,:); % Quaternion vector in time
w_t = Data(5:7,:); % Angular velocity vector in time
q_error = Data(8:11,:); % Quaternion error vector in time (not important for now)
w_error = Data(12:14,:); % Angular momentum vector in time
Tc = Data(15:18,:); % Torque controller
TW = Data(19:22,:); % Torque on wheels
HW = Data(23:26,:); % Angular momentum on wheels
t = Outsim.tout;    % Time vector

%% Plotting the data 
% Angular velocity 
w_t = rad2deg(w_t);
figure,
subplot(2,1,1)
plot(t,w_t);
hold on
plot([0 Params.tf],[-0.02,-0.02],'k--','LineWidth',1.00);
plot([0 Params.tf],[0.02,0.02],'k--','LineWidth',1.00);
ylabel('\omega(t) [deg/sec]');
grid on 
legend('P','Q','R');
title('Angular velocity');
xlabel('t[sec]');

% Euler angles
eul = quat2eul([q_t(4,:)',q_t(1,:)',q_t(2,:)',q_t(3,:)']);
eul = rad2deg((eul));
% % figure,
subplot(2,1,2)
plot(t,fliplr(eul))
hold on
plot([0 Params.tf],[ss_eul,ss_eul]*0.98,'k--','LineWidth',1.00);
plot([0 Params.tf],[ss_eul,ss_eul]*1.02,'k--','LineWidth',1.00);
ylabel('eul(t) [deg]');
grid on 
legend('\phi','\theta','\psi');
title('Euler angles');
xlabel('t[sec]');

% subplot(2,1,2)
% plot(t,q_t)
% xlabel('t');
% ylabel('q(t)');
% grid on 
% legend('q1','q2','q3','q4');

% Wheels torque
figure,
subplot(2,1,1)
plot(t,TW); 
hold on;
% plot(t,Tc,'--');
plot([0,t(end)],T_lim*ones(1,2),'--r');
plot([0,t(end)],-T_lim*ones(1,2),'--r');
xlabel('t[sec]');
ylabel('Wheels torque [Nm]');
lgd = legend('RW 1 A','RW 2 A','RW 3 A','RW 4 A');
lgd.NumColumns = 2;
grid on
title('Wheels torque');

subplot(2,1,2)
plot(t,HW);
hold on;
plot([0,t(end)],H_lim*ones(1,2),'--r');
plot([0,t(end)],-H_lim*ones(1,2),'--r');
xlabel('t[sec]');
ylabel('Wheels angular momentum [Nms]');
legend('RW 1','RW 2','RW 3','RW 4');
grid on
title('Wheels angular momentum');

% quaternion error (Shay said that it's unnecessary)
% figure,
% plot(t,q_error)
% hold on
% plot([0 Params.tf],[-0.02,-0.02],'k--','LineWidth',1.00);
% plot([0 Params.tf],[0.02,0.02],'k--','LineWidth',1.00);
% plot([0,Params.tf],[0.98,0.98],'k--','LineWidth',1.00);
% plot([0 Params.tf],[1.02,1.02],'k--','LineWidth',1.00);
% xlabel('t[sec]');
% ylabel('\delta q');
% legend('\delta q_1','\delta q_2','\delta q_3','\delta q_4');
% grid on
% title('Quaternion error');

% angle error
angle_error = 2*atan2d(sqrt(q_error(1,:).^2+q_error(2,:).^2+q_error(3,:).^2),q_error(4,:));
figure,
plot(t,angle_error);
xlabel('t[sec]');
ylabel('\phi_{error}[deg]');
grid on







%% expirements:

mu = 3.986004418*10^14; % [m^3/sec^2]
R_eq = 6378.137*10^3; % [m]
J2 = 1.08264*10^(-3); % 

% Sat Initailize Conditions:
alt = 700*10^3; %[m]
a = alt + R_eq; %[m]
ecc = 0.1;
inc = deg2rad(55); %[rad]
w = deg2rad(10); %[rad]
Omega = deg2rad(15); %[rad]
f = 0; %[rad]
Sat0_vec_OE = [a; ecc; inc; w; Omega; f];
Sat0_vec_Cart = OE2Cart(Sat0_vec_OE,mu);
%Time span:
t_init = 0;
t_final = Params.tf; % simulation time [s]
tspan = t_init:0.01:t_final;
[t, Sat_vec_Cart] = ode45(@ODE2BP_Unperturbed_Cart,t, Sat0_vec_Cart);

function dYdt = ODE2BP_Unperturbed_Cart(t, Y)
    u = 3.986004418*10^(14); % [m^3/sec^2]

    x = Y(1); % [m]
    y = Y(2); % [m]
    z = Y(3); % [m]
    Vx = Y(4); % [m/s]
    Vy = Y(5); % [m/s]
    Vz = Y(6); % [m/s]

    r = sqrt(x^2 + y^2 + z^2); %[m]

    xddot = -(u*x)/(r^3); %-u/(r^(3/2)*x); % [m/s^2]
    yddot = -(u*y)/(r^3); %-u/(r^(3/2)*y); % [m/s^2]
    zddot = -(u*z)/(r^3); %-u/(r^(3/2)*z); % [m/s^2]
    dYdt = [Vx;Vy;Vz;xddot;yddot;zddot]; % Ydt
end


