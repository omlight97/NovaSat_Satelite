clc; 
clear variables;
close all;
set(0,'DefaultAxesFontName','Times','DefaultAxesFontSize',12)
set(0,'defaultLineLineWidth',1.5,'defaultLineMarkerSize',8)
%% Pararmeters
% Inertia matrix [kg*m^2]

Params.J = [[0.268649,-0.0009353033,-0.0072718662];...
            [-0.0009353033,0.101493,0.0008260417];...
            [-0.0072718662,0.0008260417,0.240842]];

% wheel's inertia [kg*m^2]
Params.Jw = 5.01e-5;

roll = input("Please enter target roll angle: "); % Desired roll angle input
pitch = input("Please enter target pitch angle: "); % Desired pitch angle input
yaw = input("Please enter target yaw angle: "); % Desired yaw angle input



zeta = 1.0; % Damping ratio
wn = 0.1; % omega_n - Natural frequency 

% PD Controller gains:
% Assigning a gain matrix:
Params.gains.kpx = 2*wn*wn*Params.J(1,1);
Params.gains.kdx = 2*10*zeta*wn*Params.J(1,1);
Params.gains.kpy = 2*wn*wn*Params.J(2,2);
Params.gains.kdy = 2*10*zeta*wn*Params.J(2,2);
Params.gains.kpz = 2*wn*wn*Params.J(3,3);
Params.gains.kdz = 2*10*zeta*wn*Params.J(3,3);


% Initial conditions
Params.q0 = [0,0,0,1]'; % Initial quaternion
Params.w0 = [20,20,20]'; % Angular Velocity [deg/sec]
Params.w0_W = 0*[1,1,1,1]';

% Target conditions
Params.q_T = euler_to_quaternion(roll,pitch,yaw); % Desired target quaternion



% Wheel configuration
% Pyramid Configuration (the chosen one)
aux = 1/sqrt(3); % Factoring by 1/sqrt(3) to get nicer looking vectors in the code
Params.W1 = aux * [1;-1;1]; % Coordinates of the corresponding wheel
Params.W2 = aux * [-1;1;1]; % same
Params.W3 = aux * [-1;-1;1]; 
Params.W4 = aux * [1;1;1]; 

Params.W = [Params.W1,Params.W2,Params.W3,Params.W4]; % Wheel config matrix
Params.Winv = pinv(Params.W); % Inverse of the wheel config matrix

Params.torquebox = load('torquebox_modified.mat'); % Given by the manufacturer, the input is the controller output, the output is the torque working on each wheel 

% Wheel limits 
H_lim = 0.03; % Angular momentum limit [Nms] 

%% Simulate and extract the data

Params.tf = 1000; % Simulation time [sec]
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
plot([0 Params.tf],[3,3],'r--','LineWidth',1.00);
plot([0 Params.tf],[-3,-3],'r--','LineWidth',1.00);
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
plot([0 Params.tf],[roll, roll]*0.98,'k--','LineWidth',1.00);
plot([0 Params.tf],[roll, roll]*1.02,'k--','LineWidth',1.00);
plot([0 Params.tf],[pitch, pitch]*0.98,'k--','LineWidth',1.00);
plot([0 Params.tf],[pitch, pitch]*1.02,'k--','LineWidth',1.00);
plot([0 Params.tf],[yaw, yaw]*0.98,'k--','LineWidth',1.00);
plot([0 Params.tf],[yaw, yaw]*1.02,'k--','LineWidth',1.00);
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

