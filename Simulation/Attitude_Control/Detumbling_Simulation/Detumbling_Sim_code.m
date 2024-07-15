clc; 
clear variables;
close all;
set(0,'DefaultAxesFontName','Times','DefaultAxesFontSize',12)
set(0,'defaultLineLineWidth',1.5,'defaultLineMarkerSize',8)
%% Parameters
% Inertia matrix [kg*m^2]
% Params.J = [[0.27,0.00,-0.01];...
%            [0.00,0.10,0.00];...
%            [-0.01,0.00,0.24]];

Params.J = [[0.3764118626,-0.0005718386,0.00139429076];...
            [-0.0005718386,0.202548101,-0.00032738262];...
            [0.00139429076,-0.00032738262,0.3145056482]];

% wheel's inertia [kg*m^2]
Params.Jw = 5.01e-5;

% roll_rate = input("Please enter initial roll angular rate: ");
% pitch_rate = input("Please enter initial pitch angular rate: ");
% yaw_rate = input("Please enter initial yaw angular rate: ");

% roll_rate = 16;
% pitch_rate = 10;
% yaw_rate = 10;
% kc = -20.254;
% kp = 0.48*kc;
% kd = kc/1.6;
% ki = kc/3;
random_vec = rand(3,1);
initial_ang_velocity = 30*random_vec/norm(random_vec);
norm(initial_ang_velocity)

zeta = 1.0; % Damping ratio
wn = 0.42; % omega_n - Natural frequency 

% PI Controller gains:

% Assigning a gain matrix:
Params.gains.kpx = 2*wn*wn*Params.J(1,1);
Params.gains.kix = 2*zeta*wn*Params.J(1,1);
Params.gains.kpy = 2*wn*wn*Params.J(2,2);
Params.gains.kiy = 2*zeta*wn*Params.J(2,2);
Params.gains.kpz = 2*wn*wn*Params.J(3,3);
Params.gains.kiz = 2*zeta*wn*Params.J(3,3);


% Initial conditions
% Params.w0 = [roll_rate,pitch_rate,yaw_rate]'; % Angular Velocity [deg/sec]
Params.w0 = initial_ang_velocity';
Params.w0_W = 0*[1,1,1,1]';


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
% T_lim = 2e-3; % Torque limit [Nm]

% ss_eul = 30; % Target Euler angle for cruise control loop [degree]

%% Simulate and extract the data

Params.tf = 300; % Simulation time [sec]
Outsim = sim('Detumbling_Control_SimPID'); % Simulink model
Data = Outsim.Data.signals.values(:,:,:);
Data = reshape(Data,[18 length(Data)]);
w_t = Data(1:3,:); % Angular velocity vector in time
w_error = Data(4:6,:); % Angular momentum vector in time
Tc = Data(7:10,:); % Torque controller
TW = Data(11:14,:); % Torque on wheels
HW = Data(15:18,:); % Angular momentum on wheels
t = Outsim.tout;    % Time vector

%% Plotting the data 
% Angular velocity 
w_t = rad2deg(w_t);
figure()
plot(t,w_t);
hold on
plot([0 Params.tf],[-0.02,-0.02],'k--','LineWidth',1.00);
plot([0 Params.tf],[0.02,0.02],'k--','LineWidth',1.00);
% plot([0 Params.tf],[3,3],'r--','LineWidth',1.00);
% plot([0 Params.tf],[-3,-3],'r--','LineWidth',1.00);
ylabel('\omega(t) [deg/sec]');
grid on 
legend('P','Q','R');
title('Angular velocity');
xlabel('t[sec]');

% Euler angles
% eul = quat2eul([q_t(4,:)',q_t(1,:)',q_t(2,:)',q_t(3,:)']);
% eul = rad2deg((eul));
% % figure,
% subplot(2,1,2)
% plot(t,fliplr(eul))
% hold on
% plot([0 Params.tf],[roll,roll]*0.98,'k--','LineWidth',1.00);
% plot([0 Params.tf],[roll,roll]*1.02,'k--','LineWidth',1.00);
% plot([0 Params.tf],[pitch,pitch]*0.98,'k--','LineWidth',1.00);
% plot([0 Params.tf],[pitch,pitch]*1.02,'k--','LineWidth',1.00);
% plot([0 Params.tf],[yaw,yaw]*0.98,'k--','LineWidth',1.00);
% plot([0 Params.tf],[yaw,yaw]*1.02,'k--','LineWidth',1.00);
% ylabel('eul(t) [deg]');
% grid on 
% legend('\phi','\theta','\psi');
% title('Euler angles');
% xlabel('t[sec]');

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



%%

pxx1 = -1./roots(tf2(1,1).Denominator);
pxy1 = -1./roots(tf2(2,1).Denominator);
pxz1 = -1./roots(tf2(3,1).Denominator);

pyx1 = -1./roots(tf2(1,2).Denominator);
pyy1 = -1./roots(tf2(2,2).Denominator);
pyz1 = -1./roots(tf2(3,2).Denominator);

pzx1 = -1./roots(tf2(1,3).Denominator);
pzy1 = -1./roots(tf2(2,3).Denominator);
pzz1 = -1./roots(tf2(3,3).Denominator);

pxx2 = -1./roots(tf3(1,1).Denominator);
pxy2 = -1./roots(tf3(2,1).Denominator);
pxz2 = -1./roots(tf3(3,1).Denominator);

pyx2 = -1./roots(tf3(1,2).Denominator);
pyy2 = -1./roots(tf3(2,2).Denominator);
pyz2 = -1./roots(tf3(3,2).Denominator);

pzx2 = -1./roots(tf3(1,3).Denominator);
pzy2 = -1./roots(tf3(2,3).Denominator);
pzz2 = -1./roots(tf3(3,3).Denominator);


