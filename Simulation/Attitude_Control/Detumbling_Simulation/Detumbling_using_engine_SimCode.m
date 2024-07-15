clear all
clc

Params.ImpulseBit_BiProp = 10; %[N*sec]
theta = deg2rad(45);
gamma1 = deg2rad(10);
gamma2 = deg2rad(7);
gamma3 = deg2rad(10);
gamma4 = deg2rad(7);

%UPDATED- Moments od inertia -Taken at the center of mass and aligned with the output coordinate system
Params.J = [[0.37641186264,-0.00057183860,0.00139429076];...
            [-0.00057183860,0.20254810096,-0.00032738262];...
            [0.00139429076, -0.00032738262,0.31450564820]];


w_c = [0,0,0]';
Params.w0 = [30,0,0]'; % Angular Velocity [deg/sec]

vector_thruster1 = [-sin(gamma1)*sin(theta) -sin(gamma1)*cos(theta) -cos(gamma1)]/...
    norm([-sin(gamma1)*sin(theta) -sin(gamma1)*cos(theta) -cos(gamma1)]);
arm_vec_1 =[-99.85 -86.22 -152.16].*10^-3 ; %[m]

vector_thruster2 = [sin(gamma2)*sin(theta) -sin(gamma2)*cos(theta) -cos(gamma2)]/...
    (norm([sin(gamma2)*sin(theta) -sin(gamma2)*cos(theta) -cos(gamma2)]));
arm_vec_2 = [71.19 -85.97 -152.16].*10^-3 ; %[m]

vector_thruster3 = [-sin(gamma3)*sin(theta) sin(gamma3)*cos(theta) -cos(gamma3)]/...
    (norm([-sin(gamma3)*sin(theta) sin(gamma3)*cos(theta) -cos(gamma3)]));
arm_vec_3 = [-100.1 84.8 -152.16].*10^-3 ; %[m]

vector_thruster4 = [sin(gamma4)*sin(theta) sin(gamma4)*cos(theta) -cos(gamma4)]/...
    (norm([sin(gamma4)*sin(theta) sin(gamma4)*cos(theta) -cos(gamma4)]));
arm_vec_4 = [70.94 85.07 -152.16].*10^-3 ; %[m]

moment_thruster1 = cross(arm_vec_1,vector_thruster1) ;
moment_thruster2 = cross(arm_vec_2,vector_thruster2);
moment_thruster3 = cross(arm_vec_3,vector_thruster3);
moment_thruster4 = cross(arm_vec_4,vector_thruster4);


% gains for +X axis moment
syms  a1 b1 c1 d1 positive
eqn_s_X_p = abs(a1).*moment_thruster1+abs(b1).*moment_thruster2+abs(c1).*moment_thruster3+abs(d1).*moment_thruster4 == [1 0 0];
s_X_p=solve(eqn_s_X_p,[a1 b1 c1 d1], 'Real',true);

% gains for -X axis moment
syms  a2 b2 c2 d2 positive
eqn_s_X_m = abs(a2).*moment_thruster1+abs(b2).*moment_thruster2+abs(c2).*moment_thruster3+abs(d2).*moment_thruster4 == [-1 0 0];
s_X_m=solve(eqn_s_X_m,[a2 b2 c2 d2], 'Real',true);

% gains for +Y axis moment
syms a3 b3 c3 d3 positive
eqn_s_Y_p = abs(a3).*moment_thruster1+abs(b3).*moment_thruster2+abs(c3).*moment_thruster3+abs(d3).*moment_thruster4 == [0 1 0];
s_Y_p=solve(eqn_s_Y_p,[a3 b3 c3 d3], 'Real',true);

% gains for -Y axis moment
syms  a4 b4 c4 d4 positive
eqn_s_Y_m = abs(a4).*moment_thruster1+abs(b4).*moment_thruster2+abs(c4).*moment_thruster3+abs(d4).*moment_thruster4 == [0 -1 0];
s_Y_m=solve(eqn_s_Y_m,[a4 b4 c4 d4], 'Real',true);

% gains for +Z axis moment
syms  a5 b5 c5 d5 positive
eqn_Z_p = abs(a5).*moment_thruster1+abs(b5).*moment_thruster2+abs(c5).*moment_thruster3+abs(d5).*moment_thruster4 == [0 0 1];
s_Z_p=solve(eqn_Z_p,[a5 b5 c5 d5], 'Real',true);


% gains for -Z axis moment
syms  a6 b6 c6 d6 positive
eqn_Z_m = abs(a6).*moment_thruster1+abs(b6).*moment_thruster2+abs(c6).*moment_thruster3+abs(d6).*moment_thruster4 == [0 -1 0];
s_Z_m =solve(eqn_Z_m,[a6 b6 c6 d6], 'Real',true);

Roll.p.thruster1 = double(s_X_m.a2);
Roll.p.thruster2 = double(s_X_m.b2);
Roll.p.thruster3 = double(s_X_m.c2);
Roll.p.thruster4 = double(s_X_m.d2);

Roll.m.thruster1 = double(s_X_p.a1);
Roll.m.thruster2 = double(s_X_p.b1);
Roll.m.thruster3 = double(s_X_p.c1);
Roll.m.thruster4 = double(s_X_p.d1);

Pitch.p.thruster1 = double(s_Y_m.a4);
Pitch.p.thruster2 = double(s_Y_m.b4);
Pitch.p.thruster3 = double(s_Y_m.c4);
Pitch.p.thruster4 = double(s_Y_m.d4);

Pitch.m.thruster1 = double(s_Y_p.a3);
Pitch.m.thruster2 = double(s_Y_p.b3);
Pitch.m.thruster3 = double(s_Y_p.c3);
Pitch.m.thruster4 = double(s_Y_p.d3);

Yaw.p.thruster1 = double(s_Z_m.a6);
Yaw.p.thruster2 = double(s_Z_m.b6);
Yaw.p.thruster3 = double(s_Z_m.c6);
Yaw.p.thruster4 = double(s_Z_m.d6);

Yaw.m.thruster1 = double(s_Z_p.a5);
Yaw.m.thruster2 = double(s_Z_p.b5);
Yaw.m.thruster3 = double(s_Z_p.c5);
Yaw.m.thruster4 = double(s_Z_p.d5);

%Gains determination- P controller
gain.Roll.p = 1;
gain.Roll.m = 1;
gain.Pitch.p = 1;
gain.Pitch.m = 1;
gain.Yaw.p = 1;
gain.Yaw.m = 1;

%Location of each nozzle exit- taken from center of mass
Params.arm_vec1 = [-99.85 -86.22 -152.16].*10^-3 ; %[m]
Params.arm_vec2 = [71.19 -85.97 -152.16].*10^-3 ; %[m]
Params.arm_vec3 = [-100.1 84.8 -152.16].*10^-3 ; %[m]
Params.arm_vec4 = [70.94 85.07 -152.16].*10^-3 ; %[m]

%Unit vector of thrust vectors for each thruster
Params.thrust_vec1 = [-sin(gamma1)*sin(theta) -sin(gamma1)*cos(theta) -cos(gamma1)]/...
    (norm([-sin(gamma1)*sin(theta) -sin(gamma1)*cos(theta) -cos(gamma1)])); %[m]
Params.thrust_vec2 = [sin(gamma2)*sin(theta) -sin(gamma2)*cos(theta) -cos(gamma2)]/...
    (norm([sin(gamma2)*sin(theta) -sin(gamma2)*cos(theta) -cos(gamma2)])); %[m]
Params.thrust_vec3 = [-sin(gamma3)*sin(theta) sin(gamma3)*cos(theta) -cos(gamma3)]/...
    (norm([-sin(gamma3)*sin(theta) sin(gamma3)*cos(theta) -cos(gamma3)])); %[m]
Params.thrust_vec4 = [sin(gamma4)*sin(theta) sin(gamma4)*cos(theta) -cos(gamma4)]/...
    (norm([sin(gamma4)*sin(theta) sin(gamma4)*cos(theta) -cos(gamma4)])); %[m]




%Time constant- for all thrusters (TBD)
tau_Thrusters= 30e-03; %[sec]

% Getting the thrust req depending on del_omega (AKA del_moment), using
% relevant gains
Roll_thrust_operator = abs(-1/( (1/(norm(Params.arm_vec1)))*(-sin(gamma1)*cos(theta)*Params.arm_vec1(3)+cos(gamma1)*Params.arm_vec1(2))+...
        (1/(norm(Params.arm_vec2)))*(-sin(gamma2)*cos(theta)*Params.arm_vec2(3)+cos(gamma2)*Params.arm_vec2(2))+...
        (1/(norm(Params.arm_vec3)))*(sin(gamma3)*sin(theta)*Params.arm_vec3(3)-cos(gamma3)*Params.arm_vec3(1))+...
        (1/(norm(Params.arm_vec4)))*(sin(gamma4)*cos(theta)*Params.arm_vec4(3)+cos(gamma4)*Params.arm_vec4(2))));

Pitch_thrust_operator = abs(-1/( (1/(norm(Params.arm_vec1)))*(sin(gamma1)*sin(theta)*Params.arm_vec1(3)-cos(gamma1)*Params.arm_vec1(1))+...
        (1/(norm(Params.arm_vec2)))*(-sin(gamma2)*sin(theta)*Params.arm_vec2(3)-cos(gamma2)*Params.arm_vec2(1))+...
        (1/(norm(Params.arm_vec3)))*(sin(gamma3)*cos(theta)*Params.arm_vec3(3)+cos(gamma3)*Params.arm_vec3(2))+...
        (1/(norm(Params.arm_vec4)))*(sin(gamma4)*sin(theta)*Params.arm_vec4(2)-cos(theta)*sin(gamma4)*Params.arm_vec4(1))));

Yaw_thrust_operator = abs(-1/( (1/(norm(Params.arm_vec1)))*(-sin(gamma1)*sin(theta)*Params.arm_vec1(2)+cos(theta)*sin(gamma1)*Params.arm_vec1(1))+...
        (1/(norm(Params.arm_vec2)))*(-sin(gamma2)*sin(theta)*Params.arm_vec2(2)-cos(theta)*sin(gamma2)*Params.arm_vec2(1))+...
        (1/(norm(Params.arm_vec3)))*(-sin(gamma3)*sin(theta)*Params.arm_vec3(2)-sin(gamma3)*cos(theta)*Params.arm_vec3(1))+...
        (1/(norm(Params.arm_vec4)))*(-sin(gamma4)*sin(theta)*Params.arm_vec4(3)-cos(gamma4)*Params.arm_vec4(1))));





%% Simulate and extract the data

Params.tf = 3; % Simulation time [sec]
Outsim = sim('Maneuver_sim1'); % Simulink model
Data = Outsim.Data.signals.values(:,:,:);

w_t = Data(1:3,:); % Angular velocity vector in time
TW = Data(4:6,:); % Torque on wheels
t = Outsim.tout;    % Time vector

%% Plotting the data 
% Angular velocity 
w_t = rad2deg(w_t);
figure,
subplot(2,1,1)
plot(t,w_t);
hold on
xlabel('t[sec]');ylabel('\omega(t) [deg/sec]');
grid on 
legend('P','Q','R');
title('Angular velocity');


% Engine torque
figure,
subplot(2,1,1)
plot(t,TW); 
xlabel('t [sec]');
ylabel('Engine torque [Nm]');
lgd = legend('Around x',' Around y','Around z');
lgd.NumColumns = 2;
grid on
title('Total Engine torque');