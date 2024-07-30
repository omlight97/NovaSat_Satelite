function [Outsim]= RunSimulink(q_eul_i,w_i,q_t)
    Params.q0 = q_eul_i;
    Params.w0 = w_i; % Angular Velocity [deg/sec]
    Params.w0_W = deg2rad([0;0;0;0]);
    Params.q_T = q_t;
    % External Torques
    SolarRadiation_External_Torque = [0;0;0];
    Drag_External_Torque = [0;0;0];
    Magneto_External_Torque = [0;0;0];
    Params.External_Torque  = SolarRadiation_External_Torque + Drag_External_Torque + Magneto_External_Torque;
    % Inertia matrix [kg*m^2]
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
    % PD Controller gains:
    zeta = 0.7; % Damping ratio
    wn = 0.1; % omega_n - Natural frequency
    gain_mult = 1;
    kp = gain_mult*2*wn*wn*Params.J(1,1); % Proportional controller gain
    kd = 2*zeta*wn*10*Params.J(1,1);      % Derivative controller gain
    Params.gains.kpx = kp;
    Params.gains.kdx = kd;
    Params.gains.kpy = kp;
    Params.gains.kdy = kd;
    Params.gains.kpz = kp;
    Params.gains.kdz = kd;
    % Wheel configuration - Pyramid 
    aux = 1/sqrt(3); % Factoring by 1/sqrt(3) to get nicer looking vectors in the code
    Params.W1 = aux * [1;-1;1]; % Coordinates of the corresponding wheel
    Params.W2 = aux * [-1;1;1]; 
    Params.W3 = aux * [-1;-1;1]; 
    Params.W4 = aux * [1;1;1]; 
    Params.W = [Params.W1,Params.W2,Params.W3,Params.W4]; % Wheel config matrix
    Params.Winv = pinv(Params.W); % Inverse of the wheel config matrix
    Params.torquebox = load('torquebox_modified.mat'); % Given by the manufacturer, the input is the controller output, the output is the torque working on each wheel 
    % Wheel limits 
    H_lim = 0.0015;%0.03; % Angular momentum limit [Nms] 
    T_lim = 2e-3; % Torque limit [Nm]
    Params.H_lim = H_lim;
    Params.tf = 60;
    % Simulate - attitude control
    assignin('base','Params',Params)
    Outsim = sim('Control_Sim_dev');
end