function [Next_Step_Angular,Params,Flags] = Nova_Attitude_Prop(Current_Step_Angular,Params,Flags)
% NovaSat Attitude SIMULATION
% Originally by: May Alon (Jericco)
% NovaSAT editors: Shai Peled & Edos Osazuwa
% This function calculates the angular state of the satellite each step including angles, rates,
% performances of reaction wheels and errors.

%% Current step euler angles and angular rates
% Current attitude in terms of euler angles [rad]
psi_i   = Current_Step_Angular.Psi;
tet_i = Current_Step_Angular.Theta;
phi_i   = Current_Step_Angular.Phi;
eul_i = [psi_i,tet_i,phi_i];
% Convert euler angles to quaternion
q_eul_i = eul2quat(eul_i);
q_eul_i = flip(q_eul_i);

% Current angular rate [rad/sec]
p_i = Current_Step_Angular.P;
q_i = Current_Step_Angular.Q;
r_i = Current_Step_Angular.R;
w_i = [p_i;q_i;r_i];

%% Initialize taregt euler angles and angular rates
% Initialize targen attitude in terms of euler angles [rad]
psi_t = 0;
tet_t = 0;
phi_t = 0;
eul_t = [psi_t,tet_t,phi_t];

% Initialize target angular rate [rad/sec]
p_t = 0;
q_t = 0;
r_t = 0;
w_t = [p_t;q_t;r_t];

%% Helping calculations
% I_to_B matrix
% LVLH_to_Body = rotx(phi_i)*rotz(psi_i)*roty(tet_i);
LVLH_to_Body = rotx(phi_i)*roty(tet_i)*rotz(psi_i);

% Magnetic Torque
% Magnetic_Torque_B = Magnetic_field_function(I_to_B,Params.Magnetic_Field_I,Params.Magnetic_Dipol_B);

% Current Satellite\Sun\ImmarSat position
SatPosition_ECI = Params.SatPosition';
SunPosition_ECI = Params.SunPosition';
CommsSatPosition_ECI = Params.CommsSatPosition'; 

%Sat2Sun
Sat2Sun_ECI = SatPosition_ECI - SunPosition_ECI;
orbital_elements = table2array(Params.orbital_elements);
inc = deg2rad(orbital_elements(3));
RAAN = deg2rad(orbital_elements(4));
theta = deg2rad(orbital_elements(5) + orbital_elements(6));
Sat2Sun_LVLH = eci2LVLH(Sat2Sun_ECI,RAAN,inc,theta);
Sat2Sun_Body = LVLH_to_Body*Sat2Sun_LVLH;
%Sat2Comms
Sat2Comms_ECI = SatPosition_ECI - CommsSatPosition_ECI;
Sat2Comms_LVLH = eci2LVLH(Sat2Comms_ECI,RAAN,inc,theta);
Sat2Comms_Body = LVLH_to_Body*Sat2Comms_LVLH;

% Max angular rate [rad/sec]
w_max = [Params.p_max;Params.q_max;Params.r_max];

%% Attitude logic
if Flags.GRB
    % GRB State Logic - freeze attitude
    [eul_t,w_t] = GRB_Att_Logic(eul_i,w_i);
elseif Flags.Communication
    % Communication State Logic - attitude towards: Immar Satellites
    [eul_t,w_t] = Comms_Att_Logic(Sat2Comms_LVLH);
elseif Flags.IsDay
    % Sun Search State Logic - attitude towards: Sun
    [eul_t,w_t] = SunSearch_Att_Logic(Sat2Sun_LVLH,Flags);
elseif ~Flags.IsDay
    % Night state - minimum energy, keep current angle\rates as long as
    % not exceeding max rate
    [eul_t,w_t] = Night_Att_Logic(eul_i,w_max);
end

% Convert euler angles to quaternion
q_eul_t = eul2quat(eul_t);
q_eul_t = flip(q_eul_t);


%% Simulink
OverrideSimulink = true;
if(OverrideSimulink)
    % Calculated target angular state [rad]
    Next_Step_Angular.Psi = eul_t(1);
    Next_Step_Angular.Theta = eul_t(2);
    Next_Step_Angular.Phi = eul_t(3);
    % Calculated target angular rate [rad\s]
    Next_Step_Angular.P = w_t(1);
    Next_Step_Angular.Q = w_t(2);
    Next_Step_Angular.R = w_t(3);
    Params.Attitude_Control_Data = 0;
else
    % Set conditions to simulink
    [~,Outsim] = RunSimulink(q_eul_i,w_i,q_eul_t);

    % Export data from simulation
    Data = Outsim.Data.signals.values(:,:,:);
    Data = reshape(Data,[26 length(Data)]);
    q_f = Data(1:4,end); % final quaternion
    w_f = Data(5:7,end); % [rad/sec] final angular velocity
    q_error = Data(8:11,end); % error quaternion
    w_error = Data(12:14,end);% [rad/sec] angular velocity error
    Tc = Data(15:17,end); % [Nm] Torque command
    % Convert quaternion to euler angles - ZYX sequnce
    eul_f = quat2eul(flip(q_f(end))');
    eul_error = quat2eul(flip(q_error(end))');

    % Calculated next angular state
    Next_Step_Angular.Psi = eul_f(1);
    Next_Step_Angular.Theta = eul_f(2);
    Next_Step_Angular.Phi = eul_f(3);

    Next_Step_Angular.P = w_f(1);
    Next_Step_Angular.Q = w_f(2);
    Next_Step_Angular.R = w_f(3);

    Params.Attitude_Control_Data.Psi_error = eul_error(1);
    Params.Attitude_Control_Data.Theta_error = eul_error(2);
    Params.Attitude_Control_Data.Phi_error = eul_error(3);

    Params.Attitude_Control_Data.P_error = w_error(1);
    Params.Attitude_Control_Data.Q_error = w_error(2);
    Params.Attitude_Control_Data.R_error = w_error(3);

    Params.Attitude_Control_Data.Lc = Tc(1);
    Params.Attitude_Control_Data.Mc = Tc(2);
    Params.Attitude_Control_Data.Nc = Tc(3);
end
end

function [eul_t,w_t] = GRB_Att_Logic(eul_i,w_i)
    eul_t = eul_i;
    w_t = w_i;
end

function [eul_t,w_t] = Comms_Att_Logic(Sat2Comms_LVLH)
    %% Communication State Logic
    Z_LVLH = [0;0;1]; %Z axis in LVLH frame
    uSat2Comms_Body = Sat2Comms_LVLH./norm(Sat2Comms_LVLH); 
    eul_t = ang_between_vec(Z_LVLH,uSat2Comms_Body);

    % Target angular rate [rad/sec]
    p_t = 0;
    q_t = 0;
    r_t = 0;
    w_t = [p_t;q_t;r_t];
end

function [eul_t,w_t] = SunSearch_Att_Logic(Sun2Sun_LVLH,Flags)
    %% Sun Search State Logic
    Z_LVLH = [0;0;1]; %Z axis in LVLH frame
    SunSensorFOV = deg2rad(120); %Sun sensor field of view
    SunSensorPositionVector_B = [0;1;0]; %Sun sensor Heading - assuming pointing side at the moment

    uSat2Sun_B = Sun2Sun_LVLH./norm(Sun2Sun_LVLH); % Satellite center to the center of the Sun (unit vector). - body frame
    isLOS2Sun = dot(SunSensorPositionVector_B, uSat2Sun_B) > cos(SunSensorFOV);

    isLOS2Sun = true; %% temp - until logic is completed and verifyied

    if(isLOS2Sun) % is sun in field of view
        eul_t = ang_between_vec(Z_LVLH,uSat2Sun_B);
        % psi_t = asin(SunSensorPositionVector_B(3)/uSat2Sun_B(2));
        % tet_t = atan(uSat2Sun_B(2)/uSat2Sun_B(1)); 
        % phi_t = 0;
        % eul_t = [psi_t,tet_t,phi_t];
    else % Roatation sequence logic
        %0 - Initial sunsearch logic algorithm
        if Flags.sun_search.initial_flag
            Params.eul0 =  [psi_i,tet_i,phi_i];
            Flags.sun_search.initial_flag = 0;
            Flags.state_one_flag = 1;
            Flags.state_two_flag = 0;
            Flags.state_three_flag = 0;
        %1 - rotate 270 around main axis
        elseif Flags.state_one_flag
            [eul_t,Flags.state_one_flag,Flags.state_two_flag] = SunSearch_first_Manuver(Params.eul0,eul_i); % pass imu readings untill psi = psi0+270
            %if finished first manuver, reset the initial conditions for the second manuver
            if Flags.sun_search.state_one_flag == 0 && Flags.sun_search.state_two_flag == 1
                Params.eul0 =  [psi_i,tet_i,phi_i];
            end
        %2 - rotate 45 pitch and then 270 arund main axis
        elseif Flags.state_two_flag
            [eul_t,Flags.state_two_flag,Flags.state_three_flag] = SunSearch_second_Manuver(Params.eul0,eul_i); % pass imu readings untill psi = psi0+360
            %if finished second manuver, reset the initial conditions for the third manuver
            if Flags.sun_search.state_two_flag == 0 && Flags.sun_search.state_three_flag == 1
                Params.eul0 =  [psi_i,tet_i,phi_i];
            end
        %3 - rotate -90 pitch and then 270 arund main axis
        elseif Flags.state_three_flag
            [eul_t,Flags.state_three_flag] = SunSearch_third_Manuver(Params.eul0,eul_i); % pass imu readings untill psi = psi0+360
        %4 - Error
        else
            Flags.sun_search.initial_flag = 0;
            msgbox('Was not able to find sun after 3 rotation sequences.')
        end
    end

    % % Target attitude in terms of euler angles [rad]
    % psi_t = 1;
    % tet_t = 1;
    % phi_t = 1;
    % eul_t = [psi_t,tet_t,phi_t];

    % Target angular rate [rad/sec]
    p_t = 0;
    q_t = 0;
    r_t = 0;
    w_t = [p_t;q_t;r_t];
end

function [eul_t,w_t] = Night_Att_Logic(eul_i,w_max)
    % Target angular rate [rad/sec]
    % w_max=1;
    % w_safty_factor=1;
    % w_max_check = w_i > w_max*w_safty_factor;
    % if any(w_max_check)
    %     w_i = w_max(w_max_check);
    % end

    % Target attitude in terms of euler angles [rad]
    eul_t = eul_i;
    
    % Target angular rate [rad/sec]
    p_t = 2;
    q_t = 2;
    r_t = 2;
    w_t = [p_t;q_t;r_t];
end

function [Params,Outsim]= RunSimulink(q_eul_i,w_i,q_t)
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