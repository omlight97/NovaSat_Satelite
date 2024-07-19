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
% Convert to quaternion
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
% Convert to quaternion
q_eul_t = eul2quat(eul_t);
q_eul_t = flip(q_eul_t);

% Initialize target angular rate [rad/sec]
p_t = 0;
q_t = 0;
r_t = 0;
w_t = [p_t;q_t;r_t];

%% Helping calculations
% I_to_B matrix
I_to_B = rotx(phi_i)*rotz(psi_i)*roty(tet_i);

% Magnetic Torque
% Magnetic_Torque_B = Magnetic_field_function(I_to_B,Params.Magnetic_Field_I,Params.Magnetic_Dipol_B);

% Current Satellite\Sun\ImmarSat position
SatPosition = Params.SatPosition';
SunPosition = Params.SunPosition';
CommsSatPosition = Params.CommsSatPosition'; 

% Max angular rate [rad/sec]
w_max = [Params.p_max;Params.q_max;Params.r_max];

%% Attitude logic
if Flags.Communication
    % Communication State Logic - attitude towards: Immar Satellites
    [eul_t,w_t] = Comms_Att_Logic(I_to_B,SatPosition,CommsSatPosition);
elseif Flags.IsDay
    % Sun Search State Logic - attitude towards: Sun
    [eul_t,w_t] = SunSearch_Att_Logic(eul_t,I_to_B,SatPosition,SunPosition,Flags);
elseif ~Flags.IsDay
    % Night state - minimum energy, keep current angle\rates as long as
    % not exceeding max rate
    [eul_t,w_t] = Night_Att_Logic(w_max);
end
% Convert euler angles to quaternion
q_eul_t = eul2quat(eul_t);
q_eul_t = flip(q_eul_t);


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
    % Simulate - attitude control
    Outsim = sim('Control_Sim');
    % Export data from simulation
    Data = Outsim.Data.signals.values(:,:,:);
    Data = reshape(Data,[17 length(Data)]);
    q_f = Data(1:4,end); % final quaternion
    w_f = Data(5:7,end); % [rad/sec] final angular velocity
    q_error = Data(8:11,end); % error quaternion
    w_error = Data(12:14,end);% [rad/sec] angular velocity error
    Tc = Data(15:17,end); % [Nm] Torque command
    % Convert quaternion to euler angles - ZYX sequnce
    eul_f = quat2eul(flip(q_f)');
    eul_error = quat2eul(flip(q_error)');

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


function [eul_t,w_t] = Comms_Att_Logic(I_to_B,SatPosition_I,CommsSatPosition_I)
    % Communication State Logic
    Z_B = [1;0;0]; %Z axis in body frame
    Sat2Comms_I = SatPosition_I - CommsSatPosition_I;
    Sat2Comms_B = I_to_B*Sat2Comms_I;
    uSat2Comms_B = Sat2Comms_B./norm(Sat2Comms_B); 
    psi_t = asin(Z_B(3)/uSat2Comms_B(2));
    tet_t = atan(uSat2Comms_B(2)/uSat2Comms_B(1)); 
    phi_t = 0;
    
    psi_t = 0;
    tet_t = 0;
    phi_t = 0;

    eul_t = [psi_t,tet_t,phi_t];

    % Target angular rate [rad/sec]
    p_t = 0;
    q_t = 0;
    r_t = 0;
    w_t = [p_t;q_t;r_t];
end

function [eul_t,w_t] = SunSearch_Att_Logic(eul_i,I_to_B,SatPosition_I,SunPosition_I,Flags)
    %% Sun Search State Logic
    % Current attitude in terms of euler angles [rad]
    psi_i = eul_i(1);
    tet_i = eul_i(2);
    phi_i = eul_i(3);
    
    % Calculations
    SunSensorFOV = deg2rad(120); %Sun sensor field of view
    SunSensorPositionVector_B = [0;1;0]; %Sun sensor Heading - assuming pointing side at the moment
    Sat2Sun_I = SatPosition_I - SunPosition_I;
    Sat2Sun_B = I_to_B*Sat2Sun_I;
    uSat2Sun_B = Sat2Sun_B./norm(Sat2Sun_B); % Satellite center to the center of the Sun (unit vector). - body frame
    isLOS2Sun = dot(SunSensorPositionVector_B, uSat2Sun_B) > cos(SunSensorFOV);

    isLOS2Sun = true; %% temp - until logic is completed and verifyied

    if(isLOS2Sun) % is sun in field of view
        psi_t = asin(SunSensorPositionVector_B(3)/uSat2Sun_B(2));
        tet_t = atan(uSat2Sun_B(2)/uSat2Sun_B(1)); 
        phi_t = 0;
        eul_t = [psi_t,tet_t,phi_t];
    else % Roatation sequence logic
        % Flags.sun_search.start_logic = 0;
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
        %3 - rotate -45 pitch and then 270 arund main axis
        elseif Flags.state_three_flag
            [eul_t,Flags.state_three_flag] = SunSearch_third_Manuver(Params.eul0,eul_i); % pass imu readings untill psi = psi0+360
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

function [eul_t,w_t] = Night_Att_Logic(w_max)
    % Target angular rate [rad/sec]
    % w_max=1;
    % w_safty_factor=1;
    % w_max_check = w_i > w_max*w_safty_factor;
    % if any(w_max_check)
    %     w_i = w_max(w_max_check);
    % end
    
    % Target attitude in terms of euler angles [rad]
    psi_t = 2;
    tet_t = 2;
    phi_t = 2;
    eul_t = [psi_t,tet_t,phi_t];
    
    % Target angular rate [rad/sec]
    p_t = 2;
    q_t = 2;
    r_t = 2;
    w_t = [p_t;q_t;r_t];
end