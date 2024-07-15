function [Next_Angular_State,Params,Flags] = Nova_Attitude_Prop(Current_Step_Angular,Flags,Params)
% NovaSat Attitude SIMULATION
% Originally by: May Alon (Jericco)
% NovaSAT editors: Shai Peled & Edos Osazuwa
% This function calculates the angular state of the satellite each step including angles, rates, 
% performances of reaction wheels and errors.

%% Current step euler angles and angular rates
% Current attitude in terms of euler angles [rad]
Psi   = Current_Step_Angular.Psi;
Tet = Current_Step_Angular.Theta;
Phi   = Current_Step_Angular.Phi;
eul_i = [Psi,Tet,Phi];
% Convert to quaternion
q_eul_i = eul2quat(eul_i);
q_eul_i = flip(q_eul_i);

% Current angular rate [rad/sec]
P = Current_Step_Angular.P;
Q = Current_Step_Angular.Q;
R = Current_Step_Angular.R;
w_i = [P;Q;R];
% Convert to quaternion
% q_w_i = eul2quat(w_i);
% q_w_i = flip(q_w_i);

%% Helping calculations
% I_to_B matrix
I_to_B = rotx(Phi)*rotz(Psi)*roty(Tet);

%Magnetic Torque
% Magnetic_Torque_B = Magnetic_field_function(I_to_B,Params.Magnetic_Field_I,Params.Magnetic_Dipol_B);


%% Attitude logic
if Flags.Communication 
    % Communication state
    % Target angular rate [rad/sec]
    p_t = 0;
    q_t = 0;
    r_t = 0;
    w_t = [p_t;q_t;r_t];
    % Target attitude in terms of euler angles [rad]
    psi_t = 0;
    tet_t = 0;
    phi_t = 0;
    eul_t = [psi_t,tet_t,phi_t];
    % Convert to quaternion
    q_t = eul2quat(eul_t);
    q_t = flip(q_t);
elseif Flags.isDay 
    % Sun Search state
    % Get sun vector from Sun Sensor - currently matlab online function approxECISunPosition()
    [Sat2Sun_B,isLOS2Sun] = SunSensorModel(Params.Sat_Pos_I,I_to_B,UTCTime,Flags.SunSensor);
    if(isLOS2Sun)
        tet_target = atan(Sat2Sun_B(2)/Sat2Sun_B(1)); %??
        psi_target = asin(SunSensorPositionVector(3)/uSat2Sun_B(2)); %??
        phi_target = 0;
        eul2target = [tet_target;psi_target;phi_target];
    % Roatation sequence logic
    elseif Flags.sun_search.initial_flag
        [Flags.sun_search.state_one_flag,Flags.sun_search.state_two_flag,Flags.sun_search.state_three_flag] = state_finder();
        if state_one_flag
            [psi,psi0,Flags.sun_search.rotation_one_done_flag] = rotate_psi_to_psi0_plus_360() % pass imu readings untill psi = psi0+360
            if Flags.sun_search.rotation_one_done_flag
                Flags.sun_search.state_one_flag = 0;
                Flags.sun_search.state_two_flag = 1;
            end
        elseif state_two_flag
            [psi,psi0,Flags.sun_search.rotation_two_done_flag] = rotate_theta_45_psi_to_psi0_plus_360() % pass imu readings untill psi = psi0+360
            if Flags.sun_search.rotation_two_done_flag
                Flags.sun_search.state_two_flag = 0;
                Flags.sun_search.state_three_flag = 1;
            end
        elseif state_three_flag
            [psi,psi0,Flags.sun_search.rotation_one_done_flag] = rotate_theta_m45_psi_to_psi0_plus_360() % pass imu readings untill psi = psi0+360
            if Flags.sun_search.rotation_three_done_flag
                Flags.sun_search.state_three_flag = 0;
                msgbox('Was not able to find sun after 3 rotation sequences.')
            end
        end
    else
        p_t = 1;
        q_t = 1;
        r_t = 1;
        w_t = [p_t;q_t;r_t];
        % Target attitude in terms of euler angles [rad]
        psi_t = 1;
        tet_t = 1;
        phi_t = 1;
        eul_t = [psi_t,tet_t,phi_t];
        % Convert to quaternion
        q_t = eul2quat(eul_t);
        q_t = flip(q_t);
    end
    % warning('Invalid state specified.');
else
    % Night state - minimum energy, keep current angle\rates as long as
      % not exceeding max rate
    % Target angular rate [rad/sec]
    w_max_check = w_i > w_max*w_safty_factor;
    if any(w_max_check)
        w_i = w_max(w_max_check);
    end        
    
    % Target angular rate [rad/sec]
    p_t = 2;
    q_t = 2;
    r_t = 2;
    w_t = [p_t;q_t;r_t];
    % Target attitude in terms of euler angles [rad]
    psi_t = 2;
    tet_t = 2;
    phi_t = 2;
    eul_t = [psi_t,tet_t,phi_t];
    % Convert to quaternion
    q_t = eul2quat(eul_t);
    q_t = flip(q_t);
end
 
OverrideSimulink = true;
if(OverrideSimulink)
    % Calculated next angular state
    Next_Angular_State.Psi = eul_t(1);
    Next_Angular_State.Theta = eul_t(2);
    Next_Angular_State.Phi = eul_t(3);
    
    Next_Angular_State.P = w_t(1);
    Next_Angular_State.Q = w_t(2);
    Next_Angular_State.R = w_t(3);
    Attitude_Control_Data = 0;
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
    Next_Angular_State.Psi = eul_f(1);
    Next_Angular_State.Theta = eul_f(2);
    Next_Angular_State.Phi = eul_f(3);
    
    Next_Angular_State.P = w_f(1);
    Next_Angular_State.Q = w_f(2);
    Next_Angular_State.R = w_f(3);
    
    Attitude_Control_Data.Psi_error = eul_error(1);
    Attitude_Control_Data.Theta_error = eul_error(2);
    Attitude_Control_Data.Phi_error = eul_error(3);
    
    Attitude_Control_Data.P_error = w_error(1);
    Attitude_Control_Data.Q_error = w_error(2);
    Attitude_Control_Data.R_error = w_error(3);
    
    Attitude_Control_Data.Lc = Tc(1);
    Attitude_Control_Data.Mc = Tc(2);
    Attitude_Control_Data.Nc = Tc(3);
end
end