function [Next_Angular_State,Flags] = Nova_Attitude_Prop2(States,Current_Step_Angular,Flags,Params)
% NovaSat Attitude SIMULATION
% Originally by: May Alon (Jericco)
% NovaSAT editors: Shai Peled & Edos Osazuwa
% This function calculates the angular state of the satellite each step including angles, rates, 
% performances of reaction wheels and errors.

% Current step euler angles and angular rates
psi   = Current_Step_Angular.angles(1);
tet = Current_Step_Angular.angles(2);
phi   = Current_Step_Angular.angles(3);

p = Current_Step_Angular.rate(1);
q = Current_Step_Angular.rate(2);
r = Current_Step_Angular.rate(3);

% Initial angular rate [rad/sec]
w_i = [p;q;r];
% Initial attitude in terms of euler angles [rad]
eul_i = [psi,tet,phi];
% Convert to quaternion
q_i = eul2quat(eul_i);
q_i = flip(q_i);

% Desired attitude and angular rate - target
SatState = Flags;
switch SatState
    case 0
      % Day state - 
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
    case 1
      % Night state -
        % Target angular rate [rad/sec]
        p_t = 1;
        q_t =1;
        r_t = 1;
        w_t = [p_t;q_t;r_t];
        % Target attitude in terms of euler angles [rad]
        psi_t =1;
        tet_t = 1;
        phi_t = 1;
        eul_t = [psi_t,tet_t,phi_t];
        % Convert to quaternion
        q_t = eul2quat(eul_t);
        q_t = flip(q_t);
    case 'SunSearch'
      % Sun Search state
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
    otherwise
      warning('Invalid state specified.');
end


if(true)
    % Next angular state
    Next_Angular_State.Psi = eul_t(1);
    Next_Angular_State.Theta = eul_t(2);
    Next_Angular_State.Phi = eul_t(3);
    
    Next_Angular_State.P = w_t(1);
    Next_Angular_State.Q = w_t(2);
    Next_Angular_State.R = w_t(3);
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

