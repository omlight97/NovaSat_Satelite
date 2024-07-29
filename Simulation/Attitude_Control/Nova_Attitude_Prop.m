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
OverrideSimulink = false;
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
    Outsim = RunSimulink(q_eul_i,w_i,q_eul_t);

    % Export data from simulation
    Data = Outsim.Data.signals.values(:,:,:);
    Data = reshape(Data,[26 length(Data)]);
    q_f = Data(1:4,end); % final quaternion
    w_f = Data(5:7,end); % [rad/sec] final angular velocity
    q_error = Data(8:11,end); % error quaternion
    w_error = Data(12:14,end);% [rad/sec] angular velocity error
    Tc = Data(15:17,end); % [Nm] Torque command
    TW = Data(19:22,end); % Torque on wheels
    HW = Data(23:26,end); % Angular momentum on wheels
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
    
    Params.Attitude_Control_Data.TW1 = TW(1);
    Params.Attitude_Control_Data.TW2 = TW(2);
    Params.Attitude_Control_Data.TW3 = TW(3);
    Params.Attitude_Control_Data.TW4 = TW(4);
    Params.Attitude_Control_Data.HW1 = HW(1);
    Params.Attitude_Control_Data.HW2 = HW(2);
    Params.Attitude_Control_Data.HW3 = HW(3);
    Params.Attitude_Control_Data.HW4 = HW(4);
end
end





