function [Next_Angular_State,Wheels_Data,More_Data_Attitude_Control,Flags] = Jeri_Attitude_Prop(States,Current_Step_Angular,Flags,Params)
%This function calculates the angular state of the satellite each step including angles, rates, 
% performances of reaction wheels and errors.
%   Detailed explanation goes here

Psi   = Current_Step_Angular.angles(1);
Theta = Current_Step_Angular.angles(2);
Phi   = Current_Step_Angular.angles(3);

P = Current_Step_Angular.rate(1);
Q = Current_Step_Angular.rate(2);
R = Current_Step_Angular.rate(3);

% Initial angular velcoity [rad/sec]
w_i = [P;Q;R];
% Initial attitude in terms of euler angles [rad]
eul_i = [Psi,Theta,Phi];
% Convert to quaternion
quat = eul2quat(eul_i);
q_i = flip(quat);

% Need to specify qc and wc - desired attitude and angular velocity

% Simulate
Outsim = sim('ADCS_sim');
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

Next_Angular_State.P = w_t(1);
Next_Angular_State.Q = w_t(2);
Next_Angular_State.R = w_t(3);

Next_Angular_State.Psi = eul_f(1);
Next_Angular_State.Theta = eul_f(2);
Next_Angular_State.Phi = eul_f(3);

More_Data_Attitude_Control.Psi_error = eul_error(1);
More_Data_Attitude_Control.Theta_error = eul_error(2);
More_Data_Attitude_Control.Phi_error = eul_error(3);

More_Data_Attitude_Control.P_error = w_error(1);
More_Data_Attitude_Control.Q_error = w_error(2);
More_Data_Attitude_Control.R_error = w_error(3);

More_Data_Attitude_Control.Lc = Tc(1);
More_Data_Attitude_Control.Mc = Tc(2);
More_Data_Attitude_Control.Nc = Tc(3);
end

