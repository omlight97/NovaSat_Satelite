clc; 
clear variables;

[Params, Flags] = JeriParams();
[Next_Angular_State,Wheels_Data,More_Data_Attitude_Control,Flags] = Jeri_Attitude_Prop(States,Current_Step_Angular,Flags,Params);