function [Next_Angular_State,Params,Flags] = Nova_Attitude_Prop_new(Current_Step_Angular,Flags,Params,PositionCommTime2)
% clc;
% close all;
% clear all;
if Flags.IsDay
a = Params.SatPosition';
b = Params.SunPosition';
end
if Flags.Communication
a = Params.SatPosition';
b = PositionCommTime2';
else
    a=[0;0;0];
    b=[0;0;0];
end
% Method described in
% https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
a = a./norm(a);
b = b./norm(b);
v = cross(a,b);
vx = [0 -v(3) v(2) ; v(3) 0 -v(1); -v(2) v(1) 0 ];
c = dot(a,b);
I = eye(3);
R=I+vx+vx^2*(1/(1+c));
R = round(R,5);
% From Rot matrix to euler coordinates, follows XYZ, described in:
% http://www.gregslabaugh.net/publications/euler.pdf
if (R(3,1) ~=1) && (R(3,1) ~=-1)
    theta_1 = -asin(R(3,1));
    theta_2 = pi-theta_1;
    psi_1 = atan2((R(3,2)/cos(theta_1)),(R(3,3)/cos(theta_1)));
    psi_2 = atan2((R(3,2)/cos(theta_2)),(R(3,3)/cos(theta_2)));
    phi_1 = atan2((R(2,1)/cos(theta_1)),(R(1,1)/cos(theta_1)));
    phi_2 = atan2((R(2,1)/cos(theta_2)),(R(1,1)/cos(theta_2)));
    theta = min(theta_1,theta_2);
    psi = min(psi_1,psi_2);
    phi = min(phi_1,phi_2);
else
    phi = 0;
    if R(3,1) == -1
        theta = pi/2;
        psi = phi+atan2(R(1,2),R(1,3));
    else
        theta = -pi/2;
        psi = -phi+atan2(-R(1,2),-R(1,3));
    end
end
theta = (theta);
psi = (psi);
phi = (phi);
if (a(1)==0 && a(2)==0 && a(3)==0)
theta = ( Current_Step_Angular.Theta);
psi = ( Current_Step_Angular.Psi);
phi = ( Current_Step_Angular.Phi);
end
% if isnan(theta)
%     theta=0;
% end
% if isnan(psi)
%     psi=0;
% end
% if isnan(phi)
%     phi=0;
% end
 Next_Angular_State.Psi = psi;
    Next_Angular_State.Theta = theta;
    Next_Angular_State.Phi = phi;
end
