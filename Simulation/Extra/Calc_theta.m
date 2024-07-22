function Theta_angle = Calc_theta(SunPosition, Current_Step_Angular ,Current_Step_Orbit)
%--------------------------------------------------------
% PURPOSE: This function calculate the angle theta which 
%          is the angle between the satelitte's location 
%          and the sun position vector. 
%          Theta determines how much solar power the 
%          solar panels produce.
%          For theta = 0 the power production is at its 
%          maximum and the solar panels are right in 
%          front of the sun and for theta = 90, there isn't 
%          power production
%--------------------------------------------------------
% INPUT: WorldModel for the sun position, Current_Step_Step 
%        for the satelitte's position and Current_Step_Angular 
%        for the coordinates transformation 
%--------------------------------------------------------
% OUTPUT: theta angle in rad
%--------------------------------------------------------

%--------keep in mind!------------------
%   units and coordinates systems!
%---------------------------------------

psi   = Current_Step_Angular.Psi;
theta = Current_Step_Angular.Theta;
phi   = Current_Step_Angular.Phi;

r = (Current_Step_Orbit); %ECI
r_sun = SunPosition; %ECI

sat2sun = r_sun-r;  %ECI
sat2sun_Body = ECI2Body(sat2sun,phi,psi,theta); %Body

Theta_angle = atan2(sat2sun_Body(2),sat2sun_Body(1)); %[rad]
Theta_angle=0;
end