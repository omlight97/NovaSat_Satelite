function [F]=View_Factor(Face_normal)
%Make sure that the face normal is in the satellite frame of reference
%(Body)

psi   = Current_Step_Angular.angles(1);
theta = Current_Step_Angular.angles(2);
phi   = Current_Step_Angular.angles(3);


X_s_lvlh=Body2LVLH(Face_normal, phi, theta, psi); %tranform the Face_normal to LVLH frame
radial_dir_LVLH=[1 0 0]; %vector perpenpendicular to the surface of the moon (X direction LVLH)
beta=acos((radial_dir_LVLH*X_s_lvlh)/norm(X_s_lvlh)); %Beta is calculated as the dot product of the radial direction and the face_normal in LVLH
h=(Params.R+Params.Altitude0)/Params.R; %h is defined a the distance of from the center of the moon to the face of satelite over the radius of the moon

%rule for view factor taken from this doc page 10 http://webserver.dmt.upm.es/~isidoro/tc3/Radiation%20View%20factors.pdf
    if abs(beta)>arcos(1/h)
        x=sqrt(h^2-1);
        y=-x*cot(beta);
        F=(pi*h^2)^-1*(cos(beta)*acos(y)-x*sin(beta*sqrt(1-y^2)))+(1/pi)*atan2(sin(beta*sqrt(1-y^2))/x);
    else
        F=cos(beta)/h^2;
    end
end