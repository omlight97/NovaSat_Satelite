
function Alpha_angle = Calc_Alpha(World_Model, Current_Step_Angular, Current_Step_Orbit)

%IMPORTANT, make sure that the angle are in [rad] and the position of the
%sun and satellite are in ToD frame
psi   = Current_Step_Angular.angles(1);
theta = Current_Step_Angular.angles(2);
phi   = Current_Step_Angular.angles(3);
r = Current_Step_Orbit.r;
v = Current_Step_Orbit.v;

r_sun = World_Model.SunPosition; %in TOD
x_axis_sat_inBody = Body2LVLH([1,0,0], phi, theta, psi); % in Body
x_axis_sat_inTOD  = LVLH2ToD(x_axis_sat_inBody,r,v);

r_sat2sun  = make_it_row(r_sun) - make_it_row(r); %make tranlation -> relative position sat to sun
x_axis = make_it_col(x_axis_sat_inTOD); %To make it short

Alpha_angle = acos((r_sat2sun*x_axis)/(norm(r_sat2sun)*norm(x_axis)));


end