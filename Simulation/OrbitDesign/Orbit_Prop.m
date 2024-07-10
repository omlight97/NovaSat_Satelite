function [Next_step_Orbit,t_orbit] = Orbit_Prop(current_step_Orbit, Numeric_properties, Params, t_now)

% function [Next_step_Orbit, Next_elements] = Orbit_Prop(current_step_Orbit, Numeric_properties, Params)
%% old prop
% Orbit Design

% elements = StateVec2OrbitalElements(current_step_Orbit,Params);
% theta    = elements.theta;
% Next_elements = elements;
% 
% dt  = Numeric_properties.dt;
% 
% n        = Params.n;
% theta    = theta + n*dt; %True for circular orbit only!!
% 
% [Next_step_Orbit] = OrbitalElements2StateVec(elements, theta, Params);
% t_orbit = t_now+dt;
%% new prop

dt  = Numeric_properties.dt;

r0 = current_step_Orbit.r;
v0 = current_step_Orbit.v;

opts = Numeric_properties.opts;
tspan = [t_now, (t_now+dt)];
[t,y] = ode45(@(t,y) SolverOrbit(t,y, Params), tspan, [r0',v0'], opts);
Next_step_Orbit.r = y(end,1:3);
Next_step_Orbit.v = y(end,4:6);
t_orbit = t(end);


end
