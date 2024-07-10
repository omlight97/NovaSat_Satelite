
function [Next_step_Orbit, Next_elements] = Orbit_Prop_circular(current_step_Orbit, Numeric_properties, Params)
%% old prop
% Orbit Design

elements = StateVec2OrbitalElements(current_step_Orbit,Params);
theta    = elements.theta;
Next_elements = elements;

dt  = Numeric_properties.dt;

n        = Params.n;
theta    = theta + n*dt; %True for circular orbit only!!

[Next_step_Orbit] = OrbitalElements2StateVec(elements, theta, Params);

end
