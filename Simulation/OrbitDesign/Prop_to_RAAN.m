function [Next_step_Orbit] = Prop_to_RAAN(r0_and_v0, Numeric_properties, Params)

current_step_Orbit = r0_and_v0;
Next_step_Orbit = current_step_Orbit;

while ~(abs(current_step_Orbit.r(3)) <= Numeric_properties.Tol_km && abs(current_step_Orbit.v(3))>=0)
    [Next_step_Orbit] = Orbit_Prop(current_step_Orbit, Numeric_properties, Params);
    current_step_Orbit = Next_step_Orbit;
end

if (abs(current_step_Orbit.r(3)) <= Numeric_properties.Tol_km && abs(current_step_Orbit.v(3))>=0)
    disp('NOTE: Scenario starts at RAAN')
end

end


