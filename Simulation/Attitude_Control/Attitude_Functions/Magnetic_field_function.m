function Magnetic_Torque_B = Magnetic_field_function(Magnetic_Field_I,tetpsiphi_vec,Magnetic_Dipol_B)

    tet = tetpsiphi_vec(1);
    psi = tetpsiphi_vec(2);
    phi = tetpsiphi_vec(3);
    I_to_B = tetpsiphi(tet,psi,phi);

    Magnetic_Field_B = I_to_B*Magnetic_Field_I;


    Magnetic_Torque_B = cross(Magnetic_Dipol_B,Magnetic_Field_B);
end



function I_to_B = tetpsiphi(tet,psi,phi)
    I_to_B = rotx(phi)*rotz(psi)*roty(tet);
end

function mat = rotx(angle)
    mat = [1, 0         , 0;
           0, cos(angle), -sin(angle);
           0, sin(angle), cos(angle)];
end
function mat = roty(angle)
    mat = [cos(angle) , 0, sin(angle);
           0          , 1, 0;
           -sin(angle), 0, cos(angle)];
end
function mat = rotz(angle)
    mat = [cos(angle), -sin(angle), 0;
           sin(angle), cos(angle) , 0;
           0         , 0          , 1];
end
