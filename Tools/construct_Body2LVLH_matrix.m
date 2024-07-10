function Rotation_matrix=construct_Body2LVLH_matrix(phi, theta, psi)
Rotation_matrix=[cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
                 sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi); 
                 -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
end