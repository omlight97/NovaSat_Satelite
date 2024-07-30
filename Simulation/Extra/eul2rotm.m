function R = eul2rotm(eul, sequence)
    % This function assumes ZYX sequence
    alpha = eul(1); % yaw
    beta = eul(2);  % pitch
    gamma = eul(3); % roll
    
    Rz = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
    Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    Rx = [1 0 0; 0 cos(gamma) -sin(gamma); 0 sin(gamma) cos(gamma)];
    
    R = Rz * Ry * Rx;
end