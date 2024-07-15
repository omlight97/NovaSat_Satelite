function vec_body = ECI2Body(vec_eci, phi,psi,theta)
%--------------------------------------------------------
% PURPOSE: This function performs the transition from 
%          ECI coordinates to body axis system
%--------------------------------------------------------
% INPUT: vector in ECI coordinates and body angles - phi,psi,theta
%--------------------------------------------------------
% OUTPUT: vector in body coordinates
%--------------------------------------------------------
    I2B = rotx(phi)*rotz(psi)*roty(theta);
    vec_body = I2B*vec_eci;
end