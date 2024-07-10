function [state]=OrbitalElements2StateVec(elements, theta, Params)

%------------------------------ WHAT DOES THIS FUNCTION DO? ------------------------------%

% This function gets the orbital elements:
% 1. eccentricity (ecc)
% 2. argument of perigee (omega)
% 3. right ascension of the the ascending node (RAAN)
% 4. semi major axis (a)
% 5. inclination (inc)
% 6. specific angular momentum (h)
% 7. true anomaly (theta)

% and returns the position vector (r_vec) and the velocity vector (v_vec) in ECI.


%------------------------------ EXAMPLE ------------------------------%

% >> Enter the following orbital elements:
% const.mu=398600; %km^3/s^2
% elements.omega=deg2rad(62.24);
% elements.ecc=0.1589;
% elements.inc=deg2rad(102.3);
% elements.RAAN=deg2rad(11.396);
% elements.a=8059;
% elements.h=55957;
% elements.theta=deg2rad(20);

% >> you suppose to get:
% r_vec = 1.0e+03 * [1.1898, -1.2319, 6.6169]; %[km]
% v_vec = [ -7.8381, -1.9035, 1.4548]; %[km/s]


%------------------------------ WARNINGS ------------------------------%
% All angles should be in [rad]!

%----------------------------------------------------------------------%


mu    = Params.mu;
ecc   = elements.ecc;
omega = elements.omega;
inc   = elements.inc;
RAAN  = elements.RAAN;
a     = elements.a;

 p    = a*(1-ecc^2);
 h    = sqrt(p*mu);

r         = a*(1-ecc^2)/(1+ecc*cos(theta));
r_vec = peri2cart([r*cos(theta),r*sin(theta),0],inc,omega,RAAN);


v_vec     = (mu/h)*peri2cart([-sin(theta) ,ecc+cos(theta) ,0],inc,omega,RAAN);

state.r = r_vec;
state.v = v_vec;

end

