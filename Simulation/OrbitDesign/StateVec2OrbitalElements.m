function elements = StateVec2OrbitalElements(state, Params)


%------------------------------ WHAT DOES THIS FUNCTION DO? ------------------------------%

% This function gets the position vector (r_vec) and the velocity vector (v_vec) in ECI,
% and returns the orbital elements:

% 1. eccentricity (ecc)
% 2. argument of perigee (omega)
% 3. right ascension of the the ascending node (RAAN)
% 4. semi major axis (a)
% 5. inclination (inc)
% 6. specific angular momentum (h)
% 7. true anomaly (theta)

% in one structure called "elements"


%------------------------------ EXAMPLE ------------------------------%

% >> Enter the following vectors + physical constant structure:
% const.mu=398600; %km^3/s^2
% r_vec = 1.0e+03 * [1.1898, -1.2319, 6.6169]; %[km]
% v_vec = [ -7.8381, -1.9035, 1.4548]; %[km/s]

% >> you suppose to get one structure with all the orbital elements:
% elements.omega=deg2rad(62.24);
% elements.ecc=0.1589;
% elements.inc=deg2rad(102.3);
% elements.RAAN=deg2rad(11.396);
% elements.a=8059;
% elements.h=55957;
% elements.theta=deg2rad(20);


%------------------------------ WARNINGS ------------------------------%
% All angles should be in [rad]!

mu = Params.mu;

r_vec = state.r;
v_vec = state.v;

r=norm(r_vec);
v=norm(v_vec);

h_vec=cross(r_vec,v_vec);
h=norm(h_vec);

inc=acos(h_vec(3)/h);
if ~(inc>=0 && inc<pi)
    inc=2*pi-inc;
end

NodeLine_vec=cross([0,0,1],h_vec);
NodeLine=norm(NodeLine_vec);
RAAN=acos(NodeLine_vec(1)/NodeLine);
if NodeLine_vec(2)<0
    RAAN=2*pi-RAAN;
end

ecc_vec=(1/mu)*(cross(v_vec,h_vec)-(mu/r)*r_vec);
ecc=norm(ecc_vec);

ecc_vec = make_it_col(ecc_vec);


omega=acos((NodeLine_vec*ecc_vec)/(NodeLine*ecc));
if ecc_vec(3)<0
    omega=2*pi-omega;
end

r_vec = make_it_col(r_vec);
v_vec = make_it_col(v_vec);

v_r=r_vec'*v_vec/r;

theta=acos(ecc_vec'*r_vec/(ecc*r));
if v_r<0
    theta=2*pi-theta;
end
a=r*(1+ecc*cos(theta))/(1-ecc^2);

elements.omega  = omega;
elements.ecc    = ecc;
elements.inc    = inc;
elements.RAAN   = RAAN;
elements.theta  = theta;
elements.a      = a;
end







