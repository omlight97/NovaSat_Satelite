function X = Body2ToD(X, phi, theta, psi, r, v)
%IMPORTANT, make sure that the r is the position in [km] and v is the
%velocity in [km/h] and  make sure that the angle are in [rad]
X=make_it_col(X); %make the vector a column vector
r=make_it_col(r); %make the vector a column vector
v=make_it_col(v); %make the vector a column vector
X=Body2LVLH(X, phi, theta, psi); %tranform from Body to LVLH
X=LVLH2ToD(X, r, v); %tranform from LVLH to ToD
end