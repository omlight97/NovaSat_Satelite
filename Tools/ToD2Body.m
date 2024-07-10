function X = ToD2Body(X, phi, theta, psi, r, v)
%IMPORTANT, make sure that the r is the position in [km] and v is the
%velocity in [km/h] and  make sure that the angle are in [rad]
X=make_it_col(X); %make the vector a column vector
r=make_it_col(r); %make the vector a column vector
v=make_it_col(v); %make the vector a column vector
X=ToD2LVLH(X,r,v); %tranform from ToD to LVLH
X=LVLH2Body(X, phi, theta, psi); %tranform from Body to ToD
end