function X=ToD2LVLH(X,r,v)
%IMPORTANT, make sure that the r is the position in [km] and v is the velocity in [km/h]
X=make_it_col(X); %make the vector a column vector
r=make_it_col(r); %make the vector a column vector
v=make_it_col(v); %make the vector a column vector
Rotation_matrix=construct_LVLH2ToD_matrix(r,v); %obtain the tranformation matrix
X=transpose(Rotation_matrix)*X;
end