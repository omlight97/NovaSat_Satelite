function X=LVLH2Body(X, phi, theta, psi)
%IMPORTANT, make sure that the angle are in [rad]
X=make_it_col(X);  %make the vector a column vector
Rotation_matrix=construct_Body2LVLH_matrix(phi, theta, psi);  %obtain the tranformation matrix
X=transpose(Rotation_matrix)*X; %inverse operation of ToD2Body inverse(rotation_matrix)*X
end