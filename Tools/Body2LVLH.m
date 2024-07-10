function X=Body2LVLH(X, phi, theta, psi)
%IMPORTANT, make sure that the angle are in [rad]
X=make_it_col(X); %make the vector a column vector
Rotation_matrix=construct_Body2LVLH_matrix(phi, theta, psi); %obtain the tranformation matrix
X=Rotation_matrix*X;
end