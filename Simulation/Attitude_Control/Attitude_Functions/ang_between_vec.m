function eul = ang_between_vec(a,b)
    na   = a / norm(a);
    nb   = b / norm(b);
    v    = cross(na, nb);
    skew = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
    R    = eye(3) + skew + skew ^ 2 * (1 - dot(na, nb)) / (norm(v))^2; 
    
    eul = rotm2eul( R );
    %psi tet phi
end