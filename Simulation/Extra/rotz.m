function mat = rotz(angle)
    mat = [cos(angle), -sin(angle), 0;
           sin(angle), cos(angle) , 0;
           0         , 0          , 1];
end