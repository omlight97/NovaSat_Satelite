function Rotation_matrix=construct_LVLH2ToD_matrix(r,v)
O_3I=-r/norm(r);
O_2I=-(cross(r,v))/norm(cross(r,v));
O_1I=cross(O_2I,O_3I);
Rotation_matrix=[O_1I O_2I O_3I];
end