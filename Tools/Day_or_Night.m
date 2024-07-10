function Day_Flag = Day_or_Night(Params, r_vec, r_sun)

R = Params.R;
r_sun = make_it_row(r_sun);
r_vec = make_it_col(r_vec);

delta1 = acos(r_sun*r_vec/norm(r_sun*r_vec));

if delta1 >= 90*pi/180
    delta2 = pi - delta1;
    x = norm(r_vec)*sin(delta2);
    if x <= R
        Day_Flag = 0; % Dark
    else
        Day_Flag = 1; % Day Light
    end
    
else
    Day_Flag = 1; % Day Light
    
end


end
