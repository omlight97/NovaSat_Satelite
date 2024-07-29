function [Month_vec] = Name2Month(Name_vec)
Name_vec = string(Name_vec);
Month_vec = zeros(size(Name_vec));
for k=1:length(Name_vec)
    if Name_vec(k) == 'Jan'
        Month_vec(k) =01;
    elseif Name_vec(k) == 'Feb'
        Month_vec(k) =02;
    elseif Name_vec(k) == 'Mar'
        Month_vec(k) =03;
    elseif Name_vec(k) == 'Apr'
        Month_vec(k) =04;
    elseif Name_vec(k) == 'May'
        Month_vec(k) =05;
    elseif Name_vec(k) == 'Jun'
        Month_vec(k) =06;
    elseif Name_vec(k) == 'Jul'
        Month_vec(k) =07;
    elseif Name_vec(k) == 'Aug'
        Month_vec(k) =08;
    elseif Name_vec(k) == 'Sep'
        Month_vec(k) =09;
    elseif Name_vec(k) == 'Oct'
        Month_vec(k) =10;
    elseif Name_vec(k) == 'Nov'
        Month_vec(k) =11;
    elseif Name_vec(k) == 'Dec'
        Month_vec(k) =12;
    end
end
end