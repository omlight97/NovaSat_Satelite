function [vec] = make_it_col(vec)

[m,~] = size(vec);

if (m==1) % meaning, if vec is a row rector
vec = vec'; 
end

end