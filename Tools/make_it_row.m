function [vec] = make_it_row(vec)

[m,~] = size(vec);

if ~(m==1) % meaning, if vec is not a row rector
vec = vec'; 
end

end