function vector = Norm_Each_Row(Matrix)

% ------------------------------------------------------------------------- %
% This function calculate the norm of each row of the matrix (this matrix
% is the input). The output is the vector of the norms.
% Example:
% If the input is
%                        Matrix = [1 2 3; 2 2 2];
% Then the output is:
%                         vector = [3.7417; 3.4641];
% ------------------------------------------------------------------------- %
% (C) May Alon, The JERICCO project
% For any questions or comments: may.alon@campus.technion.ac.il
% Last update: 01.04.22
% ------------------------------------------------------------------------------ %

vector = (sum((Matrix(:,1:3)).^2,2)).^0.5;

end