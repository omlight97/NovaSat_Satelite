function dy = SolverOrbit(t,y, Params)

mu = Params.mu;
R  = Params.R;
J2 = 0;

r  = sqrt(y(1)^2 + y(2)^2 + y(3)^2);

dy = [y(4:6);-(mu.*y(1:3)./r^3).*(1-1.5*J2.*(R/r)^2.*(5.*(y(3)./r)^2-[1;1;3]))];
end