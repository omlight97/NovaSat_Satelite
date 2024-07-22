function [atm_dense] = stk_atm_density(path)

x = readcell(path);
atm_dense = zeros(1,length(x));
for idx = 1:length(x)
    % try
        cellarr = (x{idx,end});
        density = (cellarr);
        atm_dense(idx) = density;
    % end
end
atm_dense = atm_dense*(1/1000)^3;









% 'C:\Users\gilim\Documents\studies\MATLAB\Semter8\NovaSat\sim_new_final\NovaSat_Satelite\Simulation\Extra\NOVASAT-16U_AtmosphericDensity.txt'