function[eul_t state_three_flag] = SunSearch_third_Manuver(eul0,eul_i)
    %rotate_theta_-90_psi_to_psi0_plus_270
    psi_i = eul_i(1);
    tet_i = eul_i(2);
    phi_i = eul_i(3);

    psi_0 = eul_0(1);
    tet_0 = eul_0(2);
    phi_0 = eul_0(3);
    
    psi_t = psi_i;
    tet_t = tet_i;
    phi_t = phi_i;

    theta_target = tet_0 - 90;
    if theta_target <= tet_t
        tet_t = tet_i - 1;
    end

    psi_target = psi_0 + 270;
    if psi_target >= psi_t
        psi_t = psi_i + 1;
    end

    if psi_target <= psi_t && theta_target >= tet_t
        state_three_flag = 1;
    else
        state_three_flag = 0;
    end
    
    eul_t = [psi_t,tet_t,phi_t];

end
