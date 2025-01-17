function[eul_t,state_one_flag,state_two_flag] = SunSearch_First_Manuver(eul0,eul_i)
    %rotate_psi_to_psi0_plus_270
    psi_i = eul_i(1);
    tet_i = eul_i(2);
    phi_i = eul_i(3);

    psi_0 = eul_0(1);
    tet_0 = eul_0(2);
    phi_0 = eul_0(3);
    
    psi_t = psi_i + 1;
    tet_t = tet_i;
    phi_t = phi_i;

    psi_target = psi_0 + 270;
    if psi_target <= psi_t
        state_one_flag = 0;
        state_two_flag = 1;
    end
    
    eul_t = [psi_t,tet_t,phi_t];
end