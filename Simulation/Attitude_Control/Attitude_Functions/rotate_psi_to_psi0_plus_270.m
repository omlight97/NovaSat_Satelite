function[psi_next done_flag] = rotate_psi_to_psi0_plus_270(psi_0)

% TODO : ASK HILEL FOR GUIDANCE
psi_next = psi + 1;
psi_target = psi_0 + 270;


if psi_target <= psi_next
    done_flag = 1;
else
    done_flag = 0;
end




end