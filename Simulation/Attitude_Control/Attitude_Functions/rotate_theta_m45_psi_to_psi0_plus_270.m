function[] = rotate_theta_45_psi_to_psi0_plus_270(psi_0,psi,theta,theta_0)

% TODO : ASK HILEL FOR GUIDANCE
psi_next = psi + 1;
psi_target = psi_0 + 270;

theta_next = psi + 1;
theta_target = theta_0 - 45;


if psi_target <= psi_next && theta_target >= theta_next
    done_flag = 1;
else
    done_flag = 0;
end





end