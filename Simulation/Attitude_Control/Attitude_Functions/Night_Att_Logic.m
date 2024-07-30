function [eul_t,w_t] = Night_Att_Logic(eul_i,w_max)
    % Target angular rate [rad/sec]
    % w_max=1;
    % w_safty_factor=1;
    % w_max_check = w_i > w_max*w_safty_factor;
    % if any(w_max_check)
    %     w_i = w_max(w_max_check);
    % end

    % Target attitude in terms of euler angles [rad]
    eul_t = eul_i;
    
    % Target angular rate [rad/sec]
    p_t = 2;
    q_t = 2;
    r_t = 2;
    w_t = [p_t;q_t;r_t];
end