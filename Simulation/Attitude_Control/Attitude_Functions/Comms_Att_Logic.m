function [eul_t,w_t] = Comms_Att_Logic(Sat2Comms_LVLH)
    %% Communication State Logic
    Z_LVLH = [0;0;1]; %Z axis in LVLH frame
    uSat2Comms_Body = Sat2Comms_LVLH./norm(Sat2Comms_LVLH); 
    eul_t = ang_between_vec(Z_LVLH,uSat2Comms_Body);

    % Target angular rate [rad/sec]
    p_t = 0;
    q_t = 0;
    r_t = 0;
    w_t = [p_t;q_t;r_t];
end
