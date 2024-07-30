function [eul_t,w_t] = SunSearch_Att_Logic(Sun2Sun_LVLH,Flags)
    %% Sun Search State Logic
    Z_LVLH = [0;0;1]; %Z axis in LVLH frame
    SunSensorFOV = deg2rad(120); %Sun sensor field of view
    SunSensorPositionVector_B = [0;1;0]; %Sun sensor Heading - assuming pointing side at the moment

    uSat2Sun_B = Sun2Sun_LVLH./norm(Sun2Sun_LVLH); % Satellite center to the center of the Sun (unit vector). - body frame
    isLOS2Sun = dot(SunSensorPositionVector_B, uSat2Sun_B) > cos(SunSensorFOV);

    isLOS2Sun = true; %% temp - until logic is completed and verifyied

    if(isLOS2Sun) % is sun in field of view
        eul_t = ang_between_vec(Z_LVLH,uSat2Sun_B);
        % psi_t = asin(SunSensorPositionVector_B(3)/uSat2Sun_B(2));
        % tet_t = atan(uSat2Sun_B(2)/uSat2Sun_B(1)); 
        % phi_t = 0;
        % eul_t = [psi_t,tet_t,phi_t];
    else % Roatation sequence logic
        %0 - Initial sunsearch logic algorithm
        if Flags.sun_search.initial_flag
            Params.eul0 =  [psi_i,tet_i,phi_i];
            Flags.sun_search.initial_flag = 0;
            Flags.state_one_flag = 1;
            Flags.state_two_flag = 0;
            Flags.state_three_flag = 0;
        %1 - rotate 270 around main axis
        elseif Flags.state_one_flag
            [eul_t,Flags.state_one_flag,Flags.state_two_flag] = SunSearch_first_Manuver(Params.eul0,eul_i); % pass imu readings untill psi = psi0+270
            %if finished first manuver, reset the initial conditions for the second manuver
            if Flags.sun_search.state_one_flag == 0 && Flags.sun_search.state_two_flag == 1
                Params.eul0 =  [psi_i,tet_i,phi_i];
            end
        %2 - rotate 45 pitch and then 270 arund main axis
        elseif Flags.state_two_flag
            [eul_t,Flags.state_two_flag,Flags.state_three_flag] = SunSearch_second_Manuver(Params.eul0,eul_i); % pass imu readings untill psi = psi0+360
            %if finished second manuver, reset the initial conditions for the third manuver
            if Flags.sun_search.state_two_flag == 0 && Flags.sun_search.state_three_flag == 1
                Params.eul0 =  [psi_i,tet_i,phi_i];
            end
        %3 - rotate -90 pitch and then 270 arund main axis
        elseif Flags.state_three_flag
            [eul_t,Flags.state_three_flag] = SunSearch_third_Manuver(Params.eul0,eul_i); % pass imu readings untill psi = psi0+360
        %4 - Error
        else
            Flags.sun_search.initial_flag = 0;
            msgbox('Was not able to find sun after 3 rotation sequences.')
        end
    end

    % % Target attitude in terms of euler angles [rad]
    % psi_t = 1;
    % tet_t = 1;
    % phi_t = 1;
    % eul_t = [psi_t,tet_t,phi_t];

    % Target angular rate [rad/sec]
    p_t = 0;
    q_t = 0;
    r_t = 0;
    w_t = [p_t;q_t;r_t];
end

