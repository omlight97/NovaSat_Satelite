function [uSat2Sun_B,isLOS2Sun,eul2target] = SunSensorModel(Earth2Sun_I,Sat_Pos_I,I_to_B,UTCTime)
    SunSensorFOV = deg2rad(120); %Sun sensor field of view
    SunSensorPositionVector = [0;1;0]; %Sun sensor Heading - assuming pointing side at the moment
    % Earth2Sun_I = approxECISunPosition(UTCTime); % Earth's center to the center of the Sun. - Online function
    % Earth2Sun_I = Earth2Sun_I';
    
    Sat2Sun_I = Sat_Pos_I - Earth2Sun_I;
    Sat2Sun_B = I_to_B*Sat2Sun_I;

    uSat2Sun_B = Sat2Sun_B./norm(Sat2Sun_B); % Satellite center to the center of the Sun (unit vector). - body frame

    isLOS2Sun = dot(SunSensorPositionVector, uSat2Sun_B) > cos(SunSensorFOV);


    if(isLOS2Sun)
        tet_target = atan(Sat2Sun_B(2)/Sat2Sun_B(1)); %??
        psi_target = asin(SunSensorPositionVector(3)/uSat2Sun_B(2)); %??
        phi_target = 0;
        eul2target = [tet_target;psi_target;phi_target];
    else
        tet_target = atan(Sat2Sun_B(2)/Sat2Sun_B(1)); %??
        psi_target = asin(SunSensorPositionVector(3)/uSat2Sun_B(2)); %??
        phi_target = 0;
        eul2target = [tet_target;psi_target;phi_target];
    end
end