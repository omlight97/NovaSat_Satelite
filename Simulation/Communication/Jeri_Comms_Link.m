function Link_Analysis = Jeri_Comms_Link(dt, Link_Budget, Comms_Mode)
    % Parameters:
    % % double dt - Time Step [s], unused for now
    % % struct Link_Budget - Structure containing the necessary link budget info
    % % string Comms_Mode - Value indicating whether we are conducting
    % % % uplink ('RX') or downlink ('TX'), should be moved into the Flags
    % % % variable later
    % Returns:
    % % struct Link_Analysis - Structure containing the link analysis
    % % % parameters

    % Notes:
    % % The parameters are defined in their own data structure
    % % (Link_Budget) for now, they will need to be changed to be sub-fields
    % % of the Operational Modes.

    S = Link_Budget.Propagation_Path_Length; %[km]
    L_a = Link_Budget.Polarization_Loss; %[dB]
    L_I = Link_Budget.Implementation_Loss; %[dB]
    Req_Eb_over_N0 = Link_Budget.Required_Eb_over_N0; %[dB]
    BER = Link_Budget.Bit_Error_Rate; % Unused for now

    if Comms_Mode == "TX"        
        f = Link_Budget.TX.Frequency; %[GHz]
        R = Link_Budget.TX.Data_Rate; %[bps]
        
        %Transmitter (JERICCO) Specs
        P = Link_Budget.TX.Transmit_Power; %[W]
        L_l = Link_Budget.TX.Transmit_Line_Loss; %[dB]
        Theta_t = Link_Budget.TX.Transmit_Antenna_Beamwidth; %[deg]
        G_pt = Link_Budget.TX.Peak_Transmit_Antenna_Gain; %[dBi]
        D_t = Link_Budget.TX.Transmit_Antenna_Diameter; %[m]
        e_t = Link_Budget.TX.Transmit_Antenna_Pointing_Offset; %[deg]
        L_pt = Link_Budget.TX.Transmit_Antenna_Pointing_Loss; %[dB]

        %Receiver (DSN) Specs
        D_r = Link_Budget.TX.Receive_Antenna_Diameter; %[m]
        G_rp = Link_Budget.TX.Peak_Receive_Antenna_Gain; %[dBi]
        Theta_r = Link_Budget.TX.Receive_Antenna_Beamwidth; %[deg]
        e_r = Link_Budget.TX.Receive_Antenna_Pointing_Error; %[deg]
        L_pr = Link_Budget.TX.Receive_Antenna_Pointing_Loss; %[dB]
        G_over_T = Link_Budget.TX.Receive_Antenna_Gain_to_Noise_Temp_Ratio; %[dB/K]

    elseif Comms_Mode == "RX"
        f = Link_Budget.RX.Frequency; %[GHz]
        R = Link_Budget.RX.Data_Rate; %[bps]
        
        %Transmitter (DSN) Specs
        P = Link_Budget.RX.Transmit_Power; %[W]
        L_l = Link_Budget.RX.Transmit_Line_Loss; %[dB]
        Theta_t = Link_Budget.RX.Transmit_Antenna_Beamwidth; %[deg]
        G_pt = Link_Budget.RX.Peak_Transmit_Antenna_Gain; %[dBi]
        D_t = Link_Budget.RX.Transmit_Antenna_Diameter; %[m]
        e_t = Link_Budget.RX.Transmit_Antenna_Pointing_Offset; %[deg]
        L_pt = Link_Budget.RX.Transmit_Antenna_Pointing_Loss; %[dB]

        %Receiver (JERICCO) Specs
        D_r = Link_Budget.RX.Receive_Antenna_Diameter; %[m]
        G_rp = Link_Budget.RX.Peak_Receive_Antenna_Gain; %[dBi]
        Theta_r = Link_Budget.RX.Receive_Antenna_Beamwidth; %[deg]
        e_r = Link_Budget.RX.Receive_Antenna_Pointing_Error; %[deg]
        L_pr = Link_Budget.RX.Receive_Antenna_Pointing_Loss; %[dB]
        G_over_T = Link_Budget.RX.Receive_Antenna_Gain_to_Noise_Temp_Ratio; %[dB/K]
    else
        disp("Error! Invalid Communications Mode");
    end
    
    P_dB = 10*log10(P); %[dBW], Log Transmitter Power
    G_t = G_pt + L_pt; %[dBi], Net Transmit Antenna Gain
    G_r = G_rp + L_pr; %[dBi], Net Receive Antenna Gain
    EIRP = P_dB + G_t + L_l; %[dBW], Equivalent Isotropic Radiated Power
    L_s = 147.55 - 20*log10(S*1000)-20*log10(f*10^9); %[dB], Space Loss
    T_s = G_r / G_over_T; %[K], System Noise Temperature
    Eb_over_N0 = EIRP + L_pr + L_s + L_a + G_r + 228.6 - 10*log10(T_s) - 10*log10(R); %[dimensionless], Energy/Bit to Noise Density Ratio
    C_over_N0 = Eb_over_N0 + 10*log10(R); %[dB-Hz], Carrier-to-Noise Density Ratio
    Link_Margin = Eb_over_N0 - (Req_Eb_over_N0 + L_I); %[dB], Available Link Margin

    Link_Analysis.EIRP = EIRP;
    Link_Analysis.Eb_over_N0 = Eb_over_N0;
    Link_Analysis.C_over_N0 = C_over_N0;
    Link_Analysis.Margin = Link_Margin;
end