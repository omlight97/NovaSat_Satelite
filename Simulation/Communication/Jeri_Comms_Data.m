function Data_Storage = Jeri_Comms_Data(dt, Data_Storage)
    % Parameters:
    % % double dt - Time Step [s]
    % % struct Data_Storage - Structure containing the necessary data info

    % Data Movement Order: Generation, Transmission, Reception

    % These parameters are defined in their own data structure
    % (Data_Storage) for now, they will need to be changed to be sub-fields
    % of the Operational Modes.
    Total_Storage = Data_Storage.Total_Storage; %[Gb]
    Ava_Storage = Data_Storage.Ava_Storage; %[Gb]
    Telemetry_TX = Data_Storage.Telemetry_TX; %[Mb/s]
    Payload_TX = Data_Storage.Payload_TX; %[Mb/s]
    Command_RX = Data_Storage.Command_RX; %[Mb/s]
    Telemetry_Gen = Data_Storage.Telemetry_Gen; %[Mb/s]
    Payload_Gen = Data_Storage.Payload_Gen; %[Mb/s]
    
    % Let's assume we generate telemetry and payload data first:
    Data_Gen = (Telemetry_Gen + Payload_Gen)*dt;

    if Ava_Storage - (Data_Gen/1000) >= 0
        Ava_Storage = Ava_Storage - (Data_Gen/1000);
    else
        disp('Data Storage Error: Insufficient Storage Space')
    end

    % Now we will transmit the requested amount of data to the ground
    % station:
    Data_Out = (Telemetry_TX + Payload_TX)*dt;
    
    if Ava_Storage + (Data_Out/1000) <= Total_Storage
        Ava_Storage = Ava_Storage + (Data_Out/1000);
    else
        disp(['Data Transmission Error: Transmission packet is' ...
            ' larger than the amount of stored data.'])
    end

    % Finally we will receive new data from the ground station:
    Data_In = Command_RX*dt;
    
    if Ava_Storage - (Data_In/1000) >= 0
        Ava_Storage = Ava_Storage - (Data_In/1000);
    else
        disp('Data Storage Error: Insufficient Storage Space')
    end
  
    Data_Storage.Ava_Storage = Ava_Storage;
end