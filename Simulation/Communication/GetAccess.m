function [Access_Times] = GetAccess(Time_vec,WantedData)

% ---------------------------------- GENERAL ---------------------------------- %
% This function reads txt. file from STK and converts it into a vector that
% for each time slot is 0 if it doesnt see the communication satellite or 1
% if it sees the communication satellite
% ----------------------------------------------------------------------------- %

% ----------------------------------- INPUT ----------------------------------- %
% The input is the name of the vector of simulation times
% ------------------------------------------------------------------------------ %

% ----------------------------------- OUTPUT ----------------------------------- %
% The output is: 
%             For N communication satellites:
%             We get a vector with N columms and each row is a time step
%             cell = 0 if we dont see the satellite
%             cell = 1 if we see the satellite
% ------------------------------------------------------------------------------ %

% ---------------------------------- WARNINGS ---------------------------------- %
% Please Place .txt files named "Inmarsat#.txt in the folder of the
% function (Communication)
% ------------------------------------------------------------------------------ %

% ------------------------------------------------------------------------------ %
% (C) Yarden Milshtein & Benny Muchnick - NovaSAT
% Last update: 1.7.24 (Yarden & Benny - NovaSAT)
% ------------------------------------------------------------------------------ %

if contains(WantedData,'Sun') || contains(WantedData,'GS')
    Instances = 1;
    Access_Times = zeros(length(Time_vec),Instances);
elseif contains(WantedData, 'Inmarsat')
    Instances = 3;
    Access_Times = zeros(length(Time_vec),Instances);
end

Instances = 3; % How many files do we have

Start_Date = [2026,1,1];
Start_Time = 10; % Start hour of the day

Start_Date_sec = datenum(datetime(Start_Date))*86400;
Start_time_sec = Start_Time*3600;
Start_Point = Start_Date_sec + Start_time_sec;

if contains(WantedData,'Sun')
        
        File_Name = ['Sun','.txt']; 
        Comm_Table = readtable(File_Name,'ReadVariableNames', false); 
           
        % Filtering the end of the table that is not numbers
        NaN_index = find(isnan(table2array(Comm_Table(:,1))),1);
        Comm_Table = Comm_Table(1:NaN_index-1,:);
        
        % Finding vector of start times in seconds since start
        Comm_Start_Temp(:,1) = Comm_Table.Var1; 
        Comm_Start_Temp(:,2) = Name2Month(Comm_Table.Var2);
        Comm_Start_Temp(:,3) = Comm_Table.Var3;
    
        Comm_Start_Date = datenum(datetime(Comm_Start_Temp(:,3),Comm_Start_Temp(:,2),Comm_Start_Temp(:,1)))*86400;
        Comm_Start_Time = seconds(table2array(Comm_Table(:,4)));
        Comm_Start_sec = Comm_Start_Date + Comm_Start_Time;
        
        Comm_Start_Final = Comm_Start_sec - Start_Point;
        
        % Finding vector of stop times in seconds since start
        Comm_Stop_Temp(:,1) = Comm_Table.Var5; 
        Comm_Stop_Temp(:,2) = Name2Month(Comm_Table.Var6);
        Comm_Stop_Temp(:,3) = Comm_Table.Var7;
    
        Comm_Stop_Date = datenum(datetime(Comm_Stop_Temp(:,3),Comm_Stop_Temp(:,2),Comm_Stop_Temp(:,1)))*86400;
        Comm_Stop_Time = seconds(table2array(Comm_Table(:,8)));
        Comm_Stop_sec = Comm_Stop_Date + Comm_Stop_Time;
    
        Comm_Stop_Final = Comm_Stop_sec - Start_Point;
        
        % We are going to populate the Communications time vector with 1 if we
        % are in an active communication time with sat #k
        for j =1:length(Comm_Stop_Final)
            ind_start = find(Time_vec>=Comm_Start_Final(j),1);
            ind_stop = find(Time_vec>=Comm_Stop_Final(j),1);
            Access_Times(ind_start:ind_stop) = 1;
       end

elseif contains(WantedData,'GS')
        
        File_Name = ['GS','.txt']; 
        Comm_Table = readtable(File_Name,'ReadVariableNames', false); 
           
        % Filtering the end of the table that is not numbers
        NaN_index = find(isnan(table2array(Comm_Table(:,1))),1);
        Comm_Table = Comm_Table(1:NaN_index-1,:);
        
        % Finding vector of start times in seconds since start
        Comm_Start_Temp(:,1) = Comm_Table.Var2; 
        Comm_Start_Temp(:,2) = Name2Month(Comm_Table.Var3);
        Comm_Start_Temp(:,3) = Comm_Table.Var4;
    
        Comm_Start_Date = datenum(datetime(Comm_Start_Temp(:,3),Comm_Start_Temp(:,2),Comm_Start_Temp(:,1)))*86400;
        Comm_Start_Time = seconds(table2array(Comm_Table(:,5)));
        Comm_Start_sec = Comm_Start_Date + Comm_Start_Time;
        
        Comm_Start_Final = Comm_Start_sec - Start_Point;
        
        % Finding vector of stop times in seconds since start
        Comm_Stop_Temp(:,1) = Comm_Table.Var6; 
        Comm_Stop_Temp(:,2) = Name2Month(Comm_Table.Var7);
        Comm_Stop_Temp(:,3) = Comm_Table.Var8;
    
        Comm_Stop_Date = datenum(datetime(Comm_Stop_Temp(:,3),Comm_Stop_Temp(:,2),Comm_Stop_Temp(:,1)))*86400;
        Comm_Stop_Time = seconds(table2array(Comm_Table(:,9)));
        Comm_Stop_sec = Comm_Stop_Date + Comm_Stop_Time;
    
        Comm_Stop_Final = Comm_Stop_sec - Start_Point;
        
        % We are going to populate the Communications time vector with 1 if we
        % are in an active communication time with sat #k
        for j =1:length(Comm_Stop_Final)
            ind_start = find(Time_vec>=Comm_Start_Final(j),1);
            ind_stop = find(Time_vec>=Comm_Stop_Final(j),1);
            Access_Times(ind_start:ind_stop) = 1;
       end

elseif contains(WantedData, 'Inmarsat')
    for k=1:Instances
        clear Comm_Start_Temp Comm_Start_Date Comm_Start_Time Comm_Start_sec Comm_Start_Final
        clear Comm_Stop_Temp Comm_Stop_Date Comm_Stop_Time Comm_Stop_sec Comm_Stop_Final
        
        File_Name = ['Inmarsat',num2str(k),'.txt'];
        
        Comm_Table = readtable(File_Name,'ReadVariableNames', false);    
    
        % Filtering the end of the table that is not numbers
        NaN_index = find(isnan(table2array(Comm_Table(:,1))),1);
        Comm_Table = Comm_Table(1:NaN_index-1,:);
        Comm_Start_Temp(:,1) = Comm_Table.Var2; 
        Comm_Start_Temp(:,2) = Name2Month(Comm_Table.Var3);
        Comm_Start_Temp(:,3) = Comm_Table.Var4;
    
        Comm_Start_Date = datenum(datetime(Comm_Start_Temp(:,3),Comm_Start_Temp(:,2),Comm_Start_Temp(:,1)))*86400;
        Comm_Start_Time = seconds(table2array(Comm_Table(:,5)));
        Comm_Start_sec = Comm_Start_Date + Comm_Start_Time;
        
        Comm_Start_Final = Comm_Start_sec - Start_Point;
        
        % Finding vector of stop times in seconds since start
        Comm_Stop_Temp(:,1) = Comm_Table.Var6; 
        Comm_Stop_Temp(:,2) = Name2Month(Comm_Table.Var7);
        Comm_Stop_Temp(:,3) = Comm_Table.Var8;
    
        Comm_Stop_Date = datenum(datetime(Comm_Stop_Temp(:,3),Comm_Stop_Temp(:,2),Comm_Stop_Temp(:,1)))*86400;
        Comm_Stop_Time = seconds(table2array(Comm_Table(:,9)));
        Comm_Stop_sec = Comm_Stop_Date + Comm_Stop_Time;
    
        Comm_Stop_Final = Comm_Stop_sec - Start_Point;
        
        % We are going to populate the Communications time vector with 1 if we
        % are in an active communication time with sat #k
        for j =1:length(Comm_Stop_Final)
            ind_start = find(Time_vec>=Comm_Start_Final(j),1);
            ind_stop = find(Time_vec>=Comm_Stop_Final(j),1);
            Access_Times(ind_start:ind_stop,k) = 1;
        end
       
    end
end
end
