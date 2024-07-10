
function [Elapsed_sec, DataFromSTK] = Read_Data_From_STK(Original_File_Name)


% ---------------------------------- GENERAL ---------------------------------- %
% This function reads txt. file from STK and converts it into data and time
% ----------------------------------------------------------------------------- %

% ----------------------------------- INPUT ----------------------------------- %
% The input is the name of the txt. file (char/str type; without format).
% Examples:
% [Dates, DataFromSTK] = Read_Data_From_STK('JerricoFileFromSTK');
% [Dates, DataFromSTK] = Read_Data_From_STK('Jerrico_inclinationTOD');
% ------------------------------------------------------------------------------ %

% ----------------------------------- OUTPUT ----------------------------------- %
% The output is: 
%             1. Elapsed_sec   =   A vector of elapsed seconds since start
%                                 of mission
%             2. DataFromSTK   =   A matrix of all the data in the original
%                                 txt file, divided into columns
% ------------------------------------------------------------------------------ %

% ---------------------------------- WARNINGS ---------------------------------- %
% 1. This function is only valid when there is only one column of time
%    (cannot be used for eclipse summary)
% 2. The time vector only includes dates (without specific hour)
% ------------------------------------------------------------------------------ %

% ------------------------------------------------------------------------------ %
% (C) May Alon, The JERICCO project, Mission and Orbit Design
% For any questions or comments: may.alon@campus.technion.ac.il
% Last update: 16.06.24 (Yarden & Benny - NovaSAT)
% ------------------------------------------------------------------------------ %

fclose('all');
File_Name = [Original_File_Name,'.txt'];

disp([newline,'----------------------',newline,...
    'Working On It...',newline,...
    '----------------------',newline])

New_File = [Original_File_Name,'_EDITED.txt'];

copyfile(File_Name,New_File);
fid = fopen(File_Name,'r');
FileFromSTK_To_Edit = fopen(New_File,'w');

while ~feof(fid)
    tline = fgetl(fid);
    if contains(tline,'----')
        break
    end
end

while ~feof(fid)
    tline = fgetl(fid);
    for i=1:12
        newlinetxt = tline;
        if contains(newlinetxt,char(month(datetime(2024,i,1),'shortname')))
            newline_temp = strrep(newlinetxt,char(month(datetime(2024,i,1),'shortname')),num2str(i));
        end
    end
    fprintf(FileFromSTK_To_Edit,'%s',[newline_temp,newline]);
end

fclose('all');
clearvars -except New_File
% Taking data from table and converting the data and time
% to elapsed seconds since first time
AllData_temp = readtable(New_File, 'ReadVariableNames', false);
AllData = [AllData_temp(:,3),AllData_temp(:,2),AllData_temp(:,1),AllData_temp(:,4),AllData_temp(:,5:end)];

Dates_sec = datenum(datetime(table2array(AllData(:,1:3))));
Dates_sec = 86400*(Dates_sec - Dates_sec(1));
Time_sec = seconds(table2array(AllData_temp(:,4)));
Total_sec = Dates_sec + Time_sec;
Elapsed_sec = Total_sec - Total_sec(1);
DataFromSTK = AllData(:,5:end);

delete(New_File);
clear AllData_temp AllData 

disp([newline,'----------------------',newline,...
    'Your Data Is Ready!',newline,...
    '----------------------',newline])

% Transfer Data to elapsed seconds

%{
    
% Example
clc
clear
close all
    
[Dates, DataFromSTK] = Read_Data_From_STK('Jerrico_inclinationTOD');
    
plot(Dates,DataFromSTK(:,1),'linewidth',1.5)
% datetick('x','mmm-yy')
set(gcf,'color','w');%white background
box on
grid on
set(gca,'FontSize',13);ylabel('i [deg]')
    
%}
    
end



