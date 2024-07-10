
function [Dates, DataFromSTK] = Read_Data_From_STK(Original_File_Name)


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
%             1. Dates         =   A vector of "daytime" type, includes only dates
%                                 (without specific hour)
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
% Last update: 26.03.22
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
        if contains(newlinetxt,char(month(datetime(2022,i,1),'shortname')))
            newline_temp = strrep(newlinetxt,char(month(datetime(2022,i,1),'shortname')),num2str(i));
        end
    end
    fprintf(FileFromSTK_To_Edit,'%s',[newline_temp,newline]);
end

fclose('all');
clearvars -except New_File
AllData_temp = load(New_File);
AllData = [AllData_temp(:,3),AllData_temp(:,2),AllData_temp(:,1),AllData_temp(:,5:end)];

Dates = datetime(AllData(:,1:3));
DataFromSTK = AllData(:,4:end);

delete(New_File);
clear AllData_temp AllData 

disp([newline,'----------------------',newline,...
    'Your Data Is Ready!',newline,...
    '----------------------',newline])

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



