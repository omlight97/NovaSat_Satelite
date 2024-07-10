function plot_orbital_elements(t,all_orbital_elements)

%---------------------- What Does This Function Do? ----------------------%

% This function gets as an input a time vector and a matrix, which each row
% of it is a different orbital element at a given moment. The vector of the
% orbital elements is:

%                      all_orbital_elements = [a, ecc, inc, RAAN, omega, f];

%------------------------------ Warnings ------------------------------%

% All angles should be in radians
% The angles in the plot are in [deg], except for M (mean anomaly)

%-----------------------------------------------------------------------%

t_days = t/(24*60*60);

% elements_name_for_legend = {'semi major axis','eccentricity','inclination','RAAN'...
%     ,'argument of perigee','true anomaly'};
% elements_name_for_axis = {'a [km]','e','i [deg]','RAAN [deg]'...
%     ,'\omega [deg]','f [deg]'};

elements_name_for_legend = {'semi major axis','eccentricity',...
    'inclination','\Omega','argument of perigee','mean anomaly'};
elements_name_for_axis = {'a [km]','e','i [deg]','RAAN [rad]'...
    ,'\omega [deg]','M [deg]'};

hold on
figure
Len = length(elements_name_for_legend);
for k=1:Len
    subplot(3,2,k);
    if k<3
        plot(t_days,all_orbital_elements(:,k),'linewidth',1.5,'displayname',elements_name_for_legend{k});
    else
        plot(t_days,rad2deg(all_orbital_elements(:,k)),'linewidth',1.5,'displayname',elements_name_for_legend{k});
    end
    title(['(',num2str(k),')'])
    xlabel('Time [days]')
    ylabel(elements_name_for_axis{k})
    general_settings
end
hold off

end
