function Plot_Orbit_and_Moon(SimData, Params)

plot_moon(Params)
hold on
plot3(SimData.orbit.r(:,1),SimData.orbit.r(:,2),SimData.orbit.r(:,3),...
    '-','color',[255,174,201]/255,'Linewidth',3)
plot3(SimData.orbit.r(1,1),SimData.orbit.r(1,2),SimData.orbit.r(1,3),...
    '*','color','red','Markersize',12)

[z_max, ind] = max(SimData.orbit.r(:,3));
x_at_zmax = SimData.orbit.r(ind,1);
y_at_zmax = SimData.orbit.r(ind,2);
plot3(x_at_zmax,y_at_zmax,z_max,'<','markersize',8,'linewidth',6,'color','black')
legend({'The Moon','Orbit', 'Initial Position','Direction of Motion'},'Location','northeast')

xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')

general_settings
end
