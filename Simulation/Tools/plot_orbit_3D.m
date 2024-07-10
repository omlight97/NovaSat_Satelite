function plot_orbit_3D(xa)

% plot the orbit of the satellite in 3D
% The input is a matrix, which its first 3 columns are the components of the
% radius vector in a cartesian coordinate system
% May Alon

plot3(xa(:,1),xa(:,2),xa(:,3),'-','linewidth',1.5,'displayname','trajectory','color',[255,174,201]/255)
plot3(xa(1,1),xa(1,2),xa(1,3),'*','linewidth',1.5,'markersize',10); %Initial position
axis equal
xlabel('x(km)','interpreter', 'latex', 'fontsize', 21)
ylabel('y(km)','interpreter', 'latex','fontsize', 21)
zlabel('z(km)','interpreter', 'latex','fontsize', 21)
general_settings
legend show

end