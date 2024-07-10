function plot_moon(Params)

Rm = Params.R; %[km]
[x, y, z] = sphere;
x = x*Rm;
y = y*Rm;
z = z*Rm;
globe=surf(x, y, z);
cdata = imread('moon_full.jpg');
set(globe,  'FaceColor',  'texturemap',  'CData', cdata,'EdgeColor',  'none');
axis equal

% light('Position',[-1 0 0],'Style','local') %add sun light, optional
general_settings
xlabel('x')
ylabel('y')
zlabel('z')

end
