close all

origin = [0, 0, 0];
U = 10; %1 U, 10 cm
tr = 0.3;

new_origin = [U, U, 0];

a = 2*U;
b = a;
c = 3*U;

X_body = [0 0 0 0 0 a; a 0 a a a a; a 0 a a a a; 0 0 0 0 0 a];
Y_body = [0 0 0 0 b 0; 0 b 0 0 b b; 0 b b b b b; 0 0 b b b 0];
Z_body = [0 0 c 0 0 0; 0 0 c 0 0 0; c c c 0 c c; c c c 0 c c];

[X_camera, Y_camera, Z_camera] = sphere;

% X_camera = 0.3*U*X_camera - new_origin(1);
% Y_camera = 0.3*U*Y_camera - new_origin(1);
X_camera = 0.3*U*X_camera;
Y_camera = 0.3*U*Y_camera;
Z_camera = 0.2*U*Z_camera;

X = X_body; Y = Y_body; Z = Z_body;

C = 'blue';

X = X - new_origin(1);
Y = Y - new_origin(2);

s1 = fill3(X,Y,Z,C,'FaceAlpha',tr);
hold on
s2 = surf(X_camera,Y_camera,Z_camera);
s2.FaceColor = 'green';
s2.FaceAlpha = 0.5;
axis equal


direction = [1, 1, 1];
angle = 0; %deg

rotate(s1,direction,angle)
rotate(s2,direction,angle)
hold off
