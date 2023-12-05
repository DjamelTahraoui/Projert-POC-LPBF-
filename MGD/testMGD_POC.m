clear; clc; close all;

% Angles de rotation
syms theta_x theta_y 

% Paramètres de la source laser dans Rs
P0_sourcePosition = [0; 0; 0; 1];
u0_sourceDirection = [-1; 0; 0; 0];

%% Matrices de transformation homogene
s_T_ax0 = TaitBryanXYZ_Transformation(-45*pi/180, 75*pi/180, -90*pi/180,[-146.971; 1.696; 0]);
ax0_T_ax = TaitBryanXYZ_Transformation(theta_x, 0, 0, [0; 0; 0]);
ax_T_mx = TaitBryanXYZ_Transformation(0, 0, 0, [0; 0; 1.98]);
s_T_mx = s_T_ax0 * ax0_T_ax * ax_T_mx;

s_T_ay0 = TaitBryanXYZ_Transformation(-142.5*pi/180, 0, 0,[0; -38.387; 10.752]);
ay0_T_ay = TaitBryanXYZ_Transformation(theta_y, 0, 0, [0; 0; 0]);
ay_T_my = TaitBryanXYZ_Transformation(0, 0, 0, [0; 0; 1.98]);
s_T_my = s_T_ay0 * ay0_T_ay * ay_T_my;

s_T_v1 = TaitBryanXYZ_Transformation(0, 0, 0,[-145.8090673478875; -36.4266525410351; -30.248]);
s_T_v2 = TaitBryanXYZ_Transformation(0, 0, 0,[-145.8090673478875; -36.4266525410351; -65.048]);
s_T_p = TaitBryanXYZ_Transformation(0, 0, 0,[-145.8090673478875; -36.4266525410351; -714.248]);

%% Centres de reperes et normales (en Rs)
Omx = s_T_mx(:,4);
Z_mx = s_T_mx(:,3);
Omy = s_T_my(:,4);
Z_my = s_T_my(:,3);
Ov1 = s_T_v1(:,4);
Z_v1 = s_T_v1(:,3);
Ov2 = s_T_v2(:,4);
Z_v2 = s_T_v2(:,3); 
Op = s_T_p(:,4); 
Z_p = s_T_p(:,3);

%% Calcul du position du laser dans le plan de travail nominal
P1 = MirrorIntersection(Omx, Z_mx, P0_sourcePosition, u0_sourceDirection);  

u1 = u0_sourceDirection - 2*Z_mx*u0_sourceDirection'*Z_mx;      

P2 = MirrorIntersection(Omy, Z_my, P1, u1);  

u2 = u1 - 2*Z_my*u1'*Z_my;      

P3 = MirrorIntersection(Ov1, Z_v1, P2, u2);

P4 = windowsIntersection(Z_v1, P3, u2);  

P5 = MirrorIntersection(Ov2, Z_v2, P4, u2); 

P6 = windowsIntersection(Z_v2, P5, u2); 

P7 = MirrorIntersection(Op, Z_p, P6, u2); 

laser_position_on_workplane = inv(s_T_p) * P7; 

%% Substituer theta_x et theta_y par des valeurs de test 
point7_test_value = subs(laser_position_on_workplane, [theta_x, theta_y], [0.1*pi, 0.1*pi]);
point7_numeric = double(point7_test_value(1:2,:));
disp(point7_numeric);
