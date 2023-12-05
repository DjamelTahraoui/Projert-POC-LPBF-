% clear; clc; close all;

% erreurs source laser
syms E_ys E_zs E_bs E_cs
% erreurs axe de rotation x
syms E_yax E_zax E_aax E_bax E_cax
% erreurs miroir x
syms E_zmx E_amx E_bmx 
% erreurs axe de rotation y
syms E_yay E_zay E_aay E_bay E_cay
% erreurs miroir y
syms E_zmy E_amy E_bmy
% erreurs vitre 1
syms E_aw1 E_bw1 
% erreurs vitre 2
syms E_aw2 E_bw2 
% erreurs plan de travail 
syms E_xp E_yp E_zp E_ap E_bp E_cp

% Angles de rotation 
syms theta_x theta_y 

%% Matrices de transformation homogène
s_T_ax0 = TaitBryanXYZ_Transformation(-45*pi/180, 75*pi/180, -90*pi/180, [-146.971; 1.696; 0]);
ax0d_T_ax = TaitBryanXYZ_Transformation(theta_x, 0, 0, [0; 0; 0]); % Mouvement à checker
ax_T_mx = TaitBryanXYZ_Transformation(0, 0, 0, [0; 0; 1.98]);

s_T_ay0 = TaitBryanXYZ_Transformation(-142.5*pi/180, 0, 0, [0; -38.387; 10.752]);
ay0d_T_ay = TaitBryanXYZ_Transformation(theta_y, 0, 0, [0; 0; 0]); % Mouvement à checker
ay_T_my = TaitBryanXYZ_Transformation(0, 0, 0, [0; 0; 1.98]);

s_T_v1 = TaitBryanXYZ_Transformation(0, 0, 0, [-145.8090673478875; -36.4266525410351; -30.248]);
s_T_v2 = TaitBryanXYZ_Transformation(0, 0, 0, [-145.8090673478875; -36.4266525410351; -65.048]);
s_T_p = TaitBryanXYZ_Transformation(0, 0, 0, [-145.8090673478875; -36.4266525410351; -714.248]);

%% Intégration des défauts

s_T_sd = TaitBryanXYZ_Transformation(0, E_bs, E_cs, [0, E_ys, E_zs]);
ax0_T_ax0d = TaitBryanXYZ_Transformation(E_aax, E_bax, E_cax, [0, E_yax, E_zax]);
mx_T_mxd = TaitBryanXYZ_Transformation(E_amx, E_bmx, 0, [0, 0, E_zmx]);
ay0_T_ay0d = TaitBryanXYZ_Transformation(E_aay, E_bay, E_cay, [0, E_yay, E_zay]);
my_T_myd = TaitBryanXYZ_Transformation(E_amy, E_bmy, 0, [0, 0, E_zmy]);
v1_T_v1d = TaitBryanXYZ_Transformation(E_aw1, E_bw1, 0, [0, 0, 0]);
v2_T_v2d = TaitBryanXYZ_Transformation(E_aw2, E_bw2, 0, [0, 0, 0]);
p_T_pd = TaitBryanXYZ_Transformation(E_ap, E_bp, E_cp, [E_xp, E_yp, E_zp]);
s_T_mxd = s_T_ax0 * ax0_T_ax0d * ax0d_T_ax * ax_T_mx * mx_T_mxd; % s vers mxd
s_T_myd = s_T_ay0 * ay0_T_ay0d * ay0d_T_ay * ay_T_my * my_T_myd;% s vers myd

%% Centres de reperes et normales (en Rs)
Z_mxd = s_T_mxd(:,3);
Omxd = s_T_mxd(:,4);
Z_myd = s_T_myd(:,3);
Omyd = s_T_myd(:,4);
Z_v1d = s_T_v1 * v1_T_v1d(:,3);  
Ov1d = s_T_v1 * v1_T_v1d(:,4);  
Z_v2d = s_T_v2 * v2_T_v2d(:,3);    
Ov2d = s_T_v2 * v2_T_v2d(:,4);    
Z_pd = s_T_p * p_T_pd(:,3);
Opd = s_T_p * p_T_pd(:,4);

%% Calcul du position du laser dans le plan de travail reel 
P1 = MirrorIntersection(Omxd, Z_mxd, s_T_sd(:,4), s_T_sd * [-1; 0; 0; 0]);

u1 = s_T_sd * [-1; 0; 0; 0] - 2 * Z_mxd * transpose(s_T_sd * [-1; 0; 0; 0]) * Z_mxd;   

P2 = MirrorIntersection(Omyd, Z_myd, P1, u1);  

u2 = u1 - 2 * Z_myd * u1' * Z_myd; 

P3 = MirrorIntersection(Ov1d, Z_v1d, P2, u2); 

P4 = windowsIntersection(Z_v1d, P3, u2);  

P5 = MirrorIntersection(Ov2d, Z_v2d, P4, u2); 

P6 = windowsIntersection(Z_v2d, P5, u2); 

P7 = MirrorIntersection(Opd, Z_pd, P6, u2); 

laser_position_on_workplane_with_defects = inv(s_T_p * p_T_pd) * P7; 

%% Substituer theta_x et theta_y par des valeurs de test 

Erreurs = [E_ys, E_zs, E_bs, E_cs, ...
           E_yax, E_zax, E_aax, E_bax, E_cax, ...
           E_zmx, E_amx, E_bmx, ...
           E_yay, E_zay, E_aay, E_bay, E_cay, ...
           E_zmy, E_amy, E_bmy, ...
           E_aw1, E_bw1, ...
           E_aw2, E_bw2, ...
           E_xp, E_yp, E_zp, E_ap, E_bp, E_cp];

point7_test_value = subs(laser_position_on_workplane_with_defects, [Erreurs, theta_x, theta_y], [zeros(size(Erreurs)), 0.1*pi, 0.1*pi]);
point7_numeric = double(point7_test_value(1:2,:));
disp(point7_numeric);
