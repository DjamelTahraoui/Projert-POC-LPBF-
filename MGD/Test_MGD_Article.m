% les constantes
Xax0_val = -146.971;
Yax0_val = 1.696;
Aax0_val = -45 * pi / 180;
Bax0_val = 75 * pi / 180;
Cax0_val = -90 * pi / 180;
Yay0_val = -38.387 ;
Zay0_val = 10.752;
Aay0_val = -142.5 * pi / 180;
Xp_val = -145.8090673478875;
Yp_val = -36.4266525410351;
Zp_val = -714.2480000;
epm_val = 1.98;
ev_val = 3.05;
n1_val = 1;
n2_val = 1.458;

% Déclarer les angles theta_x et theta_y comme symboles
syms theta_x theta_y

% Calculer les expressions symboliques pour K1, K2 et K3
K1 = (epm_val + Zay0_val * cos(Aay0_val + theta_y) - Yay0_val * sin(Aay0_val + theta_y)) / cos(Aay0_val - Bax0_val + theta_y);
K2 = Zp_val + 2 * ev_val - K1 * cos(Bax0_val);
K3 = sqrt(cos(2 * theta_x)^2 * cos(2 * theta_y)^2 - 1 + n2_val^2);

% Calculer les expressions symboliques pour x et y
x = -Xp_val + Xax0_val + (epm_val - Yax0_val * cos(-Aax0_val + theta_x) * sin(Bax0_val)) / sin(-Aax0_val + theta_x) - K2 * tan(2 * theta_x) / cos(2 * theta_y) - K1 * tan(2 * theta_x) + 2 * ev_val * sin(2 * theta_x) / K3;
y = -Yp_val - K1 * sin(Bax0_val) - K2 * tan(2 * theta_y) + 2 * ev_val * cos(2 * theta_x) * sin(2 * theta_y) / K3;

% % Afficher les expressions symboliques avant substitution
% disp('Expression symbolique de x avant substitution :');
% disp(x);
% 
% disp('Expression symbolique de y avant substitution :');
% disp(y);

% Affecter les valeurs spécifiques de theta_x et theta_y
theta_x_val = 0.1*pi;
theta_y_val = 0.1*pi;

% Substitution des valeurs de theta_x et theta_y dans x et y
x_val = double(subs(x, [theta_x, theta_y], [theta_x_val, theta_y_val]));
y_val = double(subs(y, [theta_x, theta_y], [theta_x_val, theta_y_val]));

[x_val ; y_val]
