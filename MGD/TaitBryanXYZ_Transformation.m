function T = TaitBryanXYZ_Transformation(alpha, beta, gamma, translation)
   
    % Matrices de rotation autour de chaque axe
    Rx = [1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
    Ry = [cos(beta), 0, sin(beta); 0, 1, 0; -sin(beta), 0, cos(beta)];
    Rz = [cos(gamma), -sin(gamma), 0; sin(gamma), cos(gamma), 0; 0, 0, 1];

    % Matrice de rotation totale
    R = Rz * Ry * Rx;

    % Matrice de transformation homogène symbolique
    T = sym(eye(4));
    T(1:3, 1:3) = R;
    T(1:3, 4) = sym(translation);
end
