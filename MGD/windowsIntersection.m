function Pj = windowsIntersection(n, Pi, ui)

    D = 0.6859 * sqrt(1 - (-transpose(ui(1:3,1)) * n(1:3,1))^2);
    uCz = cross(ui(1:3,1), n(1:3,1));
    u = cross(n(1:3,1), uCz) / norm(uCz);
    t = [u ; 0];
    Pj = Pi + 3.05 * ( D * t / sqrt(1 - D^2) - n);
end
