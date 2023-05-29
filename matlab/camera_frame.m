% Roll pitch yaw
syms a b g
%a = 0; b = 0; g = pi;
R_zg = [cos(g) -sin(g) 0; sin(g) cos(g) 0; 0 0 1];
R_yb = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
R_xa = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];

R = R_zg * R_yb * R_xa

R*[1; 1; 1];
inv(R)*(R*[1; 1; 1]);