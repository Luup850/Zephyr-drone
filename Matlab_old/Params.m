clear all;
%m = 1; % KG
%g = 9.81
% RPM to Thrust func f(x)=x^(2)*0.0005
%diameter = 1;

%sys = tf([-g*m], [m, 0, 0])

I_xx = 3*10^(-6);
I_yy = 3*10^(-6);
I_zz = 1*10^(-5);
I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];


g = 9.81;
m = 0.5;
k = 0.01;
l = 0.225; %L
kd = 0.01;
B = 0.001;

% Angular velocity required for hover.
% f = w1^2 + w3^2 + w4^2) * 0.02 = g * m
% (g * m)
format long g
%w_hover = ((g * m) / 4) / B
% w_hover = 7.830225
w_hover = 7.830229882;

kp_h = 218.061061208643;
tau_i_h = 110.138309959241;
tau_d_h = 44.6853606001277;

kp_r = 0.0160130109024398;
tau_i_r = 0.00736804577025044;
tau_d_r = 0.00653126952336714;
N_r = 1142.8279760416;


%% Stuff
syms phi theta psi phi_dot theta_dot psi_dot omega_x omega_y omega_z z
ang_vel = [phi_dot; theta_dot; psi_dot];
rot_mat = [1 0 -sin(theta); 0 cos(phi) cos(theta)*sin(phi); 0 -sin(phi) cos(theta)*cos(phi)];
% w = rot_mat * ang_vel
% Take inverse on both sides.
rotational_vel = (inv(rot_mat) * [omega_x ; omega_y ; omega_z]);

r_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
r_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
r_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
rot_mat2 = r_z*r_y*r_x;

final = rot_mat2*[0; 0; z];