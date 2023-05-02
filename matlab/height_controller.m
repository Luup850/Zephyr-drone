h1kp = 1;
h1ti = 0;
h1al = 0.07;
h1Ni = 4;

gm = 32;
phi_d = asind((1-h1al)/(1+h1al)); % Phase increase at peak
phi_i = atand(h1Ni) - 90;

Gwc = -180 + gm - phi_d - phi_i;

% target = top peak location for lead bubble.
target = 2.36
h1td = 1 / (sqrt(h1al) * target)

h1ti = h1Ni * (1/ target)

h1kp = 10^(33.1/20)
%h1kp = 1;
% Lead controller
c_d = c2d(tf([h1td 1], [h1td*h1al 1]), Ts, 'tustin');
c_i = c2d(tf([h1ti 1], [h1ti 0]),Ts,'tustin');
%c_i = 1;

M_d = 1/h1al;
M_i = sqrt(1+(1/(h1Ni^2)))
G_gain = 10^(0.141/20);

%h1kp = 1/(M_d * M_i * G_gain)

% Discrete controller open-loop
Gh1b_d_c_ol = h1kp * Gh1b_d * c_d * c_i;
% Discrete controller closed-loop
Gh1b_d_c_cl = (h1kp * Gh1b_d * c_i) / (1 + h1kp * Gh1b_d * c_d * c_i)
figure(1009)
bode(Gh1b_d, Gh1b_d_c_ol)
grid on
legend('Tf', 'Tf + Controller')

figure(1010)
step(Gh1b_d_c_cl)