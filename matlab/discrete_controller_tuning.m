h1gm = 32;
h1Ni = 8;
h1al = 0.2;
w = logspace(0,3,3000);

[wc h1kp h1ti h1td ok] = findpid_discrete(Gh1b_d, h1gm, h1Ni, h1al, Ts, w);

% Discrete controller open-loop
Gh1b_d_c_ol = h1kp * Gh1b_d * c_d * c_i;
% Discrete controller closed-loop (h1kp * c_i * c_d + hoverControl) * Gh1b_d
Gh1b_d_c_cl = (h1kp * Gh1b_d * c_i * c_d) / (1 + h1kp * Gh1b_d * c_d * c_i)
figure(1009)
bode(Gh1b_d, Gh1b_d_c_ol)
grid on
legend('Tf', 'Tf + Controller')