h1gm = 40;
h1Ni = 4;
h1al = 0.07;
w = logspace(0,3,3000);


[wc h1kp h1ti h1td ok] = findpid_discrete(bla, h1gm, h1Ni, h1al, Ts, w);

% Discrete controller open-loop
c_d = c2d(tf([h1td 1], [h1td*h1al 1]), Ts, 'tustin');
c_i = c2d(tf([h1ti 1], [h1ti 0]),Ts,'tustin');
Gh1b_d_c_ol = h1kp * Gh1b_d * c_d * c_i;
% Discrete controller closed-loop (h1kp * c_i * c_d + hoverControl) * Gh1b_d
Gh1b_d_c_cl = (h1kp * Gh1b_d * c_i * c_d) / (1 + h1kp * Gh1b_d * c_d * c_i);
figure(1009)
bode(Gh1b_d, Gh1b_d_c_ol)
grid on
legend('Tf', 'Tf + Controller')

%% Test with height vel and pos control
h1vgm = 35;
h1vNi = 8;
h1val = 0.6;
Gh1b_d_v = c2d(Gh1b * tf([1 0], [1]), Ts, 'zoh');
[wc h1vkp h1vti h1vtd ok] = findpid_discrete(Gh1b_d_v, h1vgm, h1vti, h1val, Ts, w)
c_v_d = c2d(tf([h1vtd 1], [h1vtd*h1val 1]), Ts, 'tustin');
c_v_i = c2d(tf([h1vti 1], [h1vti 0]),Ts,'tustin');
Gh1b_d_p_cl = (h1vkp * Gh1b_d_v * c_v_i) / (1 + c_v_d * h1vkp * Gh1b_d_v * c_v_i);

% Pos controller
Gh1b_d_p = Gh1b_d_p_cl * c2d(tf([1], [1 0]), Ts, 'zoh');
[wc h1kp ok] = findp_discrete(Gh1b_d_p, 32, w)



