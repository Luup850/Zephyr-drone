h1gm = 45;
h1Ni = 4; % Try 8 as well
h1al = 0.07;
w = logspace(0,3,3000);


[wc h1kp h1ti h1td ok] = findpid_discrete(Gh1b_d, h1gm, h1Ni, h1al, Ts, w);

% Discrete controller open-loop
c_d = c2d(tf([h1td 1], [h1td*h1al 1]), Ts, 'tustin');
c_i = c2d(tf([h1ti 1], [h1ti 0]),Ts,'tustin');
Gh1b_d_c_ol = h1kp * Gh1b_d * c_d * c_i;
% Discrete controller closed-loop (h1kp * c_i * c_d + hoverControl) * Gh1b_d
Gh1b_d_c_cl = feedback(Gh1b_d_c_ol, 1);
step(Gh1b_d_c_cl)
figure(1009)
bode(Gh1b_d, Gh1b_d_c_ol)
grid on
legend('Tf', 'Tf + Controller')

%% Test with height vel and pos control
h1vgm = 60;
h1vNi = 8;
h1val = 0.07;

h1gm = 60;
h1Ni = 4;
h1al = 0.07;

%2.500191470 Actual frequency used
%Gh1b_d_v = c2d(Gh1b * tf([1 0], [1]), Ts, 'zoh');
%[wc1 h1vkp h1vtd ok] = findpd_discrete(Gh1bv_d, h1vgm, h1val, Ts, w)
c_v_d = c2d(tf([h1vtd 1], [h1vtd*h1val 1]), Ts, 'tustin');
%c_v_i = c2d(tf([h1vti 1], [h1vti 0]),Ts,'tustin');
c_v_i = 1;
Gh1b_d_p_ol = h1vkp * Gh1bv_d * c_v_i * c_v_d;
bode(Gh1b_d_p_ol)
Gh1b_d_p_cl = feedback(Gh1b_d_p_ol, 1);

% Pos controller
Gh1b_d_p = Gh1b_d_p_cl * c2d(tf([1], [1 0]), Ts, 'tustin');
[wc2 h1kp ok] = findp_discrete(Gh1b_d_p, h1gm, w)
% dB = 0 is already a really good crossover frequency
Gh1b_d_p_c_ol = h1kp * Gh1b_d_p;
bode(Gh1bv_d, Gh1b_d_p_ol, Gh1b_d_p_c_ol)
grid on
legend('Tf', 'Tf + Vel Controller', 'Tf + Vel & Pos Controller')
s = feedback(Gh1b_d_p_c_ol,1);

%% Plots
hold off
bode(c2d(Gxv,Ts,'zoh'),c2d(Gx,Ts,'zoh'), c2d(Gx*xkp,Ts,'zoh'))
grid on
legend('Tf', 'Tf + Vel Controller', 'Tf + Vel & Pos Controller')
step(feedback(Gx,1))
step(feedback(Gx*xkp,1))