Gs = tf(linsys1);
%Gs = tf([8.397e-06], [1, 0.01, 0]); % Height
w = logspace(-2, 2, 3000);
[mag, phase] = bode(Gs,w);
%alpha = 0.1;
%Ni = 4;
%gammaM = 60;
alpha = 0.075;
Ni = 9;
gammaM = 60;
phi_d = rad2deg(asin((1-alpha)/(1+alpha)));
phi_i = rad2deg(atan2(-1, Ni));
pc = gammaM - 180 - phi_d - phi_i;
n = find(phase > pc, 1, 'last');
wc = w(n);
tau_d = 1/(sqrt(alpha) * wc)
tau_i = Ni/wc
Gd = tf([tau_d 1], [alpha*tau_d 1]);
Gi = tf([tau_i, 1], [tau_i 0]);
[magc, phasec] = bode(Gs*Gi*Gd, wc);
kp = 1/magc
kp_r = kp;
tau_i_r = tau_i;
tau_d_r = tau_i;
figure(100)
margin(kp*Gi*Gd*Gs)
grid on