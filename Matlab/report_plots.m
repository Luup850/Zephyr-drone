G_s = tf([1 2], [1 1 2]);
Ts = 0.25;
G_zoh = c2d(G_s,Ts,'zoh');
G_tustin = c2d(G_s,Ts,'tustin');

hold off
stepplot(G_s)
hold on
stepplot(G_zoh)
stepplot(G_tustin)
legend('S-domain','Zero Order Hold', 'Billinear')

%% Yaw
hold off
bode(Gya, Gyaol)
legend('Tf','Tf + Positional Controller')

step(feedback(Gyaol, 1))