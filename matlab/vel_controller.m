%% Stats:
% xvkp = 0.1698
% xvtd 0.4034
% xval = 0.2
% Gain margin -12 dB
% Phase margin -114

% target = 10.9
% xvkp = 0.2917
% xvtd 0.3468
% xval = 0.3
% Gain margin -12 dB
% Phase margin -114
%% Initialize
xvkp = 1;
xvti = 0;
xval = 0.3;
%xvNi = 4;
Gxv_d = c2d(Gxv, Ts, 'zoh');
%% Find target
vc_d = c2d(tf([1 1], [1*xval 1]),Ts, 'tustin');
bode(Gxv_d, vc_d)
%% Set target
target = 10.9;
xvtd = 1 / (sqrt(h1al) * target)
vc_d = c2d(tf([xvtd 1], [xvtd*xval 1]),Ts, 'tustin');
Gxv_d_c_ol = Gxv_d * vc_d;
bode(Gxv_d_c_ol)
%xvti = h1Ni * (1/ target)

%% Find gain aka. How far is target from 0.
xvkp = 10^(-10.7/20)
Gxv_d_c_ol = Gxv_d * vc_d * xvkp;
bode(Gxv_d_c_ol);
Gxv_d_c_cl = (Gxv_d * xvkp) / (1 + vc_d * Gxv_d * xvkp);
Gxv_d_c_cl = feedback((vc_d * Gxv_d * xvkp), 1)
step(Gxv_d_c_cl)
