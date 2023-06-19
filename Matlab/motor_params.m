motorMass = 0.1870; % Kg
rotor_radius = 0.02; % meter
J = (1/2) * motorMass * rotor_radius^2;
R = 0.0830; % Motor resistance
K_motor = 0.0136; % Motor constant
K_prop = 1.5e-9;
s = tf([1 0],[1]);
%[5.76583e-05, 5.76583e-05, 3.74e-05]
tf = 