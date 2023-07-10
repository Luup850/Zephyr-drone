syms K V omega omega_dot T R J i T_drag K_prop;

eqn = (K*(V-K*omega)*(1/R)-T)*(1/J) == omega_dot;

S = solve(eqn, omega);
N = solve(eqn,V);

eqn1 = K * i == T - T_drag;

B = solve(eqn1, i);

eqn2 = V == R * ((J * omega_dot + T_drag)/K) + K * omega;

solve(eqn2, omega_dot);

%% Test
eqn3 = omega_dot == (K*((V - K*omega)*(1/R) - T_drag))*(1/J);
out = solve(eqn3, omega_dot);
in = solve(eqn3, V);

simplify(out/in)