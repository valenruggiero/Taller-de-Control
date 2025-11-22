clear all; close all; clc;

A = [0 1 0 0; -137.6 -21.4 0 0; 0 0 0 1; -0.15 0 0 -1.75];
B = [0; 53.64; 0; 0];
C = [1 0 0 0; 0 0 1 0];
D = 0;

Ts = 20e-3;
Ad = eye(4) + A*Ts;
Bd = B*Ts;
Cd = C;
Dd = D;

L = [ .6371 0.0; -3.4543 0.0; 0.0 1.0301; -0.003 11.1285 ];

polos_obs_d = eig(Ad - L*Cd)
polos_obs_c = log(polos_obs_d)/Ts

K = [0.2533,   -0.0121, -116.5175,  -30]

polos_cont_d = eig(Ad - Bd*K)
polos_cont_c = log(polos_cont_d)/Ts

G = ss(Ad, Bd, Cd, Dd, Ts);
C = reg(G, K, L);

Tcl = feedback(G, C, '+')
Tcl = ss(Tcl.A, Tcl.B, [Tcl.C; 0 1 0 0 0 0 0 0; 0 0 0 1 0 0 0 0], [Tcl.D; zeros(2, 1)], Ts)

data = dlmread('obs_ini_right.csv');

start = 1

theta = data(start:end, 1)
omega = data(start:end, 2)
pos = data(start:end, 3)
vel = data(start:end, 4)

t_max = Ts*(numel(theta)-1);

[y t] = initial(Tcl, [0 0 0.14 0 0 0 0 0], t_max);

figure; stairs(t, y(:, 1), '-.'); hold on;
stairs(t, theta);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰]');
xlim([0 t_max]);
legend('Ángulo simulado', 'Ángulo observado');
grid;
print -depsc ../informe/img/noff-theta.eps

figure; stairs(t, y(:, 2), '-.'); hold on;
stairs(t, pos);
xlabel('Tiempo [s]');
ylabel('Posición [m]');
xlim([0 t_max]);
legend('Posición simulada', 'Posición observada');
grid;
print -depsc ../informe/img/noff-pos.eps

figure; stairs(t, y(:, 3), '-.'); hold on;
stairs(t, omega);
xlabel('Tiempo [s]');
ylabel('Velocidad angular [⁰/s]');
xlim([0 t_max]);
legend('Velocidad angular simulada', 'Velocidad angular observada');
grid;
print -depsc ../informe/img/noff-omega.eps

figure; stairs(t, y(:, 4), '-.'); hold on;
stairs(t, vel);
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
xlim([0 t_max]);
legend('Velocidad simulada', 'Velocidad observada');
grid;
print -depsc ../informe/img/noff-vel.eps
