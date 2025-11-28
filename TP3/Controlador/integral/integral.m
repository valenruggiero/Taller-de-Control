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

L = [ 0.0003, 0.0002; -0.4655, -0.0020; 0.0001, 0.3918; -0.0019, 1.5908]

polos_obs_d = eig(Ad - L*Cd)
polos_obs_c = log(polos_obs_d)/Ts

A_ext = [Ad , zeros(4,1);
    -Cd(2,:)*Ts, eye(1)]
B_ext = [Bd ; zeros(1,1)]

Kext = acker(A_ext, B_ext, exp(-Ts*[7 2.7 8 2.7 3]))
##Kext = [0.5649, 0.0023091, -205.82, -30]

K = Kext(1:4)
H = -Kext(5)

##K = [0.5649, 0.0023091, -205.82, -30]
##H = -149.88;

polos_cont_d = eig(A_ext - B_ext*Kext)
polos_cont_c = log(polos_cont_d)/Ts

G = ss(Ad, Bd, Cd, Dd, Ts);
C = reg(G, K, L);

Tcl = feedback(G, C, '+')
##Tcl = ss(Tcl.A, Tcl.B, [Tcl.C; 0 1 0 0 0 0 0 0; 0 0 0 1 0 0 0 0], [Tcl.D; zeros(2, 1)], Ts)

z = tf('z', Ts);
Cint = H*Ts*z/(z-1);

Tint = feedback(series(Cint, Tcl), 1, 1, 2)
Tint = ss(Tint.A, Tint.B, [Tint.C; 0 1 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0], [Tint.D; zeros(2, 1)], Ts)

data = dlmread('output20251127-193127.csv');

start = 1

ref = data(start:end, 1);
pos = data(start:end, 2);
vel = data(start:end, 5);
theta = data(start:end, 7);
omega = data(start:end, 9);

t_max = Ts*(numel(theta)-1);
##t_max = 25;
[y t] = lsim(Tint, ref);

figure; stairs(t, ref);

figure; stairs(t, y(:, 1), '-.'); hold on;
stairs(t, theta);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰]');
xlim([0 t_max]);
legend('Ángulo simulado', 'Ángulo observado');
grid;
print -depsc ../../informe/img/int-theta.eps

figure; stairs(t, y(:, 2), '-.'); hold on;
stairs(t, pos);
xlabel('Tiempo [s]');
ylabel('Posición [m]');
xlim([0 t_max]);
legend('Posición simulada', 'Posición observada');
grid;
print -depsc ../../informe/img/int-pos.eps

figure; stairs(t, y(:, 3), '-.'); hold on;
stairs(t, omega);
xlabel('Tiempo [s]');
ylabel('Velocidad angular [⁰/s]');
xlim([0 t_max]);
legend('Velocidad angular simulada', 'Velocidad angular observada');
grid;
print -depsc ../../informe/img/int-omega.eps

figure; stairs(t, y(:, 4), '-.'); hold on;
stairs(t, vel);
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
xlim([0 t_max]);
legend('Velocidad simulada', 'Velocidad observada');
grid;
print -depsc ../../informe/img/int-vel.eps
