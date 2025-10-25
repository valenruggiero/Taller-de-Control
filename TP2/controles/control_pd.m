clear all; close all; clc;5

s=tf('s')

P = -8.046 / s / (s + 1.75) / (s^2 + 21.4*s + 137.6)

##Am = [0 1 0 0; -137.6 -21.4 0 0; 0 0 0 1; -0.15 0 0 -1.75];
##Bm = [0 53.64 0 0]';
##Cm = [0 0 1 0];
##Dm = 0;
##
##P = ss(Am, Bm, Cm, Dm);

C = -150 - 0.01*s;

L = minreal(tf(P*C));

figure;
bode(L);

T = minreal(tf(L/(1+L)));

figure; step(T);

data = dlmread('output20251025-105629.csv');

start = 35;

e = data(start:end, 1);
u = data(start:end, 2);
y = data(start:end, 3);

##y = y - y(1)

##y0 = y(1);

n = numel(e);
t = (0:n-1)/50;

##y_sim = initial(T, [0 0 y0 0 0 0 y0 0], t);

y_sim = -0.05*impulse(T, t);

figure;
stairs(t,100*y, 'linewidth', 2); hold on;
stairs(t,100*y_sim, '-.', 'linewidth', 2);
grid;
xlim([0 4]);
xlabel('Tiempo [s]');
ylabel('Posición [cm]');
legend('Posición real', 'Posición simulada');

u_sim = -0.05*impulse(minreal(tf(C/(s/10000+1)/(1+L))), t);

figure;
stairs(t,u, 'linewidth', 2); hold on
stairs(t,u_sim, '-.', 'linewidth', 2);
grid;
xlim([0 4]);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰]');
legend('Comando real', 'Comando simulado');

%% ========================


data = dlmread('output20251025-110635.csv');

start = 5;

e = data(start:end, 1);
u = data(start:end, 2);
y = data(start:end, 3);

y = y - y(1)

n = numel(e);
t = (0:n-1)/50;

y_sim = 0.1*step(T, t);

figure;
stairs(t,100*y, 'linewidth', 2); hold on
stairs(t,100*y_sim, '-.', 'linewidth', 2);
grid;
xlim([0 4]);
xlabel('Tiempo [s]');
ylabel('Posición [cm]');
legend('Posición real', 'Posición simulada');

