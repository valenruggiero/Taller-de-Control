clear all; close all; clc;5

s=tf('s')

#P = -8.046 / s / (s + 1.75) / (s^2 + 21.4*s + 137.6)

Am = [0 1 0 0; -137.6 -21.4 0 0; 0 0 0 1; -0.15 0 0 -1.75];
Bm = [0 53.64 0 0]';
Cm = [0 0 1 0];
Dm = 0;

P = ss(Am, Bm, Cm, Dm);

C = -150 - 30/s;

L = P*C;

figure;
bode(L);

T = L/(1+L);

data = dlmread('output20251019-123942.csv');

start = 73;

e = data(start:end, 1);
u = data(start:end, 2);
y = data(start:end, 3);

##y = y - y(1)

##y0 = y(1);

n = numel(e);
t = (0:n-1)/50;

##y_sim = initial(T, [0 0 y0 0 0 0 y0 0], t);
y_sim = -0.047*impulse(T, t);

figure;
plot(t,100*y, 'linewidth', 2, t,100*y_sim, '-.', 'linewidth', 2);
grid;
xlim([0 20]);
xlabel('Tiempo [s]');
ylabel('Posición [cm]');
legend('Posición real', 'Posición simulada');

u_sim = -0.047*impulse(C/(1+L), t);

figure;
plot(t,u, 'linewidth', 2, t,u_sim, '-.', 'linewidth', 2);
grid;
xlim([0 20]);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰]');
legend('Comando real', 'Comando simulado');


%% ========================


##data = dlmread('output20251019-124745.csv');
data = dlmread('output20251019-125351.csv');

start = 16;

e = data(start:end, 1);
u = data(start:end, 2);
y = data(start:end, 3);

##y = y - y(1)

n = numel(e);
t = (0:n-1)/50;

y_sim = 0.1*step(T, t);

figure; step(0.1*C/(1+L));

figure;
plot(t,100*y, 'linewidth', 2, t,100*y_sim, '-.', 'linewidth', 2);
grid;
xlim([0 32]);
xlabel('Tiempo [s]');
ylabel('Posición [cm]');
legend('Posición real', 'Posición simulada');

