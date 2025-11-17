close all; clear all; clc

##data = dlmread('output20251110-133123.csv');
data = dlmread('output20251110-140753.csv');

u = data(:, 1);
x = data(:, 2);
x_hat = data(:, 3);
v = data(:, 4);
v_hat = data(:, 5);
theta = data(:, 6);
theta_hat = data(:, 7);
omega = data(:, 8);
omega_hat = data(:, 9);

Ts = 1/50;

n = numel(u)
t = (0:n-1) * Ts;

t_max = 4

figure;
stairs(t, u, 'linewidth', 1);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰]');
xlim([0 t_max]);
ylim([-25 25])
grid;
print -deps ../informe/img/obs-input.eps

figure;
stairs(t, theta, '-.', 'linewidth', 1); hold on;
stairs(t, theta_hat, 'linewidth', 1);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰]');
legend('Ángulo medido', 'Ángulo estimada');
xlim([0 t_max]);
grid;
print -deps ../informe/img/obs-theta.eps


figure;
stairs(t, omega, '-.', 'linewidth', 1); hold on;
stairs(t, omega_hat, 'linewidth', 1);
xlabel('Tiempo [s]');
ylabel('Velocidad angular [⁰/s]');
legend('Velocidad angular medida', 'Velocidad angular estimada');
xlim([0 t_max]);
grid;
print -deps ../informe/img/obs-omega.eps

figure;
stairs(t, x, '-.', 'linewidth', 1); hold on;
stairs(t, x_hat, 'linewidth', 1);
xlabel('Tiempo [s]');
ylabel('Posición [m]');
legend('Posición medida', 'Posición estimada');
xlim([0 t_max]);
grid;
print -deps ../informe/img/obs-pos.eps

figure;
stairs(t, v, '-.', 'linewidth', 1); hold on;
stairs(t, v_hat, 'linewidth', 1);
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
legend('Velocidad medida', 'Velocidad estimada');
xlim([0 t_max]);
ylim([-1.5 1.5]);
grid;
print -deps ../informe/img/obs-vel.eps


