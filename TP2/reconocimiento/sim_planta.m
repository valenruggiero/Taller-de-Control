clear all; close all; clc;

s=tf('s')

P = -8.046 / s / (s + 1.75) / (s^2 + 21.4*s + 137.6)

data = dlmread('output20251018-193033.csv');
t=0:(numel(data(:,1))-1);
t=t/50;

y = lsim(P, data(:,1), t);

figure;
stairs(t, data(:,1), 'linewidth', 2); hold on;
stairs(t, data(:,2), '-.', 'linewidth', 2);
stairs(t, 100*y, ':', 'linewidth', 2);
legend('Entrada', 'Posición medida', 'Posición simulada')
xlim([0 0.8])
ylim([-30 30])
xlabel('Tiempo [s]')
ylabel('Ángulo [⁰] o posición [cm]')
grid

