clear all; close all; clc;5

s=tf('s')

#P = -8.046 / s / (s + 1.75) / (s^2 + 21.4*s + 137.6)

Am = [0 1 0 0; -137.6 -21.4 0 0; 0 0 0 1; -0.15 0 0 -1.75];
Bm = [0 53.64 0 0]';
Cm = [0 0 1 0];
Dm = 0;

P = ss(Am, Bm, Cm, Dm);

C = -150;

L = P*C;

figure;
bode(L);

T = L/(1+L);

##data = dlmread('output20251018-202252.csv');
data = dlmread('output20251018-205622.csv');

start = 70;

e = data(start:end, 1);
u = data(start:end, 2);
y = data(start:end, 3);

y = y - y(1)

##y0 = y(1);

n = numel(e);
t = (0:n-1)/50;

##y_sim = initial(T, [0 0 y0 0 0 0 y0 0], t);
y_sim = -0.055*impulse(T, t);

figure;
plot(t,y, t,y_sim, t,u);


%% ========================


data = dlmread('output20251018-210646.csv');

start = 17;

e = data(start:end, 1);
u = data(start:end, 2);
y = data(start:end, 3);

y = y - y(1)

n = numel(e);
t = (0:n-1)/50;

y_sim = 0.1*step(T, t);

figure;
plot(t,y, t,y_sim, t,u);

