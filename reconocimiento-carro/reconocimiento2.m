close all; clear all; clc;

n = 15*50;

data = dlmread('output20251009-201734.csv');

ang = data(:,1);
pos = data(:,2)/100;

##n = numel(ang);

##u = ang(1:n);
##y = pos(1:n);

u = ang(5*50+1:5*50+n);
y = pos(5*50+1:5*50+n);
y = y - y(1);

y_vec = y(3:n)-y(1:n-2);
X = [y(1:n-2)-y(2:n-1), u(1:n-2)];

coefs = pinv(X)*y_vec;

alpha = coefs(1)
beta = coefs(2)

t = (1:n)/50;

figure;
plot(t, u); hold on;
plot(t, 100*y); hold on;
##plot(t(1:n-2), 100*(X*coefs+y(1:n-2)));
plot(t, 100*filter([0 0 beta], [1, alpha, -(1+alpha)], u));
legend('Ángulo','Posición real', 'Posición estimada');
grid

Ts = 20e-3;

pd = roots([1, alpha -(1+alpha)])
Kd = beta

rho = abs(pd)

sigma = log(rho)/Ts

K = beta/(2+alpha)/Ts
p = sigma

s = tf('s');
P = zpk([], p, K*abs(p(2)))
##P = zpk([], p, K)
y_sim = lsim(P, u, t);

figure;
plot(t, u, 'linewidth', 2); hold on;
plot(t, 100*y, '-.', 'linewidth', 2);
plot(t, 100*y_sim, ':', 'linewidth', 2);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰] o posición [cm]');
legend('Ángulo medido','Posición medida', 'Sistema identificado');
xlim([0 15])
grid;

%% Sistema completo

H = P*53.64/(s^2+21.4*s+137.6);

figure;
step(H, 2);
title('Reconocimiento 2 (polo en 40)');
