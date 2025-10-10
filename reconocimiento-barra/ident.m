clear all; close all; clc;

data = dlmread('output20251010-105924.csv');
u = data(:,1);
y = data(:,2)/100;

n = length(y);

X = [y(2:n-1), y(1:n-2), u(1:n-2)];

coefs = pinv(X)*y(3:n)
alpha = coefs(1);
beta = coefs(2);
gamma = coefs(3);

t = (1:n)/50;

figure;
plot(t, u); hold on;
plot(t, y);
plot(t, filter(gamma, [1, -alpha, -beta], u));
legend('Ángulos de ref','Datos medidos', 'Identificado');

Ts = 20e-3;
pd = roots([1, -alpha, -beta])
Kd = gamma

sigma = log(abs(pd(1)))/Ts
wd = arg(pd(1))/Ts
K = gamma / (1-alpha-beta)

p = sigma + j*[wd -wd]

s = tf('s');
P = zpk([], p, K*abs(p(1))^2)
y_sim = lsim(P, u, t);

figure;
plot(t, u); hold on;
plot(t, y);
plot(t, y_sim);
legend('Ángulos de ref','Datos medidos', 'Identificado');

