close all; clear all; clc;

n = 1*50;

##data = dlmread('output20251010-120728.csv')(1:n,:);
##data += dlmread('output20251010-121904.csv')(1:n,:);
##data += dlmread('output20251010-121922.csv')(1:n,:);
##data += dlmread('output20251010-121935.csv')(1:n,:);
##data += dlmread('output20251010-121953.csv')(1:n,:);

##data = dlmread('output20251010-123326.csv')(1:n,:);
##data += dlmread('output20251010-123345.csv')(1:n,:);
##data += dlmread('output20251010-123359.csv')(1:n,:);
##data += dlmread('output20251010-123413.csv')(1:n,:);
##data += dlmread('output20251010-123434.csv')(1:n,:);

data1 = dlmread('output20251010-124253.csv')(1:n,:);
data2 = dlmread('output20251010-124311.csv')(1:n,:);
data3 = dlmread('output20251010-124332.csv')(1:n,:);
data4 = dlmread('output20251010-124346.csv')(1:n,:);
data5 = dlmread('output20251010-124404.csv')(1:n,:);

##data1 = dlmread('output20251010-130803.csv')(1:n,:);
##data2 = dlmread('output20251010-130817.csv')(1:n,:);
##data3 = dlmread('output20251010-130828.csv')(1:n,:);
##data4 = dlmread('output20251010-130843.csv')(1:n,:);
##data5 = dlmread('output20251010-130900.csv')(1:n,:);

data1(:,2) -= data1(1,2);
data2(:,2) -= data2(1,2);
data3(:,2) -= data3(1,2);
data4(:,2) -= data4(1,2);
data5(:,2) -= data5(1,2);

data = data1 + data2 + data3 + data4 + data5;

ang = data(:,1)/5;
pos = data(:,2)/5/100;

u = ang(1:n);
y = pos(1:n);

##X = [y(2:n-1), y(1:n-2), u(1:n-2)];
##
##coefs = pinv(X)*y(3:n);
##
##alpha = coefs(1)
##beta = coefs(2)
##gamma = coefs(3)
##
##figure;
##plot(u); hold on;
##plot(y); hold on;
##plot(X*coefs);
##%plot(filter(gamma, [1, -alpha, -beta], u));
##legend('Ángulo','Posición real', 'Posición estimada');
##
##pd = roots([1, -alpha -beta])
##Kd = -gamma

t = (1:n)/50;

s = tf('s');

figure;
plot(t, u); hold;
plot(t, y);

legends = [];

##for k_carro = 0.01:0.02:0.1
##  for p_carro = -(0.5:0.5:4)
##    K = -0.3905*k_carro;
##    p = [0, -10.690+4.807j, -10.690-4.807j, p_carro];
##    H = zpk([], p, K*abs(p(2))^2*abs(p_carro));
##
##    y_est = lsim(H, u, t);
##
##    legends = [legends; ['p=' num2str(p_carro) ';k=' num2str(k_carro)]];
##
##    plot(t, y_est, 'linewidth', 2);
##  endfor
##endfor

##p_carro=1.3
##k_carro=0.03

##p_carro=2
##k_carro=0.01

p_carro=1.6
k_carro=-0.02

legends='';
K = 0.3905*k_carro;
p = [0, -10.690+4.807j, -10.690-4.807j, p_carro];
H = zpk([], p, K*abs(p(2))^2*abs(p_carro))
y_est = lsim(H, u, t);
plot(t, y_est);

legend(['u'; 'y'; legends]);
ylim([-0.15, 0.15])
grid;

figure;
step(H, 2);
title('Reconocimiento 1 (polo en 1.6)');
