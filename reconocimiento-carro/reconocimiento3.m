close all; clear all; clc;

n = 0.66*50;

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

##data1 = dlmread('output20251010-124253.csv')(1:n,:);
##data2 = dlmread('output20251010-124311.csv')(1:n,:);
##data3 = dlmread('output20251010-124332.csv')(1:n,:);
##data4 = dlmread('output20251010-124346.csv')(1:n,:);
##data5 = dlmread('output20251610-124404.csv')(1:n,:);

##data1 = dlmread('output20251010-130803.csv')(1:n,:);
##data2 = dlmread('output20251010-130817.csv')(1:n,:);
##data3 = dlmread('output20251010-130828.csv')(1:n,:);
##data4 = dlmread('output20251010-130843.csv')(1:n,:);
##data5 = dlmread('output20251010-130900.csv')(1:n,:);

data1 = dlmread('output20251016-181329.csv')(28:end,:);
data2 = dlmread('output20251016-181342.csv')(7:end,:);
data3 = dlmread('output20251016-181355.csv')(25:end,:);
data4 = dlmread('output20251016-181414.csv')(23:end,:);
data5 = dlmread('output20251016-181427.csv')(15:end,:);

data1 = data1(1:n,:);
data2 = data2(1:n,:);
data3 = data3(1:n,:);
data4 = data4(1:n,:);
data5 = data5(1:n,:);

data1(:,1) -= data1(1,1);
data2(:,1) -= data2(1,1);
data3(:,1) -= data3(1,1);
data4(:,1) -= data4(1,1);
data5(:,1) -= data5(1,1);

data1(:,2) -= data1(1,2);
data2(:,2) -= data2(1,2);
data3(:,2) -= data3(1,2);
data4(:,2) -= data4(1,2);
data5(:,2) -= data5(1,2);

figure;
plot(data1(:,1)); hold on;
plot(data2(:,1)); hold on;
plot(data3(:,1)); hold on;
plot(data4(:,1)); hold on;
plot(data5(:,1)); hold on;

figure;
plot(data1(:,2)); hold on;
plot(data2(:,2)); hold on;
plot(data3(:,2)); hold on;
plot(data4(:,2)); hold on;
plot(data5(:,2)); hold on;



data = data1 + data2 + data3 + data4 + data5;

ang = data(:,1)/5;
pos = data(:,2)/5/100;

u = ang(1:n);
y = pos(1:n);


Ts = 1/50;
t = (0:n-1)*Ts;

z = tf('z', Ts);

figure;
##plot(t, u); hold;
stairs(t, y); hold on;

p=1.5
k=-0.13

pd = 1/(1+p*Ts)
H = k*z^2*Ts^2*pd / (z-1) / (z - pd)

y_est = lsim(H, u, t);
stairs(t, y_est);

legend(['y'; 'y_estimado']);
ylim([0, 0.15])
grid;

s = tf('s');
p = (1.5 + 2)/2;
k = -(0.17 + 0.13)/2;
H = k/s/(s+p)

figure;
step(9.75*H, 2);
title('Reconocimiento final');

figure;
p=1.75
k=-0.15

stairs(t, u, 'linewidth', 2); hold;
stairs(t, 100*y, '-.', 'linewidth', 2); hold on;

pd = 1/(1+p*Ts)
H = k*z^2*Ts^2*pd / (z-1) / (z - pd)

y_est = lsim(H, u, t);
stairs(t, 100*y_est, ':', 'linewidth', 2);

legend(['Ángulo de la barra'; 'Posición medida'; 'Posición simulada']);
xlabel('Tiempo [s]');
ylabel('Ángulo [⁰] o posición [cm]');
xlim([0 0.58]);
grid;
