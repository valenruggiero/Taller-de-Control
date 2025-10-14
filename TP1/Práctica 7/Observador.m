close all; clc;
load("attemp6_ok.mat");
disp(out);
n = length(out.y);
%n = 200;

x = [out.y(2:n-1), out.y(1:n-2), out.u(1:n-2)];

alpha = pinv(x)*out.y(3:n)

%% grafico los angulos
plot(out.u); hold on;
plot(out.y); hold on;
plot(filter(alpha(3), [1, -alpha(1), -alpha(2)], out.u));
legend('Ángulos de ref','Datos simulados', 'alpha*x');
%% calculo polos discretos
Ts = 20e-3;
pd = roots([1, -alpha(1), -alpha(2)]);   % polos en z
Kd = alpha(3);                           % ganancia inicial

% Gz = zpk([], pd, Kd, Ts)                 % G(z) mínima-fase
% Gc_tustin = d2c(Gz, 'tustin')

%% Calculo polos continuos
sigma = log(real(pd))/Ts;
wd    = imag(pd)/Ts;
pc = sigma + j*wd

%% chequeamos q los valores de alpha no sean cualquier cosa
close all;
gain_cont = Kd/(1 -alpha(1) -alpha(2))*(abs(pc(1))^2)
t = linspace(0, n*20e-3, n);
P_calculada = zpk([], [pc(1) pc(2)], gain_cont);
figure; lsim(P_calculada, out.u, t); hold on;
% figure; plot(filter(alpha(3), [1, -alpha(1), -alpha(2)], out.u));
plot(t(3:n), out.y(3:n), '-r'); hold on; plot(t(3:n), x*alpha, '--g'); hold on; 
legend('Planta continua calculada', 'alpha*x','Datos simulados');
xlabel('Tiempo[s]');
ylabel('Ángulo en grados');   
xlim([0 69]);
title('Ángulo medido por la IMU según ángulo comandado por el servo');
set(findall(gcf,'type','line'),'linewidth',1);
saveas(gcf, 'ID_P_ang.eps', 'eps');
hold off;

% Polos discretos resultantes, attemp viejo
% z1 = j6.68 + 4.14
% z2 = -j6.68 + 4.14
%% Coeficiente ang servo / ang ref
% n = 200;
t = linspace(0, n*20e-3, n);
plot(out.u); hold on;
plot(out.y);
ranges = [190 270;
          380 440;
          520 600;
          670 750;
          800 890;
          950 1050;
          1090 1190;
          1260 1320;
          1410 1490;
          1530 1620];

divs   = [ 30, -30, 20, -20, 15, -15, 10, -10, 5, -5];

y_vals = zeros(size(divs));   % resultado: [y_30, y_-30, y_20, ..., y_-5]

for k = 1:numel(divs)
    y_vals(k) = mean(out.y(ranges(k,1):ranges(k,2))) / divs(k);
end

coef = mean(y_vals)


%% Implementación observador
clc;
s = tf('s');
G = ss(P_calculada)
W_r = G.b;
if rank(W_r) == order(G)
    fprintf('Es controlableeee :)\n');
end
W_o = G.c;
if rank(W_o) == order(G)
    fprintf('Es observableeeee :)\n');
end

% Controlador
% K = acker(G.a, G.b, -1)
% kr = -1/G.c/inv(G.a-G.b*K)/G.b
% Observador
% p_des_c = 5*pc;
% p_des = exp(p_des_c*Ts)
% Ad = eye(2) + G.a*Ts;
% Bd = G.b*Ts;
% Cd = G.c;
% Dd = G.d;

A = [0 1; -137.6 -21.4];
B = [0; 53.64];
C = [1 0];
D = 0;

p_des = exp(-5*abs(eig(A))*Ts)

Ad = eye(2) + A*Ts
Bd = B*Ts
Cd = C
Dd = D

L_o = acker(Ad', Cd', p_des)'
% Sistema controlado
% G_controlada = reg(G, K, L_o)
% G_c = tf(G_controlada)

lambda = (s-G.a+L_o*G.c);