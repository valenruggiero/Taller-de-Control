close all; clc;
load("attemp6_ok.mat");
disp(out);
n = length(out.y);
%n = 200;

x = [out.y(2:n-1), out.y(1:n-2), out.u(1:n-2)];

alpha = pinv(x)*out.y(3:n)

%% grafico los angul
plot(out.u); hold on;
plot(out.y); hold on;
plot(filter(alpha(3), [1, -alpha(1), -alpha(2)], out.u));
legend('Ángulos de ref','Datos simulados', 'alpha*x');
%% calculo polos
Ts = 20e-3;
pd = roots([1, -alpha(1), -alpha(2)]);   % polos en z
Kd = alpha(3);                           % ganancia inicial

% Gz = zpk([], pd, Kd, Ts)                 % G(z) mínima-fase
% Gc_tustin = d2c(Gz, 'tustin')

%% chequeamos q los valores de alpha no sean cualquier cosa
close all;
gain_cont = Kd/(1 -alpha(1) -alpha(2))*abs(-6.96-6.55j)^2
t = linspace(0, n*20e-3, n);
P_calculada = zpk([], [-6.96+6.55j -6.96-6.55j], gain_cont);
figure; lsim(P_calculada, out.u, t); hold on;
% figure; plot(filter(alpha(3), [1, -alpha(1), -alpha(2)], out.u));
plot(t(3:n), x*alpha); hold on; plot(t(3:n), out.y(3:n));


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
