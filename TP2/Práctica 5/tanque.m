clear all; close all; clc
optionss=bodeoptions;
optionss.PhaseMatching='on';
optionss.PhaseMatchingValue=-180;
optionss.PhaseMatchingFreq=1;
optionss.Grid='on';

%% Planta
syms eta h real
g = sym('9.8');
d = sym('0.01065');
l1 = sym('0.1');
l2 = sym('0.4');
L = sym('0.9');
h0 = sym('0.45');
Qi = sym('8')/1000/60;
hdot = (Qi - d^2*sym(pi)/4*eta*sqrt(2*g*h))/(l1 + h/L*(l2-l1))^2;
y = h;

h0 = sym('0.45');
eta0 = subs( solve( hdot == 0, eta ), h, h0 );

A = double( subs( jacobian(hdot, h), [h eta], [h0 eta0] ) );
B = double( subs( jacobian(hdot, eta), [h eta], [h0 eta0] ) );
C = 1;
D = 0;
eta0 = double(eta0);
h0 = double(h0);
P = tf( ss(A, B, C, D) )

%% Controlador
s = tf('s');
C0 = -1/s*(s + 0.00237);
L0 = P*C0;

figure();bode(L0,optionss,{.0001,10000});
legend('L0');
set(findall(gcf,'type','line'),'linewidth',2);

%% Ajustar gain
C1 = db2mag(5)*C0;
L1 = P*C1;
figure();margin(L1,optionss);
legend('L1');
set(findall(gcf,'type','line'),'linewidth',2);
figure; step(feedback(L1,1));

[u, t] = step(C1/(1+L1));
figure; plot(t, 10e-2*u + eta0);
set(findall(gcf,'type','line'),'linewidth',2);
%% Discretizo controlador
syms s z T
s = (z-1)/T/z;
C = (-1.778*s - 0.004215)/s;
C_bw = simplify(subs(C, T, sym(1)));
vpa(expand(C_bw))

z = tf('z');
Cd = (1.778-1.7822149999999999999966693309261*z)/(z-1);