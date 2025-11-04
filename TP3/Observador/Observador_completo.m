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


%% Observador de todas las variables de estado
s = tf('s');
P = -8.046 / s / (s + 1.75) / (s^2 + 21.4*s + 137.6);
[z, p_P, k] = zpkdata(P);
p_max = max(abs(p_P{1}));
p_des = 5*[p_max 0.4*p_max 0.4*p_max p_max];
% Fran dijo algo, q se joda y se acuerde el
% Chequear observabilidad
A = [0 1 0 0;
-137.6 -21.4 0 0;
0 0 0 1;
-0.15 0 0 -1.75];
B = [0 ; 53.64; 0; 0];
C = [1 0 0 0;
    0 0 1 0];
D = 0;

Ts = 20e-3;
Ad = eye(4) + A*Ts;
Bd = B*Ts;
Cd = C;
Dd = D;
p_des_d = exp(-Ts*p_des);

L_o = place(Ad', Cd', p_des_d)'





