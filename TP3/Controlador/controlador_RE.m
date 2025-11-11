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
Ts = .02
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



%% Observador de todas las variables de estado
s = tf('s');
P = -8.046 / s / (s + 1.75) / (s^2 + 21.4*s + 137.6);
[z, p_P, k] = zpkdata(P);
p_max = max(abs(p_P{1}));
p_des = 5*[p_max 0.4*p_max 0.4*p_max p_max/5];
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

%% Controlador
% p_c = 0.5*[p_max p_max p_max p_max];
p_des = 5*[p_max p_max p_max p_max/10];
p_c = p_des/10;
p_des_c = exp(-Ts*p_c);
K = acker(Ad, Bd, p_des_c);
K(1)
K(2)
K(3)
K(4)

K(1)=2.05
K(2)=.436
K(3)=-85.8
K(4)=5.44
% K(4)=-500;
p_d = eig(Ad - Bd*K)
p_c_des = 1/Ts * log(p_d)
p_c_des(3)=-4
p_c_des(4)=-1
K = acker(Ad, Bd, exp(p_c_des*Ts))
K(1) = 6
K(3) = -350
K(4) = -5
p_d = eig(Ad - Bd*K)
p_c_des = 1/Ts * log(p_d)
p_c_des(3) = -abs(p_c_des(3));
p_c_des(4) = -abs(p_c_des(4));
p_c_des
exp(p_c_des*Ts)
K = acker(Ad, Bd, exp(p_c_des*Ts))
p_d = eig(Ad - Bd*K)
p_c_des = 1/Ts * log(p_d);

%%
clc;
K = [8, 0.5076, -350, -5]
p_d = eig(Ad - Bd*K)
p_C = log(p_d)/Ts
p_C(3) = -abs(p_C(3));
p_C(4) = -abs(p_C(4));
K = acker(Ad, Bd, exp(Ts*p_C));
eig(Ad-Bd*K)
%% PASAMOS EN LIMPIO
K=[0.2533,   -0.0121, -116.5175,  -30]
L_o = [.6371, 0.0; -3.4543, 0.0; 0.0, 1.0301; -0.003, 11.1285];
G = ss(Ad, Bd, Cd, Dd, Ts);
RSYS = reg(G, K, L_o)
vals_obs = [out.ang_o, out.vel_ang_o, out.pos_o, out.vel_o];
% dlmwrite('obs_der.csv', vals_obs);
% [y, t] = impulse(RSYS(:, 2));
[u t] = initial(RSYS(:,2), [0 0 0 -1], 5)
close all;
% figure, impulse(RSYS(:, 2));
figure;
y = lsim(G, u, t, [0 0 0 -1]);
stairs(t, y(:,2)), hold on;
figure;
plot(out.pos(250:end));
hold off;

%% FF
clc;
Cd_ff = [0 0 1 0]
kr = inv(Cd_ff*inv(eye(4)-(Ad-Bd*K))*Bd)

%% Acción integral
A_ext = [Ad , zeros(4,1);
    Cd(1,:)*Ts, eye(1)]
B_ext = [Bd ; zeros(1,1)]


%K = acker(A_ext, B_ext, exp(Ts*p_C));