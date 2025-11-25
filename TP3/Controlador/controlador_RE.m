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
Cd_ff = [0 0 1 0] % Solo quiero la variable posicion
kr = inv(Cd_ff*inv(eye(4)-(Ad-Bd*K))*Bd)

%% Acción integral
clc;
A_ext = [Ad , zeros(4,1);
    -Cd(2,:)*Ts, eye(1)]
B_ext = [Bd ; zeros(1,1)]
% p continuos observador:
%   -11.7857
%   -11.9976
%   -12.0971
%   -12.2231
p_C = -[7 2.2 8 2.5 7]

K = acker(A_ext, B_ext, exp(Ts*p_C))
L_o = [ 0.0003  0.0002;
   -0.4655  -0.0020;
   0.0001  0.3918;
   -0.0019  1.5908]
% display(K(1))
% display(K(2))
% display(K(3))
% display(K(4))
% display(K(5))
%% Exporto, abro, grafico
clc;
headers = {'ref', 'pos_o', 'vel_o', 'ang_o', 'vel_ang_o'};
vals_obs = [out.ref, out.pos_o, out.vel_o, out.ang_o, out.vel_ang_o];

% Guardar CSV con encabezados
% Obtener fecha-hora actual
dt = datetime('now','Format','yyMMddHHmmss'); 

% Pasarlo a string
dt_str = char(dt);   % convierte datetime a char

% Crear filename final
filename = ['Control_Integral' '_' dt_str '.csv'];
fid = fopen(filename, 'w');

% Escribir encabezados
fprintf(fid, '%s,', headers{1:end-1});
fprintf(fid, '%s\n', headers{end});
fclose(fid);

% Escribir datos
dlmwrite(filename, vals_obs, '-append');

% Abro el .csv
data = readtable(filename);

% Extraer variables desde la tabla
ref      = data.ref;
pos_o    = data.pos_o;
vel_o    = data.vel_o;
ang_o    = data.ang_o;
vel_ang_o = data.vel_ang_o;

% Crear vector de tiempo
n = height(data);          % cantidad de muestras
t = Ts * (0:n-1);          

% Graficos
close all;

figure;
stairs(t, ref, '--', 'LineWidth', 1.4); hold on;
stairs(t, pos_o, 'LineWidth', 1.4);
xlabel('Tiempo [s]'); ylabel('Posición [m]');
legend('Referencia', 'Posición observada'); grid on;
saveas(gcf, 'pid-re-pos.eps', 'epsc');

% figure;
% plot(t, vel_o, 'LineWidth', 1.4);
% xlabel('Tiempo [s]'); ylabel('Velocidad [m/s]');
% legend('Velocidad', 'Velocidad Estimada'); grid on;
% saveas(gcf, 'pid-re-vel.eps', 'epsc');
% 
% figure;
% plot(t, ang_o, 'LineWidth', 1.4);
% xlabel('Tiempo [s]'); ylabel('Ángulo [°]');
% legend('Ángulo Observado'); grid on;
% saveas(gcf, 'pid-re-theta.eps', 'epsc');
% 
% figure;
% plot(t, vel_ang_o, 'LineWidth', 1.4);
% xlabel('Tiempo [s]'); ylabel('Velocidad Angular [°/s]');
% legend('Velocidad Angular Observada'); grid on;
% saveas(gcf, 'pid-re-omega.eps', 'epsc');