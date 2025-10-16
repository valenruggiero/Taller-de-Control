clear all; close all; clc;

A = [0 1; -137.6 -21.4]
B = [0; 53.64]
C = [1 0]
D = 0

Ts = 20e-3;

Ad = eye(2) + A*Ts
Bd = B*Ts
Cd = C
Dd = D

polos = eig(A)

polos_obs = -5*abs(eig(A))
polos_obs_disc = exp(polos_obs*Ts)

L = acker(Ad', Cd', polos_obs_disc)'
