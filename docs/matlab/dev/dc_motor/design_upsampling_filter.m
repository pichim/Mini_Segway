clc, clear variables
addpath ../../fcns/
%%

Ts = 200 * 1e-6;

fnyq = (1 / (1e-3*2))
fcut = 1.0 * fnyq

G = get_lowpass1(fcut, Ts);
% G = get_lowpass2(fcut, sqrt(3)/2, Ts);

figure(1)
step(G, 1/1e3), grid on

figure(2)
bode(G), grid on