clc, clear variables
addpath ../../fcns/
%%

load segway_exc_00.mat

ind_eval = data.time > 6;
data.values = data.values(ind_eval,:);
data.time = data.time(ind_eval);
data.time = data.time - data.time(1);


% frequency response estimation
Nest     = round(5.0 / Ts);
koverlap = 0.95;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);


inp = -1.0 * (data.values(:,ind.ang_cntr_out(3)));
out = data.values(:,ind.rpy(1));
[G, C] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

figure(1)
plot(data.time, [inp, out]), grid on

opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

f_bode = 1 / (2*Ts);

figure(2)
bode(G, 2*pi*G.Frequency(G.Frequency < f_bode)), grid on

figure(3)
bodemag(C, 2*pi*G.Frequency(G.Frequency < f_bode), opt), grid on
set(gca, 'YScale', 'linear')


%%

% #define MINI_SEGWAY_CPD_ANG_KP 2.2f
% #define MINI_SEGWAY_CPD_ANG_KD 0.06f
% #define MINI_SEGWAY_CPD_ANG_FCUT_D 3.0f
Kp = 2.2;
Kd = 0.05;
% Kd = Kp /(2*pi*fzero) <-> fzero = Kp /(2*pi*Kd)
fcut_d = 3.0;

inp = data.values(:,ind.rob_coord_inp(1));
out = data.values(:,ind.rpy(1));
Ga = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

out = data.values(:,ind.gyro(1));
Gw = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);
% gyro_filtered
Gw = Gw * get_lowpass1(fcut_d, Ts);

Gint = tf([Ts 0], [1 -1], Ts);

figure(4)
bode(Ga, Gw * Gint, 2*pi*G.Frequency(G.Frequency < f_bode)), grid on

Gaw = Ga / Gw;

Gcl = -feedback(feedback(-Gw, Kd) * Gaw, Kp);

figure(5)
bode(G, Gcl, 2*pi*G.Frequency(G.Frequency < f_bode)), grid on



