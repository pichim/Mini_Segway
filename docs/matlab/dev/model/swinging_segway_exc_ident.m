clc, clear variables
addpath ../fcns/
%%

load swinging_segway_exc_02.mat

% frequency response estimation
Nest     = round(20.0 / Ts);
koverlap = 0.95;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);

inp = data.values(:,ind.vel_sp_M(1));
out = data.values(:,ind.vel_M(1));
[Gv1, Cv1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = data.values(:,ind.vel_sp_M(2));
out = data.values(:,ind.vel_M(2));
[Gv2, Cv2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = data.values(:,ind.vel_sp_M(1));
out = unwrap( data.values(:,ind.rpy(1)) ) - pi;
[Ga, Ca] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

figure(1)
plot(data.time, [inp, out]), grid on


inp = data.values(:,ind.vel_sp_M(1));
out = data.values(:,ind.gyro(1));
[Gw, Cw] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);


opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

f_bode = 1 / (2*Ts);


figure(2)
bode(Gv1, Gv2, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode)), grid on

figure(3)
bodemag(Cv1, Cv2, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode), opt), grid on
set(gca, 'YScale', 'linear')

figure(4)
Gint = tf([Ts 0], [1 -1], Ts);
bode(Ga, Gw * Gint, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode)), grid on

figure(5)
bodemag(Ca, Cw, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode), opt), grid on
set(gca, 'YScale', 'linear')


[X, X_w, freq] = calc_fft(data.values(:,ind.gyro(1)), Ts);

figure(6)
plot(freq, abs([X, X_w])), grid on
xlim([0 10])