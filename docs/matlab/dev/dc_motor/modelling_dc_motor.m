clc, clear all
addpath ..\..\fcns\
%%

load data_31_1bat.mat
% load data_31_1bat_wheel.mat
% load data_31_2bat.mat
% load data_31_2bat_wheel.mat

multp_fig_nr = 1;

% index
ind.rc = 1:4;
ind.vel_M = 5:6;
ind.ang_M = 7:8;
ind.gyro = 9:11;
ind.acc = 12:14;
ind.rpy = 15:17;
ind.voltage_M = 18:19;
ind.sinarg = 20; % might be temporary
ind.current = 21:22;


Ts = mean(diff(data.time));

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(diff(data.time * 1e6))])


figure(expand_multiple_figure_nr(2, multp_fig_nr))
ax(1) = subplot(311);
plot(data.time, data.values(:,ind.voltage_M)), grid on
ylabel('Voltage (V)')
ax(2) = subplot(312);
plot(data.time, data.values(:,ind.vel_M)), grid on
ylabel('Velocity (RPS)')
ax(3) = subplot(313);
plot(data.time, data.values(:,ind.ang_M)), grid on
ylabel('Rotation (ROT)'), xlabel('Time (sec)')
legend('Motor 1', ...
    'Motor 2', ...
    'Location', 'best')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])


figure(expand_multiple_figure_nr(3, multp_fig_nr))
plot(data.time, data.values(:,ind.current)), grid on
ylabel('Current'), xlabel('Time (sec)')
xlim([0 data.time(end)])


% rotating filter
Dlp = sqrt(3) / 2;
wlp = 2 * pi * 10;
Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), Ts, 'tustin');

% frequency response estimation
Nest     = round(5.0 / Ts);
koverlap = 0.95;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.current(1)));
[Gi1, Ci1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.current(2)));
[Gi2, Ci2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.vel_M(1)));
[Gv1, Cv1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.vel_M(2)));
[Gv2, Cv2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.ang_M(1)));
[Ga1, Ca1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.ang_M(2)));
[Ga2, Ca2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);


%%

% 31:1
dc = db2mag(-3) * 2*pi
R  = 1.80 * 6;
km = 0.825 * ((450.0 / 12.0) * (2*pi/60))^-1
b  = (km/dc - km^2) / R
L  = 7.0e-3;
J  = 1.05e-4;


A = [[-R/L -km/L 0]; ...
     [km/J -b/J 0]; ...
     [0 1 0]];
B = [1/L; 0; 0];
C = diag([1 1/(2*pi) 1/(2*pi)]);
sys = ss(A, B, C, 0);
sysd = c2d(sys, Ts);


w0 = 2*pi * 20;
Gf = c2d(tf(w0^2, [1 2*w0 w0^2]), Ts, 'tustin');
Gfd = tf([1 -1], [Ts 0], Ts) * Gf;


opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';


figure(expand_multiple_figure_nr(4, multp_fig_nr))
bode(Gi1, Gi2, sysd(1,1), 2*pi*Gi1.Frequency(Gi1.Frequency < 1/2/Ts)), grid on

figure(expand_multiple_figure_nr(5, multp_fig_nr))
bodemag(Ci1, Ci2, 2*pi*Gi1.Frequency(Gi1.Frequency < 1/2/Ts), opt), grid on
set(gca, 'YScale', 'linear')

figure(expand_multiple_figure_nr(6, multp_fig_nr))
bode(Gv1, Gv2, sysd(3,1) * Gfd, 2*pi*Gv1.Frequency(Gv1.Frequency < 1/2/Ts)), grid on

figure(expand_multiple_figure_nr(7, multp_fig_nr))
bodemag(Cv1, Cv2, 2*pi*Gv1.Frequency(Gv1.Frequency < 1/2/Ts), opt), grid on
set(gca, 'YScale', 'linear')

figure(expand_multiple_figure_nr(8, multp_fig_nr))
bode(Ga1, Ga2, sysd(3,1), 2*pi*Ga1.Frequency(Ga1.Frequency < 1/2/Ts)), grid on

figure(expand_multiple_figure_nr(9, multp_fig_nr))
bodemag(Ca1, Ca2, 2*pi*Gv1.Frequency(Ga1.Frequency < 1/2/Ts), opt), grid on
set(gca, 'YScale', 'linear')


%% dc gain

% clc, clear all
% 
% syms R L km b J
% A = [[-R/L, -km/L]; ...
%      [km/J -b/J]]
% B = [1/L; 0]
% C = diag([1 1])
% 
% dc = -C*A^-1*B
%  % b/(km^2 + R*b)
% % km/(km^2 + R*b)
% 
% syms dc_dphi
% f1 = km/(km^2 + R*b) == dc_dphi
% solve(f1, b)

