clc, clear all
addpath ..\..\fcns\
%%

load data_gpa_meas_M1_00.mat

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


Ts = mean(diff(data.time));

figure(expand_multiple_figure_nr(1, multp_fig_nr))
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
ax(1) = subplot(221);
plot(data.time, data.values(:,ind.gyro) * 180/pi), grid on
ylabel('Gyro (deg/sec)')
legend('Gyro X', ...
       'Gyro Y', ...
       'Gyro Z')
ax(2) = subplot(222);
plot(data.time, data.values(:,ind.acc) - 0*mean(data.values(:,ind.acc))), grid on
ylabel('Acc (m^2/sec)')
legend('Acc X', ...
       'Acc Y', ...
       'Acc Z')
ax(3) = subplot(223);
plot(data.time, [data.values(:,ind.rpy), ...
                 cumtrapz(data.time, data.values(:,ind.gyro))] * 180/pi), grid on
ylabel('RPY (deg)')
ax(4) = subplot(224);
plot(data.time(1:end-1), diff(data.values(:,ind.gyro)) * 180/pi), grid on
ylabel('dRPY (deg)'), xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])
legend('dRoll', ...
       'dPitch', ...
       'dYaw')


% frequency response estimation
Nest     = round(5.0 / Ts);
koverlap = 0.95;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);

inp = data.values(:,ind.voltage_M(1));
out = data.values(:,ind.vel_M(1));
[Gv1, Cv1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = data.values(:,ind.voltage_M(2));
out = data.values(:,ind.vel_M(2));
[Gv2, Cv2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = data.values(:,ind.voltage_M(1));
out = data.values(:,ind.ang_M(1));
[Ga1, Ca1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = data.values(:,ind.voltage_M(2));
out = data.values(:,ind.ang_M(2));
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


% data from section below
load data_gpa_meas_M1_00_P.mat


opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';


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

return

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


%%
clc, clear variables

% 31:1
data = readmatrix('data_gpa_meas_M1_00.txt', 'NumHeaderLines', 88); % 31:1

Ts = 5e-4;


k_gear = 31.25 / 78.125;
cntr_gains = k_gear * [4.2, 140, 0.0192];
Kp = cntr_gains(1);
Ki = cntr_gains(2);
Kd = cntr_gains(3);

% const float tau_f = 1.0f / (2.0f * M_PIf * 30.0f);
tau_f = 1.0 / (2.0 * pi * 30.0);
% const float tau_ro = 1.0f / (2.0f * M_PIf * 0.5f / (2.0f * TS));
tau_ro = 1.0 / (2.0 * pi * 0.5 / (2.0 * Ts));

Cm = pid(Kp, Ki, Kd, tau_f, Ts, ...
    'IFormula', 'BackwardEuler', ...
    'DFormula', 'Trapezoidal') * ...
    c2d(tf(1, [tau_ro 1]), Ts, 'tustin');

% velocity filter
w0 = 2.0 * pi * 20.0;
D = 1.0;
K = 1;

k0 = Ts * Ts * w0 * w0;
k1 = 4.0 * D * Ts * w0;
k2 = k0 + k1 + 4.0;
b0 = K * k0 / k2;
b1 = 2.0 * K * k0 / k2;
b2 = b0;
a1 = (2.0 * k0 - 8.0) / k2;
a2 = (k0 - k1 + 4.0) / k2;
Gfm = tf([b0 b1 b2], [1 a1 a2], Ts);


U = data(:,2) + 1i*data(:,3);
Y = data(:,4) + 1i*data(:,5);
R = data(:,6) + 1i*data(:,7);
P = frd(Y./U, data(:,1), Ts, 'Units', 'Hz');
T = frd(Y./R, data(:,1), Ts, 'Units', 'Hz');
S = 1 - T;
C = T/S/P;
SC = S*C;
SP = S*P;
freq = data(:,1);
spec = abs([U Y R]);

% -- new tune -------------------------------------------------------------

Kp = 1.2 * Kp;
Ki = 1.1 * Ki;
Kd = 1.1 * Kd;

% const float tau_f = 1.0f / (2.0f * M_PIf * 30.0f);
tau_f = 1.0 / (2.0 * pi * 30.0);
% const float tau_ro = 1.0f / (2.0f * M_PIf * 0.5f / (2.0f * TS));
tau_ro = 1.0 / (2.0 * pi * 0.4 / (2.0 * Ts));

Cmn = pid(Kp, Ki, Kd, tau_f, Ts, ...
    'IFormula', 'BackwardEuler', ...
    'DFormula', 'Trapezoidal') * ...
    c2d(tf(1, [tau_ro 1]), Ts, 'tustin');

% velocity filter
w0 = 2.0 * pi * 20.0;
D = sqrt(3) / 2;
K = 1;

k0 = Ts * Ts * w0 * w0;
k1 = 4.0 * D * Ts * w0;
k2 = k0 + k1 + 4.0;
b0 = K * k0 / k2;
b1 = 2.0 * K * k0 / k2;
b2 = b0;
a1 = (2.0 * k0 - 8.0) / k2;
a2 = (k0 - k1 + 4.0) / k2;
Gfmn = tf([b0 b1 b2], [1 a1 a2], Ts);


Sn = feedback(1, Cmn*P/Gfm*Gfmn);
Tn = 1 - Sn;

figure(1)
ax(1) = subplot(311);
semilogx(freq, spec(:,1), 'b.-'), grid on
title('Signal Spectras')
ax(2) = subplot(312);
semilogx(freq, spec(:,2), '.-', 'color', [0 0.5 0]), grid on
ax(3) = subplot(313);
semilogx(freq, spec(:,3), 'r.-'), grid on
linkaxes(ax, 'x'), clear ax
xlim([min(freq) 1/2/Ts])

opt = bodeoptions('cstprefs');

figure(2)
subplot(121)
bode(P, 'b.-', 2*pi*P.Frequency), grid on
title('Plant')
subplot(222)
opt.YLim = {db([1e-5 2]), [-180 180]};
bodemag(S, 'b.-', T, 'g.-', Sn, 'r.-', Tn, 'c.-', 2*pi*P.Frequency, opt), grid on
title('S, T')
subplot(224)
opt.YLim = {db([1e-1 1e1]), [-180 180]};
bodemag(S*Cm, 'b.-', Sn*Cmn, 'r.-', 2*pi*P.Frequency, opt), grid on
title('SC')

figure(3)
bode(C, 'b.-', Cm, 'g.-', Cmn, 'r.-', 2*pi*P.Frequency), grid on
title('Controller')

[time, step_resp]  = get_step_resp_from_frd(T , Ts);
[time, step_respn] = get_step_resp_from_frd(Tn, Ts);

figure(4)
subplot(121)
bode(T, Tn), grid on
title('Tracking')
subplot(122)
stairs(time, [step_resp, step_respn]), grid on
xlim([0 0.3])

