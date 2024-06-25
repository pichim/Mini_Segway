clc, clear all
addpath ..\..\fcns\
%%

% https://www.pololu.com/category/115/25d-metal-gearmotors

% load measurements\25D_chirp\motors_D25_without_wheels_v3.mat
% load measurements\25D_chirp\motors_D25_big_wheels.mat
load measurements\25D_chirp\motors_D25_big_wheels_v2.mat
% load measurements\25D_chirp\motors_D25_big_wheels_v3.mat


multp_fig_nr = 1;

% index
ind.rc = 1:4;
ind.vel_M = 5:6;
ind.ang_M = 7:8;
ind.gyro = 9:11;
ind.acc = 12:14;
ind.rpy = 15:17;
ind.voltage_M = 18:19;
% ind.vel_sp_M = 20:21;
ind.curr = 20:21;
ind.curr_add = 22:23;
ind.sinarg = 24;


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
plot(data.time, data.values(:,ind.curr_add)), grid on
ylabel('Current'), xlabel('Time (sec)')
xlim([0 data.time(end)])


% rotating filter
Dlp = sqrt(3) / 2;
wlp = 2 * pi * 10;
Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), Ts, 'tustin');

% frequency response estimation
Nest     = round(10.0 / Ts);
koverlap = 0.95;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.curr_add(1)));
[Gi1, Ci1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.curr_add(2)));
[Gi2, Ci2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.vel_M(1)));
[Gv1, Cv1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.vel_M(2)));
[Gv2, Cv2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.ang_M(1)));
inp = diff(inp);
out = diff(out);
[Ga1, Ca1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.ang_M(2)));
inp = diff(inp);
out = diff(out);
[Ga2, Ca2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);


%%

% #4865 47:1 Metal Gearmotor 25Dx67L mm MP 12V with 48 CPR Encoder 
dc = db2mag(-10.8) * 2.0 * pi;
R  = 7.2;
km = 0.81 * ((170.0 / 12.0) * (2*pi/60))^-1;
b  = (km/dc - km^2) / R;
L  = 7.5e-3;
J  = 1.7e-3;

Rc = 1.8e3;
Cc = 0.48e-6;

w0 = 2.0*pi * 20;
D  = sqrt(3.0) / 2.0;
Gf = c2d(tf(w0^2, [1 2.0*D*w0 w0^2]), Ts, 'tustin');
Gfd = tf([1 -1], [Ts 0], Ts) * Gf;

Gfc = tf(1, [Rc*Cc 1]);

A = [[-R/L -km/L 0]; ...
     [km/J -b/J 0]; ...
     [0 1 0]];
B = [1/L; 0; 0];
C = diag([1 1/(2*pi) 1/(2*pi)]);
sys = ss(A, B, C, 0);
sysd = c2d(sys, Ts);

Gcurr = c2d(sys(1,1) * Gfc, Ts);

Gvel = sysd(3,1) * Gfd;

Gang = sysd(3,1);


opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

f_bode = 300;


figure(expand_multiple_figure_nr(4, multp_fig_nr))
bode(Gi1, Gi2, Gcurr, 2*pi*Gi1.Frequency(Gi1.Frequency < f_bode)), grid on

figure(expand_multiple_figure_nr(5, multp_fig_nr))
bodemag(Ci1, Ci2, 2*pi*Gi1.Frequency(Gi1.Frequency < f_bode), opt), grid on
set(gca, 'YScale', 'linear')

figure(expand_multiple_figure_nr(6, multp_fig_nr))
bode(Gv1, Gv2, Gvel, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode)), grid on

figure(expand_multiple_figure_nr(7, multp_fig_nr))
bodemag(Cv1, Cv2, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode), opt), grid on
set(gca, 'YScale', 'linear')

figure(expand_multiple_figure_nr(8, multp_fig_nr))
bode(Ga1, Ga2, Gang, 2*pi*Ga1.Frequency(Ga1.Frequency < f_bode)), grid on

figure(expand_multiple_figure_nr(9, multp_fig_nr))
bodemag(Ca1, Ca2, 2*pi*Gv1.Frequency(Ga1.Frequency < f_bode), opt), grid on
set(gca, 'YScale', 'linear')


%%

fcut = 40.0;
D  = sqrt(3.0) / 2.0;
% Gf = c2d(tf(w0^2, [1 2.0*D*w0 w0^2]), Ts, 'tustin');
Gf_2 = get_lowpass2(fcut, D, Ts);
Gfd_2 = tf([1 -1], [Ts 0], Ts) * Gf_2;

Gvel_2 = sysd(3,1) * Gfd_2;

Kp = 8.0;

Gv1_2 = Gv1 / Gf * Gf_2;
Gv2_2 = Gv2 / Gf * Gf_2;

figure(expand_multiple_figure_nr(10, multp_fig_nr))
bode(Gv1_2, Gv2_2, Gvel_2, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode)), grid on

figure(expand_multiple_figure_nr(11, multp_fig_nr))
margin(Kp * Gvel_2, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode)), grid on

Gw_vel_2 = feedback(Kp * Gvel_2, 1);
Gw_v1_2 = feedback(Kp * Gv1_2, 1);
Gw_v2_2 = feedback(Kp * Gv2_2, 1);

figure(expand_multiple_figure_nr(12, multp_fig_nr))
bode(Gw_v1_2, Gw_v2_2, Gw_vel_2, 2*pi*Gv1.Frequency(Gv1.Frequency < f_bode)), grid on

figure(expand_multiple_figure_nr(13, multp_fig_nr))
step(Gw_vel_2, 0.1), grid on


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

