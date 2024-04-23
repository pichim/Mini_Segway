clc, clear variables
%%

% load data_00.mat
load data_01.mat

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

figure(2)
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


% rotating filter
Dlp = sqrt(3) / 2;
wlp = 2 * pi * 10;
Glp = c2d(tf(wlp^2, [1 2*Dlp*wlp wlp^2]), Ts, 'tustin');

% frequency response estimation
Nest     = round(5.0 / Ts);
koverlap = 0.9;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);


inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.vel_M(1)));
[G1, C1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.vel_M(2)));
[G2, C2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);


figure(3)
bode(G1, G2, 2*pi*G1.Frequency(G1.Frequency < 1/2/Ts)), grid on

opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

figure(4)
bodemag(C1, C2, 2*pi*G1.Frequency(G1.Frequency < 1/2/Ts), opt), grid on
set(gca, 'YScale', 'linear')

