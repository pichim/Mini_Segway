clc, clear variables
%%

% load data_00.mat
% load data_01.mat
% load data_02.mat
% load data_31_2bat.mat

% I pushed the changes in Mini Segway on the new "dev" branch as we said 
% previously. I put the motor testing files in the  eval_measurments 
% folder. Files are named in such way:
%  - data_gearRatio_numberOfBatteries_optionally"Wheel"
% in Matlab folder there are measurements of the IMU in two situations:
%  - when it is put steady not on the robot: data_IMU_vice
%  - when it is put steady not on the robot with the cap: data_IMU_vice_cap
% - always 2 motors exept data_488_1bat_M2.mat

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

figure(3)
plot(data.time, data.values(:,ind.current)), grid on
ylabel('Current'), xlabel('Time (sec)')
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
[Gv1, Cv1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.vel_M(2)));
[Gv2, Cv2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);


figure(4)
bode(Gv1, Gv2, 2*pi*Gv1.Frequency(Gv1.Frequency < 1/2/Ts)), grid on

opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

figure(5)
bodemag(Cv1, Cv2, 2*pi*Gv1.Frequency(Gv1.Frequency < 1/2/Ts), opt), grid on
set(gca, 'YScale', 'linear')

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(1)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.current(1)));
[Gi1, Ci1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

inp = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.voltage_M(2)));
out = apply_rotfiltfilt(Glp, data.values(:,ind.sinarg), data.values(:,ind.current(2)));
[Gi2, Ci2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);

opt.MagUnits = 'db';
opt.MagScale = 'log';

figure(6)
bode(Gi1, Gi2, 2*pi*Gi1.Frequency(Gi1.Frequency < 1/2/Ts)), grid on

opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

figure(7)
bodemag(Ci1, Ci2, 2*pi*Gi1.Frequency(Gi1.Frequency < 1/2/Ts), opt), grid on
set(gca, 'YScale', 'linear')

