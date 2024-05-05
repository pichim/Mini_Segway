clc, clear all
addpath ..\..\fcns\
%%

% load data_IMU_robot.mat
% load data_IMU_robot_cap.mat
% load data_IMU_vice.mat
load data_IMU_vice_cap.mat

multp_fig_nr = 2;

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


% figure(2)
% plot(data.time, data.values(:,ind.rc)), grid on
% ylabel('RC Data'), xlabel('Time (sec)')
% legend('Forward Speed', ...
%     'Turn Rate', ...
%     'Arming State', ...
%     'Scaled Period', ...
%     'Location', 'best')
% xlim([0 data.time(end)])
% ylim([-2 3])


figure(3)
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


figure(expand_multiple_figure_nr(4, multp_fig_nr))
ax(1) = subplot(221);
plot(data.time, data.values(:,ind.gyro) * 180/pi), grid on
ylabel('Gyro (deg/sec)')
legend('Gyro X', ...
       'Gyro Y', ...
       'Gyro Z')
ax(2) = subplot(222);
plot(data.time, data.values(:,ind.acc) - mean(data.values(:,ind.acc))), grid on
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


% select data for spectras
data_for_spectras = data.values(:,[ind.gyro, ...
    ind.acc]);

Nest     = round(2.5 / Ts);
koverlap = 0.5;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);
[pxx, freq] = estimate_spectras(data_for_spectras, window, Noverlap, Nest, Ts);
spectras = sqrt(pxx); % power -> amplitude (dc needs to be scaled differently)

figure(expand_multiple_figure_nr(5, multp_fig_nr))
plot(freq, spectras(:,1:3) * 180/pi), grid on
set(gca, 'YScale', 'log')
title('Spectras'), ylabel('Gyro (deg/sec'), xlabel('Frequency (Hz)')

figure(expand_multiple_figure_nr(6, multp_fig_nr))
plot(freq, spectras(:,4:6)), grid on
set(gca, 'YScale', 'log')
title('Spectras'), ylabel('Acc (m^2/sec)'), xlabel('Frequency (Hz)')