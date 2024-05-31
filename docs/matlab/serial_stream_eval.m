clc, clear all
addpath fcns\
%%

port = 'COM13';
baudrate = 2e6;

if (~exist('serialStream', 'var'))
    serialStream = SerialStream(port, baudrate);
else
    serialStream.reset();
end

serialStream.start()
while (serialStream.isBusy())
    pause(0.1);
end

% access the data
data = serialStream.getData();

return


%%

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
plot(data.time, data.values(:,ind.rc)), grid on
ylabel('RC Data'), xlabel('Time (sec)')
legend('Turn Rate', ...
    'Forward Speed', ...
    'Arming State', ...
    'Scaled Period', ...
    'Location', 'best')
xlim([0 data.time(end)])
ylim([-2 3])


figure(expand_multiple_figure_nr(3, multp_fig_nr))
ax(1) = subplot(311);
plot(data.time, data.values(:,ind.voltage_M)), grid on
ylabel('Voltage (V)')
ax(2) = subplot(312);
% plot(data.time, data.values(:,[ind.vel_sp_M, ind.vel_M])), grid on
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

