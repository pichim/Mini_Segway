clc, clear all
addpath fcns\
%%

% 2 khz
% max_num_of_floats = 2000000 / (4 * (8 + 2) * 2000)

% openlager
file_id = fopen('LOG278.TXT');

num_of_floats = fread(file_id, 1, 'uint8')

data_raw = fread(file_id, 'single');
length(data_raw)

fclose(file_id);


%%

data_raw = data_raw(1:floor( length(data_raw)/num_of_floats ) * num_of_floats);

data.values = reshape(data_raw, [num_of_floats, length(data_raw)/num_of_floats]).';
            
data.time = cumsum(data.values(:,1)) * 1e-6;
data.time = data.time - data.time(1);

data.values = data.values(:,2:end);


%%


multp_fig_nr = 1;

% index
ind.rc    = 1:4;
ind.vel_M = 5:6;
ind.ang_M = 7:8;
ind.gyro  = 9:11;
ind.acc   = 12:14;
ind.rpy   = 15:17;
ind.voltage_M = 18:19;
ind.curr  = 20:21;
ind.rob_pos = 22:23;
ind.rob_vel = 24:25;
ind.rob_vel_inp = 26:27;
ind.rob_vel_sp  = 28:29;


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
plot(data.time, data.values(:,ind.vel_M) / (2*pi)), grid on
ylabel('Velocity (RPS)')
ax(3) = subplot(313);
plot(data.time, data.values(:,ind.ang_M) / (2*pi)), grid on
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
                 0 *cumtrapz(data.time, data.values(:,ind.gyro))] * 180/pi), grid on
ylabel('RPY (deg)')
ax(4) = subplot(224);
plot(data.time(1:end-1), diff(data.values(:,ind.rpy)) / Ts * 180/pi), grid on
ylabel('dRPY (deg)'), xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])
legend('dRoll', ...
       'dPitch', ...
       'dYaw')


figure(expand_multiple_figure_nr(5, multp_fig_nr))

ax(1) = subplot(211);
plot(data.time, data.values(:,ind.voltage_M)), grid on
ylabel('Voltage (V)')
ax(2) = subplot(212);
plot(data.time, data.values(:,ind.curr)), grid on
ylabel('Current (A)'), xlabel('Time (sec)')
legend('Motor 1', ...
    'Motor 2', ...
    'Location', 'best')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])


figure(expand_multiple_figure_nr(6, multp_fig_nr))

ax(1) = subplot(211);
plot(data.time, data.values(:,[ind.rob_vel_sp(1), ind.rob_vel(1)])), grid on
ylabel('Forward Speed (m/s)')
ax(2) = subplot(212);
plot(data.time, data.values(:,[ind.rob_vel_sp(2), ind.rob_vel(2)]) * 180/pi), grid on
ylabel('Turn Rate (deg/sec)'), xlabel('Time (sec)')
legend('Setpoint', ...
    'Actual', ...
    'Location', 'best')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])

