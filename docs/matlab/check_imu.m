clc, clear all
%%

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


%% move up segway from horizontal to standing position

load data_roll_from_0_to_90_deg.mat

Ts = mean(diff(data.time));

data.values(:,ind.gyro) = data.values(:,ind.gyro) - ...
    mean(data.values(data.time < 6, ind.gyro));

figure(95)
plot(data.time, data.values(:,ind.gyro) * 180/pi), grid on
ylabel('Gyro (deg/sec)')
legend('Gyro X', ...
       'Gyro Y', ...
       'Gyro Z')

figure(96)
plot(data.time, cumtrapz(data.time, ...
    data.values(:,ind.gyro)) * 180/pi), grid on

% scaling looks correct
