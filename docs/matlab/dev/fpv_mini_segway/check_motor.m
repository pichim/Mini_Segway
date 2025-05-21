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

load data_motor_voltage_to_speed.mat

figure(95)
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

inp_neg = [-2.53; -4.27; -5.76]; % voltage
out_neg = [-0.69; -1.24; -1.68]; % RPS
M_neg = [inp_neg, ones(size(inp_neg))];
theta_neg = M_neg \ out_neg
theta_neg(1) * 60 * 12

inp_pos = [2.42; 4.23; 5.69]; % voltage
out_pos = [0.62; 1.23; 1.65]; % RPS
M_pos = [inp_pos, ones(size(inp_pos))];
theta_pos = M_pos \ out_pos
theta_pos(1) * 60 * 12

% theoretical value
606/35.03*12

% measrued motor constant applied to firmware