clc, clear all
%%

port = 'COM19';
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

% Access the data
data = serialStream.getData();

return


%%
%load data_05.mat  
%save data_05 data

% index
ind.rc = 1:4;
ind.vel_M = 5:6;
ind.ang_M = 7:8;
ind.gyro = 9:11;
ind.acc = 12:14;
ind.rpy = 15:17;

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
plot(data.time, data.values(:,ind.rc)), grid on
ylabel('RC Data'), xlabel('Time (sec)')
legend('Forward Speed', ...
    'Turn Rate', ...
    'Arming State', ...
    'Scaled Period', ...
    'Location', 'best')
xlim([0 data.time(end)])
ylim([-2 3])

figure(3)
ax(1) = subplot(211);
plot(data.time, data.values(:,ind.vel_M)), grid on
ylabel('Velocity (RPS)')
ax(2) = subplot(212);
plot(data.time, data.values(:,ind.ang_M)), grid on
ylabel('Rotation (ROT)'), xlabel('Time (sec)')
legend('Motor 1', ...
    'Motor 2', ...
    'Location', 'best')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])

figure(4)
ax(1) = subplot(221);
plot(data.time, data.values(:,ind.gyro) * 180/pi), grid on
ylabel('Gyro (deg/sec)')
legend('Gyro X', ...
       'Gyro Y', ...
       'Gyro Z')
ax(2) = subplot(222);
plot(data.time, data.values(:,ind.acc)), grid on
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


%%

% % % PpmIn crsf
% % load data_01.mat % save data_01 data
% % Ts = 500 * 1e-6;
% 
% % % SBus elrs
% % load data_02.mat % save data_01 data
% % Ts = 500 * 1e-6;
% 
% Ts = 1000 * 1e-6;
% 
% w0 = 40 * 2*pi;
% D = sqrt(3)/2;
% Gf = c2d(tf(w0^2, [1 2*D*w0 w0^2]), Ts, 'tustin');
% % [bf, af] = besself(3, w0);
% % Gf = c2d(tf(bf, af), Ts, 'tustin');
% 
% 
% data_f = filter(Gf.num{1}, Gf.den{1}, data.values(:,1:4));
% 
% figure(1)
% plot(data.time, [data.values(:,1:4), data_f]), grid on
% xlabel('Time (sec)')
% xlim([0 data.time(end)])
% ylim([-2 3])
%%

% % 150:1 Micro Metal Gearmotor HPCB 12V with Extended Motor Shaft
% % - seems like the is an offset due to friction, gain is 
% 
% voltage = [0.2, 0.4, 0.6, 0.8, 1.0].' * 9;
% rps = [0.35, 0.87, 1.39, 1.91, 2.50].';
% M = [voltage, ones(size(voltage))];
% x = M \ rps;
% 
% figure(99)
% plot(voltage, [rps, M*x], 'x-'), grid on
% 
% 220/12/60 % expected
% x(1)      % measured
% x(1) / (220/12/60)
%%

T_avg = 3.2;
gyro_avg = mean(data.values(data.time <= T_avg, ind.gyro));
ind_eval = data.time > T_avg;
ang = zeros(size(data.values(:,ind.gyro)));
ang(ind_eval,:) = cumtrapz(data.time(ind_eval), data.values(ind_eval,ind.gyro) - gyro_avg);
figure(99)
plot(data.time, ang * 180/pi), grid on

