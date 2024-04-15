clc, clear all
%%

port = 'COM7';
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

% load data_00.mat % save data_00 data

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
ylim([0 1.2*max(diff(data.time * 1e6))])
xlim([0 data.time(end-1)])

figure(2)
plot(data.time, data.values(:,1:4)), grid on
xlabel('Time (sec)')
xlim([0 data.time(end)])
ylim([-2 3])

figure(3)
ax(1) = subplot(311);
plot(data.time, data.values(:,[5 8])), grid on
ax(2) = subplot(312);
plot(data.time, data.values(:,[6 9])), grid on
ax(3) = subplot(313);
plot(data.time, data.values(:,[7 10])), grid on
xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])

figure(4)
ax(1) = subplot(221);
plot(data.time, data.values(:,11:13)), grid on
ax(2) = subplot(222);
plot(data.time, data.values(:,14:16)), grid on
ax(3) = subplot(223);
plot(data.time, data.values(:,17:19) * 180/pi), grid on
ax(4) = subplot(224);
plot(data.time(1:end-1), diff(data.values(:,17:19))), grid on
xlabel('Time (sec)')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])


%%

% % PpmIn crsf
% load data_01.mat % save data_01 data
% Ts = 500 * 1e-6;

% % SBus elrs
% load data_02.mat % save data_01 data
% Ts = 500 * 1e-6;

Ts = 1000 * 1e-6;

w0 = 40 * 2*pi;
D = sqrt(3)/2;
Gf = c2d(tf(w0^2, [1 2*D*w0 w0^2]), Ts, 'tustin');
% [bf, af] = besself(3, w0);
% Gf = c2d(tf(bf, af), Ts, 'tustin');


data_f = filter(Gf.num{1}, Gf.den{1}, data.values(:,1:4));

figure(1)
plot(data.time, [data.values(:,1:4), data_f]), grid on
xlabel('Time (sec)')
xlim([0 data.time(end)])
ylim([-2 3])


%%

% 150:1 Micro Metal Gearmotor HPCB 12V with Extended Motor Shaft
% - seems like the is an offset due to friction, gain is 

voltage = [0.2, 0.4, 0.6, 0.8, 1.0].' * 9;
rps = [0.35, 0.87, 1.39, 1.91, 2.50].';
M = [voltage, ones(size(voltage))];
x = M \ rps;

figure(99)
plot(voltage, [rps, M*x], 'x-'), grid on

220/12/60 % expected
x(1)      % measured
x(1) / (220/12/60)

