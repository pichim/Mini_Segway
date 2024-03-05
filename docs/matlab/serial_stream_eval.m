clc, clear all
%%

port = 'COM5';
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
title( sprintf(['Mean dTime = %0.2f musec, ', ...
                'Std. dTime = %0.2f musec, ', ...
                'Median dTime = %0.2f musec'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (musec)')
ylim([0 1.2*max(diff(data.time * 1e6))])
xlim([0 data.time(end-1)])

figure(2)
plot(data.time, data.values), grid on
xlabel('Time (sec)')
xlim([0 data.time(end)])

%%

load data_01.mat % save data_01 data

Ts = 500 * 1e-6;

w0 = 8 * 2*pi;
D = sqrt(3)/2;
Gf = c2d(tf(w0^2, [1 2*D*w0 w0^2]), Ts, 'tustin');

data_f = filter(Gf.num{1}, Gf.den{1}, data.values);

figure(1)
plot(data.time, [data.values, data_f]), grid on
xlabel('Time (sec)')
xlim([0 data.time(end)])

