clc, clear all
addpath ..\..\fcns\
%%

load data_no_upsampling_filter.mat

multp_fig_nr = 1;

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


%%

w0 = 40 * 2*pi;
D = sqrt(3)/2;
Gf = c2d(tf(w0^2, [1 2*D*w0 w0^2]), Ts, 'tustin');
% [bf, af] = besself(3, w0);
% Gf = c2d(tf(bf, af), Ts, 'tustin');

figure(expand_multiple_figure_nr(2, multp_fig_nr))
plot(data.time, data.values(:,ind.rc)), grid on, hold on
plot(data.time, filter(Gf.num{1}, Gf.den{1}, data.values(:,ind.rc(1:2)))), hold off
ylabel('RC Data'), xlabel('Time (sec)')
legend('Turn Rate', ...
    'Forward Speed', ...
    'Arming State', ...
    'Scaled Period', ...
    'Location', 'best')
xlim([0 data.time(end)])
ylim([-2 3])

load data_40Hz_upsampling_filter.mat

figure(expand_multiple_figure_nr(3, multp_fig_nr))
plot(data.time, data.values(:,ind.rc)), grid on
ylabel('RC Data'), xlabel('Time (sec)')
legend('Turn Rate', ...
    'Forward Speed', ...
    'Arming State', ...
    'Scaled Period', ...
    'Location', 'best')
xlim([0 data.time(end)])
ylim([-2 3])

return

%% expo

clc, clear all

alpha = 2.3;
% beta = 1.1;
scale = 1.0 / (exp(1)^alpha - 1.0);

x = (-1.0:1e-4:1.0).';
y = zeros(size(x));
for i = 1:length(x)
    x_abs  = abs(x(i));
    x_sign = sign(x(i));
    y(i) = scale * x_sign * (exp(x_abs)^alpha - 1.0);% * x_abs^beta;
end

figure(1)
subplot(311)
plot(x, y), grid on
ylabel('y')
subplot(312)
plot(x(2:end), diff(y) ./ diff(x), 'color', [0 0.5 0]), grid on
ylabel('\partialy/\partialx')
subplot(313)
plot(x(2:end-1), diff(diff(y) ./ diff(x)) ./ diff(x(2:end)), 'r'), grid on
ylabel('\partial^2y/\partial^2x'), xlabel('x')
