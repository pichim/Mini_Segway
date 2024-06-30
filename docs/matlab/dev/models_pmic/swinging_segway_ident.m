clc, clear variables
addpath ../fcns/
%%

load swinging_segway_00.mat

theta = data.values(:,ind.rpy(1));
time = data.time;

T_eval = [20.5171, 35.0];
ind_eval = time >= T_eval(1) & time <= T_eval(2);
theta = theta(ind_eval) + pi - 0.0041;

theta = unwrap(theta);

time = time(ind_eval);
time = time - time(1);

[X, X_w, freq] = calc_fft(theta, Ts);

figure(6)
subplot(211)
plot(time, theta * 180/pi), grid on
subplot(212)
plot(freq, abs([X, X_w])), grid on
xlim([0 10])


t1 = 3.14404;
t2 = 5.69403;
fper = 1 / (t2 - t1) * 6
Tper = 1 / fper;
d1  = 1.07081;
d13 = 0.271664;
theta = 1/12 * log(d1 / d13); % 1/n = 1/12 = 1/(13-1)
D = 1 / sqrt(1 + pi^2/theta^2)
f0 = 1 / (Tper * sqrt(1 - D^2))
w0 = 2*pi*f0

m = 0.78 + 2*0.095;
g = 9.81;
l = 0.0340;

J = (m*g*l / w0^2 - m*l^2) % 3.5697e-04
b = 2*D*w0*J               % 3.8402e-04