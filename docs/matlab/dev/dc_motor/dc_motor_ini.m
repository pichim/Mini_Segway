clc, clear variables
%%

Ts = 1e-3;

% 31:1
dc = db2mag(-3) * 2*pi
R  = 1.80 * 6;
km = 0.825 * ((450.0 / 12.0) * (2*pi/60))^-1
b  = (km/dc - km^2) / R
L  = 7.0e-3;
J  = 1.05e-4;


A = [[-R/L -km/L 0]; ...
     [km/J -b/J 0]; ...
     [0 1 0]];
B = [1/L; 0; 0];
C = diag([1 1/(2*pi) 1/(2*pi)]);
sys = ss(A, B, C, 0);
sysd = c2d(sys, Ts);


w0 = 2*pi * 20;
Gf = c2d(tf(w0^2, [1 2*w0 w0^2]), Ts, 'tustin');
Gd = tf([1 -1], [Ts 0], Ts);
Gfd = Gd * Gf;


k_gear = 31.25 / 78.125;
cntr_gains = k_gear * [4.2, 140, 0.0192];
Kp = cntr_gains(1);
Ki = cntr_gains(2);
Kd = cntr_gains(3);

Kp = 1.0 * Kp;
Ki = 1.0 * Ki;
Kd = 1.0 * Kd;

% const float tau_f = 1.0f / (2.0f * M_PIf * 30.0f);
tau_f = 1.0 / (2.0 * pi * 30.0);
% const float tau_ro = 1.0f / (2.0f * M_PIf * 0.5f / (2.0f * TS));
tau_ro = 1.0 / (2.0 * pi * 0.3 / (2.0 * Ts));

Cmn = pid(Kp, Ki, Kd, tau_f, Ts, ...
    'IFormula', 'BackwardEuler', ...
    'DFormula', 'Trapezoidal') * ...
    c2d(tf(1, [tau_ro 1]), Ts, 'tustin');

gear_ratio = 31.25;
kn = 450.0 / 12.0;
Kf = 60.0 / kn;
G_tau_f  = c2d(tf([1 0], [tau_f  1]), Ts, 'tustin');
G_tau_ro = c2d(tf(1, [tau_ro 1]), Ts, 'tustin');

