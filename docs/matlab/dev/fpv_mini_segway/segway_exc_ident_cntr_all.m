clc, clear variables
addpath ../../fcns/
%%

load segway_exc_00.mat

ind_eval = data.time > 6;
data.values = data.values(ind_eval,:);
data.time = data.time(ind_eval);
data.time = data.time - data.time(1);

% index
ind.rc    = 1:5;
ind.vel_M = 6:7;
ind.ang_M = 8:9;
ind.gyro  = 10:12;
ind.acc   = 13:15;
ind.rpy   = 16:18;
ind.voltage_M = 19:20;
ind.curr  = 21:22;
ind.rob_pos = 23:24;
ind.rob_vel = 25:26;
ind.rob_vel_inp = 27:28;
ind.rob_vel_sp  = 29:30;

ind.out = [ind.rob_pos(1), ...
    ind.rob_vel(1), ...
    ind.acc(1), ...
    ind.rpy(1), ...
    ind.gyro(1)];

Ts = 1e-3;

ind.inp = ind.rob_vel_inp(1);

% frequency response estimation
Nest     = round(5.0 / Ts);
koverlap = 0.95;
Noverlap = round(koverlap * Nest);
window   = hann(Nest);

inp = data.values(:,ind.inp);
out = data.values(:,ind.out(1));
[G1, C1] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);
out = data.values(:,ind.out(2));
[G2, C2] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);
out = data.values(:,ind.out(3));
[G3, C3] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);
out = data.values(:,ind.out(4));
[G4, C4] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);
out = data.values(:,ind.out(5));
[G5, C5] = estimate_frequency_response(inp, out, window, Noverlap, Nest, Ts);


response_data = zeros(5, 1, length(squeeze(G1.ResponseData)));
response_data(1,1,:) = squeeze(G1.ResponseData);
response_data(2,1,:) = squeeze(G2.ResponseData);
response_data(3,1,:) = squeeze(G3.ResponseData);
response_data(4,1,:) = squeeze(G4.ResponseData);
response_data(5,1,:) = squeeze(G5.ResponseData);

G = frd(response_data, G1.Frequency, Ts, 'Units', 'Hz');

opt = bodeoptions('cstprefs');
opt.MagUnits = 'abs';
opt.MagScale = 'linear';

f_bode = 1 / (2*Ts);

figure(22)
bode(G(1,1), G(2,1), G(3,1), 2*pi*G.Frequency(G.Frequency < f_bode)), grid on

figure(33)
bode(G(4,1), G(5,1), 2*pi*G.Frequency(G.Frequency < f_bode)), grid on

figure(44)
bodemag(C1, C2, C3, C4, C5, 2*pi*G.Frequency(G.Frequency < f_bode), opt), grid on
set(gca, 'YScale', 'linear')


%%

% const float u_p_pos = MINI_SEGWAY_CP_POS_KP * (robotVelSetpointIntegrator.apply(robot_vel_setpoint(0)) - robot_pos(0));
% const float u_p_vel = MINI_SEGWAY_CPD_VEL_KP * robot_vel(0);
% const float u_d_vel = MINI_SEGWAY_CPD_VEL_KD * acc_x_filtered;
% const float u_p_ang = MINI_SEGWAY_CPD_ANG_KP * imu_data.rpy(0);
% const float u_d_ang = MINI_SEGWAY_CPD_ANG_KD * gyro_theta_filtered;
% robot_vel_input(0) = -1.0f * (u_p_pos - (u_p_vel + u_d_vel + u_p_ang + u_d_ang));

% #define MINI_SEGWAY_FORWARD_CPD_ANG_KP 2.64f
% #define MINI_SEGWAY_FORWARD_CPD_ANG_KD 0.05f
% #define MINI_SEGWAY_FORWARD_CPD_ANG_D_FILTER_FREQUENCY_HZ 3.0f
% #define MINI_SEGWAY_FORWARD_CPD_VEL_KP 2.2f
% #define MINI_SEGWAY_FORWARD_CPD_VEL_KD 0.05f
% #define MINI_SEGWAY_FORWARD_CPD_VEL_D_FILTER_FREQUENCY_HZ 1.0f
% #define MINI_SEGWAY_FORWARD_CP_POS_KP 2.86f
% #define MINI_SEGWAY_TURN_CP_POS_KP 10.0f

Kp_ang = 2.64
Kd_ang = 0.05
Kp_vel = 2.2
Kd_vel = 0.05
Kp_pos = 2.86
K = [0 Kp_vel Kd_vel Kp_ang Kd_ang];
Gw = feedback(feedback(G, K) * Kp_pos, 1, 1, 1);

Kp_ang = 2.64
Kd_ang = 0.05
Kp_vel = 2.2
Kd_vel = 0.05
Kp_pos = 2.86
K = [0 Kp_vel Kd_vel Kp_ang Kd_ang];
Gw_new = feedback(feedback(G, K) * Kp_pos, 1, 1, 1);


figure(55)
bode(Gw(1,1), Gw(2,1), ...
     Gw_new(1,1), Gw_new(2,1), ...
     2*pi*G.Frequency(G.Frequency < f_bode)), grid on

figure(66)
bode(Gw(4,1), Gw(5,1), ...
     Gw_new(4,1), Gw_new(5,1), ...
     2*pi*G.Frequency(G.Frequency < f_bode)), grid on

