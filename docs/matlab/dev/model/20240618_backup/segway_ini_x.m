clc, clear variables

% files:
% - LagrangeCaseStudy_HS2019.pdf

% notes:
% - file to linearise the system based on segway_lagrange_x.m 
% - for setting parameters get_segway_params function is used, it is placed
% in parent folder to use same values in both models 

%% build nonlinear differential equations

syms R L M m Jr Jb g
syms theta   x
syms dtheta  dx
syms ddtheta ddx
syms Tm

% DGLtheta
% Jb*ddtheta + (L^2*ddtheta*m)/4 + (L*ddx*m*cos(theta))/2 - (L*g*m*sin(theta))/2

% DGLx
% ddx*(M + Jr/R^2) + (m*(- L*sin(theta)*dtheta^2 + 2*ddx + L*ddtheta*cos(theta)))/2

dgls(1) = [(Jb + 1/4*m*L^2)*ddtheta + 1/2*L*m*cos(theta)*ddx - 1/2*L*g*m*sin(theta) == -Tm];
dgls(2) = [(m + M + Jr/R^2) * ddx + 1/2*m*L*cos(theta)*ddtheta - 1/2*m*L*sin(theta)*dtheta^2 == Tm/R];

% solve so highest derivative is on the left
sol = solve(dgls, [ddtheta, ddx]);

%% linearise system

x_ = [theta dtheta dx x].';
u = Tm;

% dx_/dt = f(x_, i)
f = [dtheta; ...
     sol.ddtheta; ...
     sol.ddx; ...
     dx];

A = jacobian(f, x_);
B = jacobian(f, u);
%% substitute parameters
% path is added to use the same function to set parameters for model
addpath('../../segway_models')
p = get_segway_params;

params = [R   L   M   m   Jr   Jb   g   theta dtheta ];
values = [p.R p.L p.M p.m p.Jr p.Jb p.g 0     0      ];

A = double(subs(A, params, values));
B = double(subs(B, params, values));
sys = ss(A, B, eye(4), zeros(4, 1));

% similarity transform to relative measurement
% x is calculated to angle, linear velocity is to calculate to velocity
% (1/p.R)
T = [ 1,  0,  0,  0; ...
      0,  1,  0,  0; ...
      0, -1,  1/p.R,  0; ...
     -1,  0,  0,  1/p.R];

sys = ss(sys.A, sys.B, T*sys.C, T*sys.D);

% restore C = I
sys = ss2ss(sys, sys.c)

dcgain(sys)

% segway position is allowed to be arbitrary
sys = ss(sys.A(1:3,1:3), sys.B(1:3), eye(3), zeros(3, 1));
eig(sys)

figure(1)
pzmap(sys), grid off

% figure(2)
% bode(sys), grid on
%% simulation parameters

theta0 = 20 * pi/180;
Tsim = 3;


%% LQR - simple version, no observer, no roll-off, no additional dynamics

% extend with integrator
Ci = [0 0 1];
sys_i = ss([sys.a, zeros(3,1); Ci, 0], ...
           [sys.b; 0], ...
           [Ci, 0], ...
           0);

% controller
Ki = lqr(sys_i, diag([0 0 10 100]), 1e5)
ki = Ki(1,4);
K  = Ki(1,1:3);
sys_cl = ss([[sys.a-sys.b*K, -sys.b*ki]; [Ci, 0]], ...
             [zeros(3,1);-1], eye(3,4), 0);
eig(sys_cl)

time = (0:1e-3:Tsim).';
x0 = zeros(size(sys_cl.a(:,1))); x0(1) = theta0;
y = lsim(sys_cl, zeros(size(time)), time, x0);

figure(3)
subplot(311)
plot(time, y(:,1)*180/pi), grid on, ylabel('Angle Body (deg)')
subplot(312)
plot(time, y(:,2)*180/pi), grid on, ylabel('Velocity Body (deg/sec)')
subplot(313)
plot(time, y(:,3)*180/pi), grid on, grid on, ylabel('Velocity Wheel (deg)')
xlabel('Time (sec)')

