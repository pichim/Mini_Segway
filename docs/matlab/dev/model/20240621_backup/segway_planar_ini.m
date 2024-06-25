clc, clear variables

% literatur:
% - LagrangeCaseStudy_HS2019.pdf
% - Yoeko Mak bachelor thesis s1070819 - niet vertrouwelijk.pdf
% - 775672333-MIT.pdf

% notes:
% - Lagrange equations with absolute angle of body (theta) and absolute
%   angle of wheel phi

% equasions from:
% segway_planar_lagrange.m


%% build nonlinear differential equations

% parameters in SI units
% g : gravity
% r : radus of wheel
% b : distance between wheels
% l : distance from wheel axis to COG of body
% m : mass of body
% Jx, Jy, Jz: inertia body
% mw: mass of wheel
% Jw: inertia wheel
% bw: friction between wheel and body
% bg: friction between wheel and ground
syms g r b l m Jx Jy Jz mw Jw bw bg

% theta: absolute angle of body
% phi  : absolute angle of right wheel

syms theta   phi
syms dtheta  dphi
syms ddtheta ddphi
syms Tm


% DGLtheta
% DGLphi
J =  [[      m*l^2 + Jy,        l*m*r*cos(theta)]; ...
      [l*m*r*cos(theta), 2*Jw + m*r^2 + 2*mw*r^2]];

rhs = [         g*l*m*sin(theta); ...
       dtheta^2*l*m*r*sin(theta)];

% add torque and friction
rhs = rhs + 2 * [-(Tm - bw*(dphi - dtheta)); ...
                   Tm - bw*(dphi - dtheta) - bg*dphi];


% states
x = [dtheta dphi theta phi].';
u = [Tm].';

% dx/dt = f(x, u)
fx = [J^-1 * rhs; ...
     [dtheta; dphi]];
fx = simplify(fx);

% y = g(x, u)
gy = [dtheta, dphi-dtheta, theta, phi-theta].';

% state space model
A = jacobian(fx, x);
B = jacobian(fx, u);
C = jacobian(gy, x);
D = jacobian(gy, u);


%% substitute parameters

p = get_segway_params();

params = [  g   r   b   l   m   Jy   mw   Jw   bw   bg   theta dtheta phi dphi];
values = [p.g p.r p.b p.l p.m p.Jy p.mw p.Jw p.bw p.bg 0 0 0 0];

A = double( subs(A, params, values) );
B = double( subs(B, params, values) );
C = double( subs(C, params, values) );
D = double( subs(D, params, values) );

% model for the simulation
sys = ss(A, B, C, D);
sys = ss2ss(sys, sys.c)


%%

% (dphi1, dphi2) -> (v, dpsi)
Cw2r = [p.r];
Cr2w = Cw2r^-1;
s = 2*pi;

% Cw2r = 1;
% Cr2w = Cw2r^-1;
% s = 1.0;

x0 = zeros(4,1);
[aa, bb, cc, dd] = linmod('segway_planar_linmod_no_L');
sys_lm_no_L = ss(aa, bb, cc, dd);
sys_lm_no_L = minreal(sys_lm_no_L);
sys_lm_no_L = ss2ss(sys_lm_no_L, sys_lm_no_L.C);

[aa, bb, cc, dd] = linmod('segway_planar_linmod');
sys_lm = ss(aa, bb, cc, dd);
sys_lm = minreal(sys_lm);

figure(99)
bodemag(sys_lm_no_L, sys_lm, 2*pi*logspace(-3, log10(1/2/1e-3), 1e3)), grid on


% controller
Q = diag([0 0 0 1]);
R = 1e-2;
K = lqr(sys_lm_no_L, Q, R)

Kx = K(:,1:3)
Ki = K(:,4)

x0 = [0, 0, 10*pi/180, 0].';

Ts = 1/10e3;
fcut = 40.0;
D  = sqrt(3.0) / 2.0;
Gf = get_lowpass2(fcut, D, Ts);
Gfd = tf([1 -1], [Ts 0], Ts) * Gf;

eig(sys)

dcgain(sys)

% K =
% 
%    -3.2386   -6.1915  -13.7648  -10.0000
% 
% 
% Kx =
% 
%    -3.2386   -6.1915  -13.7648
% 
% 
% Ki =
% 
%   -10.0000
% 
% 
% ans =
% 
%          0
%     5.3376
%    -0.8042
%    -5.4838
% 
% 
% ans =
% 
%          0
%   489.2583
%     4.2450
%        Inf

% K =
% 
%    -0.8445   -6.1915   -3.5892  -10.0000
% 
% 
% Kx =
% 
%    -0.8445   -6.1915   -3.5892
% 
% 
% Ki =
% 
%   -10.0000

