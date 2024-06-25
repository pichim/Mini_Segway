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

params = [theta dtheta phi dphi];
values = [pi 0 0 0];

An = subs(A, params, values);
Bn = subs(B, params, values);
Cn = subs(C, params, values);
Dn = subs(D, params, values);

params = [  g   r   b   l   m   Jy   mw   Jw   bw   bg];
values = [p.g p.r p.b p.l p.m p.Jy p.mw p.Jw p.bw p.bg];

An = double( subs(An, params, values) );
Bn = double( subs(Bn, params, values) );
Cn = double( subs(Cn, params, values) );
Dn = double( subs(Dn, params, values) );

% model for the simulation
sys = ss(An, Bn, Cn, Dn);
sys = ss2ss(sys, sys.c)


%%

% (dphi1, dphi2) -> (v, dpsi)
Cw2r = [p.r];
Cr2w = Cw2r^-1;

Cinp = 1;
Cout = 1;

% use this to get measured states
% Cinp = 2.0 * pi * Cw2r;
% Cout = Cinp^-1;

Ts = 1/10e3;
Gf = get_lowpass2(p.fcut, p.D, Ts);
Gfd = tf([1 -1], [Ts 0], Ts) * Gf;

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
Wq = [eye(1, 2); ...
      0, Cr2w];
Wq = [Wq, zeros(2); ...
      zeros(2), Wq];
Q = Wq.' * diag([0 0 0 1e1]) * Wq
Wr = Cr2w;
R = 1e0 * (Wr.' * Wr)
K = lqr(sys_lm_no_L, Q, R)

eig(sys)

dcgain(sys)

% K =
% 
%    -0.2271   -2.8075   -1.4741   -3.1623
% 
% 
% ans =
% 
%          0
%    12.4673
%    -0.8040
%   -13.2365
% 
% 
% ans =
% 
%          0
%   489.2583
%     2.8300
%        Inf













