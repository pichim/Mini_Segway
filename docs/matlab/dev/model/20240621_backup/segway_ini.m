clc, clear variables
addpath ../../fcns/

% literatur:
% - LagrangeCaseStudy_HS2019.pdf
% - Yoeko Mak bachelor thesis s1070819 - niet vertrouwelijk.pdf
% - 775672333-MIT.pdf

% notes:
% - Lagrange equations with absolute angle of body (theta) and absolute
%   angle of wheels (phi1, phi2)

% equasions from:
% segway_lagrange.m


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
% phi1 : absolute angle of right wheel
% phi2 : absolute angle of left  wheel

syms theta   phi1   phi2
syms dtheta  dphi1  dphi2
syms ddtheta ddphi1 ddphi2
syms Tm1 Tm2


% DGLtheta
% DGLphi1
% DGLphi2
J =  [[          m*l^2 + Jy,                                                                                                                                         (l*m*r*cos(theta))/2,                                                                                                                                         (l*m*r*cos(theta))/2]; ...
      [(l*m*r*cos(theta))/2, (4*Jw*b^2 + 2*Jx*r^2 + 2*Jz*r^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 2*l^2*m*r^2 - 2*Jx*r^2*cos(2*theta) + 2*Jz*r^2*cos(2*theta) - 2*l^2*m*r^2*cos(2*theta))/(4*b^2),                                                -(r^2*(2*Jx + 2*Jz - b^2*m + 2*l^2*m - 2*Jx*cos(2*theta) + 2*Jz*cos(2*theta) - 2*l^2*m*cos(2*theta)))/(4*b^2)]; ...
      [(l*m*r*cos(theta))/2,                                                -(r^2*(2*Jx + 2*Jz - b^2*m + 2*l^2*m - 2*Jx*cos(2*theta) + 2*Jz*cos(2*theta) - 2*l^2*m*cos(2*theta)))/(4*b^2), (4*Jw*b^2 + 2*Jx*r^2 + 2*Jz*r^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 2*l^2*m*r^2 - 2*Jx*r^2*cos(2*theta) + 2*Jz*r^2*cos(2*theta) - 2*l^2*m*r^2*cos(2*theta))/(4*b^2)]];

rhs = [(Jx*dphi1^2*r^2*sin(2*theta) + Jx*dphi2^2*r^2*sin(2*theta) - Jz*dphi1^2*r^2*sin(2*theta) - Jz*dphi2^2*r^2*sin(2*theta) + 2*b^2*g*l*m*sin(theta) - 2*Jx*dphi1*dphi2*r^2*sin(2*theta) + 2*Jz*dphi1*dphi2*r^2*sin(2*theta) + b^2*dphi1*dtheta*l*m*r*sin(theta) + b^2*dphi2*dtheta*l*m*r*sin(theta))/(2*b^2); ...
                                                                                   (dtheta*r*(2*Jx*dphi2*r*sin(2*theta) - 2*Jx*dphi1*r*sin(2*theta) + 2*Jz*dphi1*r*sin(2*theta) - 2*Jz*dphi2*r*sin(2*theta) + b^2*dtheta*l*m*sin(theta) - 2*dphi1*l^2*m*r*sin(2*theta) + 2*dphi2*l^2*m*r*sin(2*theta)))/(2*b^2); ...
                                                                                   (dtheta*r*(2*Jx*dphi1*r*sin(2*theta) - 2*Jx*dphi2*r*sin(2*theta) - 2*Jz*dphi1*r*sin(2*theta) + 2*Jz*dphi2*r*sin(2*theta) + b^2*dtheta*l*m*sin(theta) + 2*dphi1*l^2*m*r*sin(2*theta) - 2*dphi2*l^2*m*r*sin(2*theta)))/(2*b^2)];

% add torque and friction
rhs = rhs + [-(Tm1 - bw*(dphi1 - dtheta)) - (Tm2 - bw*(dphi2 - dtheta)); ...
               Tm1 - bw*(dphi1 - dtheta) - bg*dphi1; ...
               Tm2 - bw*(dphi2 - dtheta) - bg*dphi2];


% states
x = [dtheta dphi1 dphi2 theta phi1 phi2].';
u = [Tm1, Tm2].';

% dx/dt = f(x, u)
fx = [J^-1 * rhs; ...
     [dtheta; dphi1; dphi2]];
fx = simplify(fx);

% y = g(x, u)
gy = [dtheta, dphi1, dphi2, theta, phi1, phi2].';

% state space model
A = jacobian(fx, x);
B = jacobian(fx, u);
C = jacobian(gy, x);
D = jacobian(gy, u);


%% substitute parameters

p = get_segway_params();

params = [theta dtheta phi1 dphi1 phi2 dphi2];
values = [0 0 0 0 0 0];

A = subs(A, params, values);
B = subs(B, params, values);
C = subs(C, params, values);
D = subs(D, params, values);

params = [  g   r   b   l   m   Jx   Jy   Jz   mw   Jw   bw   bg];
values = [p.g p.r p.b p.l p.m p.Jx p.Jy p.Jz p.mw p.Jw p.bw p.bg];

A = double( subs(A, params, values) );
B = double( subs(B, params, values) );
C = double( subs(C, params, values) );
D = double( subs(D, params, values) );

% model for the simulation
sys = ss(A, B, C, D);
sys = ss2ss(sys, sys.c)


%%

% (dphi1, dphi2) -> (v, dpsi)
Cw2r = [p.r/2     p.r/2; ...
        p.r/p.b, -p.r/p.b];
Cr2w = Cw2r^-1;
% s = 2*pi;

Cinp = eye(2);
Cout = [zeros(2,1), eye(2)];

% Cinp = 2.0 * pi * Cw2r;
% Cout = [zeros(2,1), eye(2)] * [ 1 0 0;...
%                                -1 1 0; ...
%                                -1 0 1] * 1 / (2*pi) * [eye(1, 3); ...
%                                                        zeros(2,1), Cr2w];

Ts = 1/10e3;
fcut = 40.0;
D  = sqrt(3.0) / 2.0;
Gf = get_lowpass2(fcut, D, Ts);
Gfd = tf([1 -1], [Ts 0], Ts) * Gf;

x0 = zeros(6,1);
[aa, bb, cc, dd] = linmod('segway_linmod_no_L');
sys_lm_no_L = ss(aa, bb, cc, dd);
sys_lm_no_L = balreal(sys_lm_no_L);
sys_lm_no_L = ss2ss(sys_lm_no_L, sys_lm_no_L.C);


[aa, bb, cc, dd] = dlinmod('segway_linmod', Ts);
sys_lm = ss(aa, bb, cc, dd, Ts);
% [aa, bb, cc, dd] = linmod('segway_linmod');
% sys_lm = ss(aa, bb, cc, dd);
sys_lm = balreal(sys_lm);

figure(9)
bodemag(sys_lm_no_L, sys_lm, 2*pi*logspace(-3, log10(1/2/1e-3), 1e3)), grid on


%%

% controller
Wq = [eye(1, 3); ...
      zeros(2,1), Cr2w];
Wq = [Wq, zeros(3); ...
      zeros(3), Wq];
Q = Wq.' * diag([0 0 0 0 10 10]) * Wq
Wr = Cr2w;
R = 1e0 * (Wr.' * Wr)
K = lqr(sys_lm_no_L, Q, R)

Kx = K(:,1:4)
Ki = K(:,5:6)

x0 = [0, 0, 0, 10*pi/180, 0, 0].';


% Q =
% 
%    1.0e+04 *
% 
%          0         0         0         0         0         0
%          0         0         0         0         0         0
%          0         0         0         0         0         0
%          0         0         0         0         0         0
%          0         0         0         0    1.1613   -0.0000
%          0         0         0         0   -0.0000    0.0088
% 
% 
% R =
% 
%    1.0e+03 *
% 
%     1.1613   -0.0000
%    -0.0000    0.0088
% 
% 
% K =
% 
%    -0.2932   -3.0661    0.0000   -1.6653   -3.1623   -0.0000
%    -0.0000   -0.0000    0.0483   -0.0000   -0.0000    3.1623
% 
% 
% Kx =
% 
%    -0.2932   -3.0661    0.0000   -1.6653
%    -0.0000   -0.0000    0.0483   -0.0000
% 
% 
% Ki =
% 
%    -3.1623   -0.0000
%    -0.0000    3.1623