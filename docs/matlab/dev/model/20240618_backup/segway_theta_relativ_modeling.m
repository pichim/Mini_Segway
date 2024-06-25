%% Segway
% pmic, 03.07.2012
clear all

syms Jr Js M R L g y dy ddy x dx ddx ddy bd1 bd2 tau u

%% Herleitung der NLDGLen mit Lagrange (Maple)

% x = phi
% y = theta;

% absolute Messung der Winkelsignale
DGLx = (R^2*M + Jr)*ddx - M*R*L*sin(y)*dy^2 + M*R*L*cos(y)*ddy;
DGLy = M*R*ddx*L*cos(y) + (M*L^2 + Js)*ddy - M*g*L*sin(y);

% Nicht konservative Kräfte (Momente), u = M(t), bd1 = Rollreibung, bd2 = Motorreibung
DGLx = DGLx + bd1*dx + bd2*(dx - dy) - u;
DGLy = DGLy - bd2*(dx - dy) + u;

% % relative Messung der Winkelsignale
% DGLx = (R^2*M+Jr)*ddx + (R^2*M+M*R*L*cos(y)+Jr)*ddy - M*R*dy^2*L*sin(y);
% DGLy = (R^2*M + M*R*L*cos(y) + Jr)*ddx + (M*L^2+R^2*M+2*M*R*L*cos(y)+Jr+Js)*ddy...
%         - M*R*dy^2*L*sin(y)-M*g*L*sin(y);
% 
% % Nicht konservative Kräfte, bd1 = Rollreibung, bd2 = Motorreibung (relativ)
% DGLx = DGLx + bd1*(dx + dy) + bd2*dx - u;
% DGLy = DGLy + bd1*(dx + dy);

% Umformung nach den 2ten Ableitungen
DGL = solve([DGLx,DGLy], [ddx, ddy]);

% Relative und abolute Messung (Modellbildung) ergeben identische
% Ergebnisse

%% Berechnung des linearisierten (und idealisierten) Modells

% Bestimmung der Jacobi-Matrix
A_hat = jacobian([DGL.ddx;DGL.ddy],[x dx y dy]);
B_hat = jacobian([DGL.ddx;DGL.ddy],[u]);

% Linearisierung im Arbeitspunkt x^T = [0 0 0 0]
A_hat = subs(A_hat,[x dx y dy u],[0 0 0 0 0]);
B_hat = subs(B_hat,[x dx y dy u],[0 0 0 0 0]);

% Aufbau der Matrizen
A_hat = [[0 1 0 0];A_hat(1,1:end);[0 0 0 1];A_hat(2,1:end)];
B_hat = [0;B_hat(1);0;B_hat(2)];
  
% Von absoluten in relative (Sensor) Koordinaten mittels einer Ähnlichkeitstransformation
C_hat = [[1 0 -1 0];[0 1 0 -1];[0 0 1 0];[0 0 0 1]];
A = C_hat*A_hat*C_hat^-1;
B = C_hat*B_hat;

% A = A_hat;
% B = B_hat;

%% Parameter Sustitution

Jr_  = 2.5e-5;
Js_  = 0.0063;
M_   = 1.286;
R_   = 0.0715/2;
L_   = 0.0505;
g_   = 9.81;
bd1_ = 0.001;
bd2_ = 0.004;

A = double(subs(A,[Jr Js M R L g bd1 bd2],[Jr_ Js_ M_ R_ L_ g_ bd1_ bd2_]));
B = double(subs(B,[Jr Js M R L g bd1 bd2],[Jr_ Js_ M_ R_ L_ g_ bd1_ bd2_]));

sys = ss(A,B,eye(4),zeros(4,1));
eig(sys)

Jr  = Jr_;
Js  = Js_;
M   = M_;
R   = R_;
L   = L_;
g   = g_;
bd1 = bd1_;
bd2 = bd2_;

% [aa,bb,cc,dd] = linmod('segway_theta_relativ_nl_sim');
% sys_linmod = ss(aa,bb,cc,dd);
% sys_linmod = ss2ss(sys_linmod,sys_linmod.c)

%% Zustandsregelung

phi0 = 5*pi/180;
x0 = [0 0 phi0 0]';

K = lqr(sys,diag([1000 1 1 1]),1000)

% [z,p,k] = besself(4,5);
% K = place(sys.a,sys.b,p)

sysr = ss(sys.a - sys.b*K,sys.b,sys.c,sys.d);
sysr.b = sysr.b/dcgain(sysr(1));

u_sim = zeros(2e3,1);
t_sim = linspace(0,2,2e3);
[y] = lsim(sysr,u_sim,t_sim,x0);

u = -y*K';

figure(1),clf
subplot('311')
plot(t_sim,y(:,[1 3])),grid on
legend('x','y')
subplot('312')
plot(t_sim,y(:,[2 4])),grid on
legend('dx','dy')
subplot('313')
plot(t_sim,[u]),grid on

% sim('segway_theta_relativ_nl_sim')
