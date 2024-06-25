clc, clear variables

% files:
% - LagrangeCaseStudy_HS2019.pdf

% notes:
% - file to linearise the system based on segway_lagrange.m 
% - for setting parameters get_segway_params function is used, it is placed
%   in parent folder to use same values in both models 

% changes to original file from szar:
% - L is new half of the body length resp. distance from wheel axis to COG


%% build nonlinear differential equations

% R : radus of wheel
% L : distance from wheel axis to COG of body
% M : mass of wheel
% m : mass of body
% Jr: inertia wheel
% Jb: inertia body
% g : gravity
% b : friction motor
% br: friction between wheel and ground
% all in SI units

syms R L M m Jr Jb g b br

% theta: absolute angle of body
% phi1 : absolute angle of right wheel
% phi2 : absolute angle of left  wheel

syms theta   phi1   phi2
syms dtheta  dphi1  dphi2
syms ddtheta ddphi1 ddphi2
syms Tm1 Tm2

% DGLtheta
dgls(1) = (Jb + m*L^2)*ddtheta + m*R*L*cos(theta)*ddphi1 + m*R*L*cos(theta)*ddphi2 - m*L*g*sin(theta) == -(Tm1 - b*(dphi1 - dtheta)) - (Tm2 - b*(dphi2 - dtheta));

% DGLphi1
dgls(2) = m*R*L*cos(theta)*ddtheta + (Jr + (m + M)*R^2)*ddphi1 - m*R*L*sin(theta)*dtheta^2 == Tm1 - b*(dphi1 - dtheta) - br*dphi1;

% DGLphi2
dgls(3) = m*R*L*cos(theta)*ddtheta + (Jr + (m + M)*R^2)*ddphi2 - m*R*L*sin(theta)*dtheta^2 == Tm2 - b*(dphi2 - dtheta) - br*dphi2;

% dx = [ddtheta ddphi1 ddphi2]^T
J = [[ Jb + m*L^2      , m*R*L*cos(theta) , m*R*L*cos(theta) ]; ...
     [ m*R*L*cos(theta), Jr + (m + M)*R^2 , 0                ]; ...
     [ m*R*L*cos(theta), 0                , Jr + (m + M)*R^2 ]];

% states
x = [dtheta dphi1 dphi2 theta phi1 phi2].';
u = [Tm1, Tm2].';

% dx/dt = f(x, u)
f = [simplify(J^-1 * [-(Tm1 - b*(dphi1 - dtheta)) - (Tm2 - b*(dphi2 - dtheta)) + m*L*g*sin(theta); ...
                        Tm1 - b*(dphi1 - dtheta) - br*dphi1 + m*R*L*sin(theta)*dtheta^2; ...
                        Tm2 - b*(dphi2 - dtheta) - br*dphi2 + m*R*L*sin(theta)*dtheta^2]); ...
     dtheta; ...
     dphi1; ...
     dphi2];

A = jacobian(f, x);
B = jacobian(f, u);


%% substitute parameters
% path is added to use the same function to set parameters for model

p = get_segway_params_new();

params = [R   L   M   m   Jr   Jb   g   b   br   theta dtheta phi1 dphi1];
values = [p.R p.L p.M p.m p.Jr p.Jb p.g p.b p.br 0     0      0   0   ];

A = double(subs(A, params, values));
B = double(subs(B, params, values));
sys = ss(A, B, eye(6), zeros(6, 2));

%%

clc, clear variables

syms m M L theta Jr R Jbx Jby Jbz B

J5 = (Jbx + m*L^2) * sin(theta)^2 + Jbz*cos(theta)^2 + 3/4*M*B^2 + 1/2*M*R^2;

% ddtheta dv ddpsi
J = [Jby + m*L^2   , m*L*cos(theta)    , 0; ...
     m*L*cos(theta), m + 2*M + 2*Jr/R^2, 0; ...
     0             , 0                 , J5]

% ddtheta ddphi1 ddphi2 (ddphi1 and ddphi2 relative)
T = [1, 0, 0; ...
     0, R/2, R/2; ...
     0, R/B, -R/B]

J = T.' * J * T

% ddtheta ddphi1 ddphi2 (ddphi1 and ddphi2 absolute)
T = [ 1, 0, 0; ...
     -1, 1, 0; ...
     -1, 0, 1]

J = T.' * J * T














