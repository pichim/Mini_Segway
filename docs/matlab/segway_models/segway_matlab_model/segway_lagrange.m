clc, clear variables

% files:
% - LagrangeCaseStudy_HS2019.pdf

% notes:
% - Lagrange equations with absolute angle of body (theta) and absolute
% angle of wheel (phi)
% - This file is to get nonlinear differential equations and use them in
% segway_ini.m file

%% lagranage (a bit hacky)

syms theta(t) phi(t)
syms R L M m Jr Jb g

syms THETA dotTHETA ddotTHETA 
syms PHI   dotPHI   ddotPHI

 dottheta = diff(theta   , t);
 dotphi   = diff(phi     , t);
ddottheta = diff(dottheta, t);
ddotphi   = diff(dotphi  , t);

abs_dvc_squared = (dotPHI * R)^2 + dotPHI*R*dotTHETA*L*cos(THETA) + dotTHETA^2 * (L^2)/4;

% kinetic energy
T = 1/2 * M  * (dotPHI * R)^2 + 1/2 * Jr * dotPHI^2 +...  % kinetic energy from wheel 
    1/2* m * abs_dvc_squared + 1/2 * Jb * dotTHETA^2;     % kinetic energy from body

% potential energy
U = m*g*(L/2)*cos(THETA); % point mass body and wheel

% lagrange function
Lagrange = T - U;

%   /  2         2                                               \
%   | L  dotTHETA                                      2       2 |
% m | ------------ + cos(THETA) L R dotPHI dotTHETA + R  dotPHI  |              2            2      2       2
%   \       4                                                    /   Jb dotTHETA    Jr dotPHI    M R  dotPHI    L g m cos(THETA)
% ---------------------------------------------------------------- + ------------ + ---------- + ------------ - ----------------
%                                 2                                        2             2             2                2

% dot(d L / d dot(qi)) - d L / d qi = Q for all i
%%
% 
% d L / d dot(qi)
%Partial differential equations after the first derivative of variable after time
dLddottheta = diff(Lagrange, dotTHETA)
dLddotphi   = diff(Lagrange, dotPHI  )

% dot(d L / d dot(qi))
dLddottheta = subs(dLddottheta, [THETA dotTHETA ddotTHETA PHI dotPHI ddotPHI], ...
                                [theta dottheta ddottheta phi dotphi ddotphi])
dLddotphi   = subs(dLddotphi,   [THETA dotTHETA ddotTHETA PHI dotPHI ddotPHI], ...
                                [theta dottheta ddottheta phi dotphi ddotphi])
%Differential equations from partial differential equetions after
%derivative of variable after time
ddLddottheta = diff(dLddottheta, t)
% (m*((L^2*diff(theta(t), t, t))/2 + L*R*cos(theta(t))*diff(phi(t), t, t) - L*R*sin(theta(t))*diff(phi(t), t)*diff(theta(t), t)))/2 + Jb*diff(theta(t), t, t)
ddLddotphi   = diff(dLddotphi  , t)
% (m*(2*R^2*diff(phi(t), t, t) - L*R*sin(theta(t))*diff(theta(t), t)^2 + L*R*cos(theta(t))*diff(theta(t), t, t)))/2 + Jr*diff(phi(t), t, t) + M*R^2*diff(phi(t), t, t)
%%
% need to substitute by hand, dono why...
ddLddottheta = (m*((L^2*ddotTHETA)/2 + L*R*cos(THETA)*ddotPHI - L*R*sin(THETA)*dotPHI*dotTHETA))/2 + Jb*ddotTHETA

ddLddotphi   = (m*(2*R^2*ddotPHI - L*R*sin(THETA)*dotTHETA^2 + L*R*cos(THETA)*ddotTHETA))/2 + Jr*ddotPHI + M*R^2*ddotPHI

% d L / d qi
dLdtheta = diff(Lagrange, THETA)
dLdphi   = diff(Lagrange, PHI  )

% DGL's
DGLtheta = simplify(ddLddottheta - dLdtheta)
% Jb*ddotTHETA + (L^2*ddotTHETA*m)/4 - (L*g*m*sin(THETA))/2 + (L*R*ddotPHI*m*cos(THETA))/2

DGLphi   = simplify(ddLddotphi - dLdphi)
% Jr*ddotPHI + (m*(2*ddotPHI*R^2 - L*sin(THETA)*R*dotTHETA^2 + L*ddotTHETA*cos(THETA)*R))/2 + M*R^2*ddotPHI


