clc, clear variables

% files:
% - LagrangeCaseStudy_HS2019.pdf

% notes:
% - 

%% lagranage (a bit hacky)

syms theta(t) x(t)
syms R L m M Jr Jb g

syms THETA dotTHETA ddotTHETA 
syms X     dotX     ddotX

dottheta  = diff(theta   , t);
dotx      = diff(x       , t);
ddottheta = diff(dottheta, t);
ddotx     = diff(dotx    , t);

abs_dvc_squared = dotX^2 + dotX*dotTHETA*L*cos(THETA) + dotTHETA^2 * (L^2)/4;

% kinetic energy
T = 1/2*(M + Jr / R^2) * dotX^2 + ...                 % kinetic energy from wheel
    1/2* m * abs_dvc_squared + 1/2 * Jb * dotTHETA^2; % kinetic energy from body

% potential energy
U = m*g*(L/2)*cos(THETA); % point mass body and wheel

% lagrange function
Lagrange = T - U
%                                       /  2         2                                      \
%                                       | L  dotTHETA                                     2 |
%            2                        m | ------------ + cos(THETA) L dotX dotTHETA + dotX  |
% Jb dotTHETA        2 / M    Jr  \     \       4                                           /   L g m cos(THETA)
% ------------ + dotX  | - + ---- | + ------------------------------------------------------- - ----------------
%       2              | 2      2 |                              2                                      2
%                      \     2 R  /

% dot(d L / d dot(qi)) - d L / d qi = Q for all i
%%
% d L / d dot(qi)
dLddottheta = diff(Lagrange, dotTHETA)
dLddotx   = diff(Lagrange, dotX  )

% dot(d L / d dot(qi))
dLddottheta = subs(dLddottheta, [THETA dotTHETA ddotTHETA X dotX ddotX], ...
                                [theta dottheta ddottheta x dotx ddotx])
dLddotx   = subs(dLddotx,   [THETA dotTHETA ddotTHETA X dotX ddotX], ...
                                [theta dottheta ddottheta x dotx ddotx])
ddLddottheta = diff(dLddottheta, t)
% Jb*diff(theta(t), t, t) + L*M*cos(diff(theta(t), t))*diff(x(t), t) - L*M*sin(diff(theta(t), t))*x(t)*diff(theta(t), t, t)
ddLddotx   = diff(dLddotx  , t)
% M*diff(x(t), t, t) + (Jr*diff(x(t), t, t))/R^2

%%
% need to substitute by hand, dono why...
ddLddottheta = (m*((L^2*ddotTHETA)/2 + L*cos(THETA)*ddotX - L*sin(THETA)*dotTHETA*dotX))/2 + Jb*ddotTHETA

ddLddotx = (m*(L*cos(THETA)*ddotTHETA - L*sin(THETA)*dotTHETA^2 + 2*ddotX))/2 + 2*(M/2 + Jr/(2*R^2))*ddotX

% d L / d qi
dLdtheta = diff(Lagrange, THETA)
dLdx   = diff(Lagrange, X  )

% DGL's
DGLtheta = simplify(ddLddottheta - dLdtheta)
% Jb*ddotTHETA + (L^2*ddotTHETA*m)/4 + (L*ddotX*m*cos(THETA))/2 - (L*g*m*sin(THETA))/2
% OLD Jb*ddotTHETA + L^2*M*ddotTHETA - L*M*g*sin(THETA) + L*M*R*ddotPHI*cos(THETA)

DGLx   = simplify(ddLddotx - dLdx)
% ddotX*(M + Jr/R^2) + (m*(- L*sin(THETA)*dotTHETA^2 + 2*ddotX + L*ddotTHETA*cos(THETA)))/2
% OLD Jr*ddotPHI + (M*(2*ddotPHI*R^2 - 2*L*sin(THETA)*R*dotTHETA^2 + 2*L*ddotTHETA*cos(THETA)*R))/2


