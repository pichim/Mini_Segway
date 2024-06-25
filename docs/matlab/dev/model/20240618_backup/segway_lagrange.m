clc, clear variables

% files:
% - LagrangeCaseStudy_HS2019.pdf

% notes:
% - Lagrange equations with absolute angle of body (theta) and absolute
% angle of wheels (phi1, phi2)


%% lagranage (a bit hacky)

syms theta(t) phi1(t) phi2(t)
syms g r b l m Jx Jy Jz mw Jw 

syms THETA dotTHETA ddotTHETA 
syms PHI1  dotPHI1  ddotPHI1
syms PHI2  dotPHI2  ddotPHI2

dottheta = diff(theta, t);
dotphi1 = diff(phi1, t);
dotphi2 = diff(phi2, t);
ddottheta = diff(dottheta, t);
ddotphi1 = diff(dotphi1, t);
ddotphi2 = diff(dotphi2, t);

syms x(t) y(t) psi(t)

Rz(t) = [cos(psi) -sin(psi) 0; ...
         sin(psi)  cos(psi) 0; ...
         0         0        1]

Ry(t) = [ cos(theta) 0 sin(theta); ...
          0          1 0
         -sin(theta) 0 cos(theta)]

r_vec = Rz * Ry * [0; 0; l] + [x; y; 0]

dr_vec = diff(r_vec, t)

dr_vec_squared = simplify(dr_vec.' * dr_vec);
dr_vec_squared = simplify(dr_vec_squared)

syms X dotX
syms Y dotY
syms PSI dotPSI

dotx = diff(x, t);
doty = diff(y, t);
dotpsi = diff(psi, t);

dr_vec_squared = subs(dr_vec_squared, [dottheta, dotphi1, dotphi2, dotx, doty, dotpsi, psi], [dotTHETA, dotPHI1, dotPHI2, dotX, dotY, dotPSI, PSI]);
dr_vec_squared = simplify(dr_vec_squared)

dr_vec_squared = subs(dr_vec_squared, [dotX, dotY, dotPSI], ...
    [cos(r/b*(PHI1 - PHI2)) * r/2 * (dotPHI1 + dotPHI2), ...
     sin(r/b*(PHI1 - PHI2)) * r/2 * (dotPHI1 + dotPHI2), ...
     r/b*(dotPHI1 - dotPHI2)]);
dr_vec_squared = simplify(dr_vec_squared)

dr_vec_squared = subs(dr_vec_squared, PSI, r/b*(PHI1 - PHI2));
dr_vec_squared = simplify(dr_vec_squared)

J = diag([Jx Jy Jz])

% [wx wy wz]
w = [0; diff(theta, t); 0] + Ry.' * [0; 0; diff(psi, t)]
w = subs(w, [dottheta, dotpsi, theta, psi], [dotTHETA, dotPSI, THETA, PSI])

%%

% kinetic energy
T = 1/2 * w.' * J * w + ...
    1/2 * Jw * dotPHI1^2 + ...
    1/2 * Jw * dotPHI2^2 + ...
    1/2 * m * dr_vec_squared + ...
    1/2 * mw * (r * dotPHI1)^2 + ...
    1/2 * mw * (r * dotPHI2)^2;
T = simplify(T)

% potential energy
U = m * g * l * cos(THETA); % point mass body and wheel
U = simplify(U)

% lagrange function
Lagrange = T - U;
Lagrange = simplify(Lagrange)


% dot(d L / d dot(qi)) - d L / d qi = Q for all i

% d L / d dot(qi)
dLddottheta = simplify( diff(Lagrange, dotTHETA) )
dLddotphi1  = simplify( diff(Lagrange, dotPHI1) )
dLddotphi2  = simplify( diff(Lagrange, dotPHI2) )

vars_partial = [ddotTHETA ddotPHI1 ddotPHI2 dotTHETA dotPHI1 dotPHI2 THETA PHI1 PHI2]
vars_time    = [ddottheta ddotphi1 ddotphi2 dottheta dotphi1 dotphi2 theta phi1 phi2]

% dot(d L / d dot(qi))
dLddottheta = subs(dLddottheta, vars_partial, vars_time)
dLddotphi1  = subs(dLddotphi1,  vars_partial, vars_time)
dLddotphi2  = subs(dLddotphi2,  vars_partial, vars_time)

% d/dt ( dot(d L / d dot(qi)) )
ddLddottheta = simplify( diff(dLddottheta, t) );
ddLddotphi1  = simplify( diff(dLddotphi1, t) );
ddLddotphi2  = simplify( diff(dLddotphi2, t) );

% ddLddottheta_0 = ddLddottheta
% ddLddotphi1_0  = ddLddotphi1
% ddLddotphi2_0  = ddLddotphi2

ddLddottheta = subs(ddLddottheta, vars_time, vars_partial);
ddLddotphi1  = subs(ddLddotphi1, vars_time, vars_partial);
ddLddotphi2  = subs(ddLddotphi2, vars_time, vars_partial);

% ddLddottheta_1 = subs(ddLddottheta, vars_partial, vars_time);
% simplify(ddLddottheta_0 - ddLddottheta_1)
% ddLddotphi1_1 = subs(ddLddotphi1, vars_partial, vars_time);
% simplify(ddLddotphi1_0 - ddLddotphi1_1)
% ddLddotphi2_1 = subs(ddLddotphi2, vars_partial, vars_time);
% simplify(ddLddotphi2_0 - ddLddotphi2_1)

% d L / d qi
dLdtheta = diff(Lagrange, THETA)
dLdphi1  = diff(Lagrange, PHI1)
dLdphi2  = diff(Lagrange, PHI2)

% syms THETA dotTHETA ddotTHETA 
% syms PHI1  dotPHI1  ddotPHI1
% syms PHI2  dotPHI2  ddotPHI2

% DGL's
DGLtheta = simplify(ddLddottheta - dLdtheta)
DGLtheta = collect(DGLtheta, [ddotTHETA, ddotPHI1, ddotPHI2])
% (m*l^2 + Jy)*ddotTHETA + ((l*m*r*cos(THETA))/2)*ddotPHI1 + ((l*m*r*cos(THETA))/2)*ddotPHI2 + (Jz*dotPSI^2*sin(2*THETA))/2 - (Jx*dotPSI^2*sin(2*THETA))/2 - g*l*m*sin(THETA) - (dotPHI1*dotTHETA*l*m*r*sin(THETA))/2 - (dotPHI2*dotTHETA*l*m*r*sin(THETA))/2

DGLphi1   = simplify(ddLddotphi1 - dLdphi1)
DGLphi1 = collect(DGLphi1, [ddotTHETA, ddotPHI1, ddotPHI2])
% ((l*m*r*cos(THETA))/2)*ddotTHETA + ((4*Jw*b^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 2*l^2*m*r^2 - 2*l^2*m*r^2*cos(2*THETA))/(4*b^2))*ddotPHI1 + ((b^2*m*r^2 - 2*l^2*m*r^2 + 2*l^2*m*r^2*cos(2*THETA))/(4*b^2))*ddotPHI2 - (4*dotPHI2*dotTHETA*l^2*m*r^2*sin(2*THETA) - 4*dotPHI1*dotTHETA*l^2*m*r^2*sin(2*THETA) + 2*b^2*dotTHETA^2*l*m*r*sin(THETA))/(4*b^2)

DGLphi2   = simplify(ddLddotphi2 - dLdphi2)
DGLphi2 = collect(DGLphi2, [ddotTHETA, ddotPHI1, ddotPHI2])
% ((l*m*r*cos(THETA))/2)*ddotTHETA + ((b^2*m*r^2 - 2*l^2*m*r^2 + 2*l^2*m*r^2*cos(2*THETA))/(4*b^2))*ddotPHI1 + ((4*Jw*b^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 2*l^2*m*r^2 - 2*l^2*m*r^2*cos(2*THETA))/(4*b^2))*ddotPHI2 - (4*dotPHI1*dotTHETA*l^2*m*r^2*sin(2*THETA) - 4*dotPHI2*dotTHETA*l^2*m*r^2*sin(2*THETA) + 2*b^2*dotTHETA^2*l*m*r*sin(THETA))/(4*b^2)

