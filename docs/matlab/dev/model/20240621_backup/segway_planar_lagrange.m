clc, clear variables

% literatur:
% - LagrangeCaseStudy_HS2019.pdf
% - Yoeko Mak bachelor thesis s1070819 - niet vertrouwelijk.pdf
% - 775672333-MIT.pdf

% notes:
% - Lagrange equations with absolute angle of body (theta) and absolute
%   angle of wheel (phi)


%% lagranage (a bit hacky)

% states
syms theta(t) phi(t)


% parameters in SI units
% g : gravity
% r : radus of wheel
% b : distance between wheels
% l : distance from wheel axis to COG of body
% m : mass of body
% Jy: inertia body
% mw: mass of wheel
% Jw: inertia wheel
% bw: friction between wheel and body
% bg: friction between wheel and ground
syms g r b l m Jy mw Jw bw bg


% variables for partial derivatives
syms THETA dTHETA ddTHETA 
syms PHI  dPHI  ddPHI

% time derivatives
d_theta  = diff(theta, t);
d_phi   = diff(phi , t);
dd_theta = diff(d_theta, t);
dd_phi  = diff(d_phi , t);


% intermediate variables
syms x(t)

syms X dX

d_x   = diff(x  , t);


% rotation matrix
Ry(t) = [ cos(theta) sin(theta); ...
         -sin(theta) cos(theta)];

% vector to mass of body
r_vec_m   = Ry * [0; l] + [x; 0];
r_vec_mw1 = [x; 0];
r_vec_mw2 = [x; 0];

% time derivative
dr_vec_m   = diff(r_vec_m, t);
dr_vec_mw1 = diff(r_vec_mw1, t);
dr_vec_mw2 = diff(r_vec_mw2, t);
dr_vec_m   = simplify( dr_vec_m   );
dr_vec_mw1 = simplify( dr_vec_mw1 );
dr_vec_mw2 = simplify( dr_vec_mw2 );


% position vectors to masses
dr_vec_squared_m   = dr_vec_m.'   * dr_vec_m  ;
dr_vec_squared_mw1 = dr_vec_mw1.' * dr_vec_mw1;
dr_vec_squared_mw2 = dr_vec_mw2.' * dr_vec_mw2;
dr_vec_squared_m   = simplify( dr_vec_squared_m   );
dr_vec_squared_mw1 = simplify( dr_vec_squared_mw1 );
dr_vec_squared_mw2 = simplify( dr_vec_squared_mw2 );

vars_partial = [dTHETA, dPHI, dX, THETA];
vars_time    = [d_theta, d_phi, d_x, theta];

dr_vec_squared_m   = subs(dr_vec_squared_m, vars_time, vars_partial);
dr_vec_squared_mw1 = subs(dr_vec_squared_mw1, vars_time, vars_partial);
dr_vec_squared_mw2 = subs(dr_vec_squared_mw2, vars_time, vars_partial);

% dotX = V, V = r * dPHI
vars_interm  = [dX];
vars_org     = [r * dPHI];

dr_vec_squared_m   = subs(dr_vec_squared_m  , vars_interm, vars_org);
dr_vec_squared_mw1 = subs(dr_vec_squared_mw1, vars_interm, vars_org);
dr_vec_squared_mw2 = subs(dr_vec_squared_mw2, vars_interm, vars_org);

% inertia body w.r.t. COG
J = Jy;

% rotational velocities [wx wy wz]
w = diff(theta, t);
w = subs(w, [d_theta], [dTHETA]);


%%

% kinetic energy
T = 1/2 * w.' * J * w + ...             % rot. body
    1/2 * Jw * dPHI^2 + ...             % rot. wheel 1
    1/2 * Jw * dPHI^2 + ...             % rot. wheel 2
    1/2 * m  * dr_vec_squared_m + ...   % trans. body
    1/2 * mw * dr_vec_squared_mw1 + ... % trans. wheel 1
    1/2 * mw * dr_vec_squared_mw2;      % trans. wheel 1
T = simplify( T );

% potential energy
U = m * g * l * cos(THETA); % point mass body
U = simplify( U );

% lagrange function
L = T - U;
L = simplify( L );


% dot(d L / d dot(qi)) - d L / d qi = Q for all i


% d L / d dot(qi)
dLddottheta = simplify( diff(L, dTHETA) );
dLddotphi   = simplify( diff(L, dPHI  ) );

vars_partial = [ddTHETA ddPHI dTHETA dPHI THETA PHI];
vars_time    = [dd_theta dd_phi d_theta d_phi theta phi];

% dot(d L / d dot(qi))
dLddottheta = subs(dLddottheta, vars_partial, vars_time);
dLddotphi  = subs(dLddotphi,  vars_partial, vars_time);

% d/dt ( dot(d L / d dot(qi)) )
ddLddottheta = simplify( diff(dLddottheta, t) );
ddLddotphi   = simplify( diff(dLddotphi , t) );
ddLddottheta = subs(ddLddottheta, vars_time, vars_partial);
ddLddotphi   = subs(ddLddotphi  , vars_time, vars_partial);


% d L / d qi
dLdtheta = diff(L, THETA);
dLdphi   = diff(L, PHI  );


% DGL's
DGLtheta = simplify( ddLddottheta - dLdtheta );
DGLphi   = simplify( ddLddotphi   - dLdphi   );


% convert to lower case notation
syms ddtheta ddphi dtheta dphi theta phi
vars_lowercase = [ddtheta ddphi dtheta dphi theta phi];

DGLtheta = subs(DGLtheta, vars_partial, vars_lowercase);
DGLtheta = collect(DGLtheta, [ddtheta])
% (m*l^2 + Jy)*ddtheta + ddphi*l*m*r*cos(theta) - g*l*m*sin(theta)

DGLphi = subs(DGLphi, vars_partial, vars_lowercase);
DGLphi = collect(DGLphi, [ddtheta ddphi])
% - l*m*r*sin(theta)*dtheta^2 + ddphi*(2*Jw + m*r^2 + 2*mw*r^2) + ddtheta*l*m*r*cos(theta)


% J * [ddtheta, ddphi]^T = rhs <-> [ddtheta, ddphi]^T = J^-1 * rhs

J = [(m*l^2 + Jy), l*m*r*cos(theta); ...
     l*m*r*cos(theta), (2*Jw + m*r^2 + 2*mw*r^2)];
J = simplify(J)
% [      m*l^2 + Jy,        l*m*r*cos(theta)]
% [l*m*r*cos(theta), 2*Jw + m*r^2 + 2*mw*r^2]

rhs = - [- g*l*m*sin(theta); ...
         - l*m*r*sin(theta)*dtheta^2];
rhs = simplify(rhs)
%          g*l*m*sin(theta)
% dtheta^2*l*m*r*sin(theta)








