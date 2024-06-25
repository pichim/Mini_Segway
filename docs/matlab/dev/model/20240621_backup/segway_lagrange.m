clc, clear variables

% literatur:
% - LagrangeCaseStudy_HS2019.pdf
% - Yoeko Mak bachelor thesis s1070819 - niet vertrouwelijk.pdf
% - 775672333-MIT.pdf

% notes:
% - Lagrange equations with absolute angle of body (theta) and absolute
%   angle of wheels (phi1, phi2)


%% lagranage (a bit hacky)

% states
syms theta(t) phi1(t) phi2(t)


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


% variables for partial derivatives
syms THETA dTHETA ddTHETA 
syms PHI1  dPHI1  ddPHI1
syms PHI2  dPHI2  ddPHI2

% time derivatives
d_theta  = diff(theta, t);
d_phi1   = diff(phi1 , t);
d_phi2   = diff(phi2 , t);
dd_theta = diff(d_theta, t);
dd_phi1  = diff(d_phi1 , t);
dd_phi2  = diff(d_phi2 , t);


% intermediate variables
syms x(t) y(t) psi(t)

syms X dX
syms Y dY
syms PSI dPSI

d_x   = diff(x  , t);
d_y   = diff(y  , t);
d_psi = diff(psi, t);


% rotation matrices
Rz(t) = [ cos(psi)  -sin(psi)   0; ...
          sin(psi)   cos(psi)   0; ...
          0          0          1];
Ry(t) = [ cos(theta) 0 sin(theta); ...
          0          1 0
         -sin(theta) 0 cos(theta)];

% vector to mass of body
r_vec_m   = Rz * Ry * [0; 0; l] + [x; y; 0];
r_vec_mw1 = Rz * [0; -b/2; l] + [x; y; 0];
r_vec_mw2 = Rz * [0;  b/2; l] + [x; y; 0];

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

vars_partial = [dTHETA, dPHI1, dPHI2, dX, dY, dPSI, PSI, THETA];
vars_time    = [d_theta, d_phi1, d_phi2, d_x, d_y, d_psi, psi, theta];

dr_vec_squared_m   = subs(dr_vec_squared_m, vars_time, vars_partial);
dr_vec_squared_mw1 = subs(dr_vec_squared_mw1, vars_time, vars_partial);
dr_vec_squared_mw2 = subs(dr_vec_squared_mw2, vars_time, vars_partial);

% dotX = V * cos(PSI), V = r/2 * (dPHI1 + dPHI2)
% dotY = V * sin(PSI)
% dotPSI = r/b * (dPHI1 - dPHI2)
vars_interm  = [dX, dY, dPSI];
vars_org     = [r/2 * (dPHI1 + dPHI2) * cos(r/b*(PHI1 - PHI2)), ...
                r/2 * (dPHI1 + dPHI2) * sin(r/b*(PHI1 - PHI2)), ...
                r/b * (dPHI1 - dPHI2)];

dr_vec_squared_m   = subs(dr_vec_squared_m  , vars_interm, vars_org);
dr_vec_squared_mw1 = subs(dr_vec_squared_mw1, vars_interm, vars_org);
dr_vec_squared_mw2 = subs(dr_vec_squared_mw2, vars_interm, vars_org);

% PSI = r/b * (PHI1 - PHI2)
dr_vec_squared_m   = subs(dr_vec_squared_m  , PSI, r/b * (PHI1 - PHI2));
dr_vec_squared_mw1 = subs(dr_vec_squared_mw1, PSI, r/b * (PHI1 - PHI2));
dr_vec_squared_mw2 = subs(dr_vec_squared_mw2, PSI, r/b * (PHI1 - PHI2));

dr_vec_squared_m   = simplify( dr_vec_squared_m   );
dr_vec_squared_mw1 = simplify( dr_vec_squared_mw1 );
dr_vec_squared_mw2 = simplify( dr_vec_squared_mw2 );


% inertia body w.r.t. COG
J = diag([Jx Jy Jz]);

% rotational velocities [wx wy wz]
w = [0; diff(theta, t); 0] + Ry.' * [0; 0; diff(psi, t)];
w = subs(w, [d_theta, d_psi, theta, psi], [dTHETA, dPSI, THETA, PSI]);

w = subs(w, dPSI, r/b * (dPHI1 - dPHI2));
w = simplify( w );


%%

% kinetic energy
T = 1/2 * w.' * J * w + ...             % rot. body
    1/2 * Jw * dPHI1^2 + ...            % rot. wheel 1
    1/2 * Jw * dPHI2^2 + ...            % rot. wheel 2
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
dLddotphi1  = simplify( diff(L, dPHI1 ) );
dLddotphi2  = simplify( diff(L, dPHI2 ) );

vars_partial = [ddTHETA ddPHI1 ddPHI2 dTHETA dPHI1 dPHI2 THETA PHI1 PHI2];
vars_time    = [dd_theta dd_phi1 dd_phi2 d_theta d_phi1 d_phi2 theta phi1 phi2];

% dot(d L / d dot(qi))
dLddottheta = subs(dLddottheta, vars_partial, vars_time);
dLddotphi1  = subs(dLddotphi1,  vars_partial, vars_time);
dLddotphi2  = subs(dLddotphi2,  vars_partial, vars_time);

% d/dt ( dot(d L / d dot(qi)) )
ddLddottheta = simplify( diff(dLddottheta, t) );
ddLddotphi1  = simplify( diff(dLddotphi1 , t) );
ddLddotphi2  = simplify( diff(dLddotphi2 , t) );
ddLddottheta = subs(ddLddottheta, vars_time, vars_partial);
ddLddotphi1  = subs(ddLddotphi1 , vars_time, vars_partial);
ddLddotphi2  = subs(ddLddotphi2 , vars_time, vars_partial);


% d L / d qi
dLdtheta = diff(L, THETA);
dLdphi1  = diff(L, PHI1 );
dLdphi2  = diff(L, PHI2 );


% DGL's
DGLtheta = simplify( ddLddottheta - dLdtheta );
DGLphi1  = simplify( ddLddotphi1 - dLdphi1 );
DGLphi2  = simplify( ddLddotphi2 - dLdphi2 );


% convert to lower case notation
syms ddtheta ddphi1 ddphi2 dtheta dphi1 dphi2 theta phi1 phi2
vars_lowercase = [ddtheta ddphi1 ddphi2 dtheta dphi1 dphi2 theta phi1 phi2];

DGLtheta = subs(DGLtheta, vars_partial, vars_lowercase);
DGLtheta = collect(DGLtheta, [ddtheta ddphi1 ddphi2])
% ((2*m*b^2*l^2 + 2*Jy*b^2)/(2*b^2))*ddtheta + ((l*m*r*cos(theta))/2)*ddphi1 + ((l*m*r*cos(theta))/2)*ddphi2
% - (Jx*dphi1^2*r^2*sin(2*theta) + Jx*dphi2^2*r^2*sin(2*theta) - Jz*dphi1^2*r^2*sin(2*theta) - Jz*dphi2^2*r^2*sin(2*theta) + 2*b^2*g*l*m*sin(theta) + dphi1^2*l^2*m*r^2*sin(2*theta) + dphi2^2*l^2*m*r^2*sin(2*theta) - 2*Jx*dphi1*dphi2*r^2*sin(2*theta) + 2*Jz*dphi1*dphi2*r^2*sin(2*theta) - 2*dphi1*dphi2*l^2*m*r^2*sin(2*theta))/(2*b^2)

DGLphi1 = subs(DGLphi1, vars_partial, vars_lowercase);
DGLphi1 = collect(DGLphi1, [ddtheta ddphi1 ddphi2])
% ((l*m*r*cos(theta))/2)*ddtheta + ((4*Jw*b^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*l^2*m*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2 - 4*l^2*m*r^2*cos(theta)^2)/(4*b^2))*ddphi1 + (-(4*m*l^2*r^2 - 4*m*l^2*r^2*cos(theta)^2 - m*b^2*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2)/(4*b^2))*ddphi2
% - (4*Jx*dphi2*dtheta*r^2*sin(2*theta) - 4*Jx*dphi1*dtheta*r^2*sin(2*theta) + 4*Jz*dphi1*dtheta*r^2*sin(2*theta) - 4*Jz*dphi2*dtheta*r^2*sin(2*theta) + 2*b^2*dtheta^2*l*m*r*sin(theta) - 4*dphi1*dtheta*l^2*m*r^2*sin(2*theta) + 4*dphi2*dtheta*l^2*m*r^2*sin(2*theta))/(4*b^2)

DGLphi2 = subs(DGLphi2, vars_partial, vars_lowercase);
DGLphi2 = collect(DGLphi2, [ddtheta ddphi1 ddphi2])
% ((l*m*r*cos(theta))/2)*ddtheta + (-(4*m*l^2*r^2 - 4*m*l^2*r^2*cos(theta)^2 - m*b^2*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2)/(4*b^2))*ddphi1 + ((4*Jw*b^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*l^2*m*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2 - 4*l^2*m*r^2*cos(theta)^2)/(4*b^2))*ddphi2
% - (4*Jx*dphi1*dtheta*r^2*sin(2*theta) - 4*Jx*dphi2*dtheta*r^2*sin(2*theta) - 4*Jz*dphi1*dtheta*r^2*sin(2*theta) + 4*Jz*dphi2*dtheta*r^2*sin(2*theta) + 2*b^2*dtheta^2*l*m*r*sin(theta) + 4*dphi1*dtheta*l^2*m*r^2*sin(2*theta) - 4*dphi2*dtheta*l^2*m*r^2*sin(2*theta))/(4*b^2)


% J * [ddtheta, ddphi1, ddphi2]^T = rhs <-> [ddtheta, ddphi1, ddphi2]^T = J^-1 * rhs

J = [((2*m*b^2*l^2 + 2*Jy*b^2)/(2*b^2)), ((l*m*r*cos(theta))/2), ((l*m*r*cos(theta))/2); ...
     ((l*m*r*cos(theta))/2), ((4*Jw*b^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*l^2*m*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2 - 4*l^2*m*r^2*cos(theta)^2)/(4*b^2)), (-(4*m*l^2*r^2 - 4*m*l^2*r^2*cos(theta)^2 - m*b^2*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2)/(4*b^2)); ...
     ((l*m*r*cos(theta))/2), (-(4*m*l^2*r^2 - 4*m*l^2*r^2*cos(theta)^2 - m*b^2*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2)/(4*b^2)), ((4*Jw*b^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*l^2*m*r^2 + 4*Jz*r^2*cos(theta)^2 + 4*Jx*r^2*sin(theta)^2 - 4*l^2*m*r^2*cos(theta)^2)/(4*b^2))];
J = simplify(J)
% [          m*l^2 + Jy,                                                                                                                (l*m*r*cos(theta))/2,                                                                                                                (l*m*r*cos(theta))/2]
% [(l*m*r*cos(theta))/2, (4*Jw*b^2 + 4*Jz*r^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*Jx*r^2*sin(theta)^2 - 4*Jz*r^2*sin(theta)^2 + 4*l^2*m*r^2*sin(theta)^2)/(4*b^2),                                        -(r^2*(4*Jz - b^2*m + 4*Jx*sin(theta)^2 - 4*Jz*sin(theta)^2 + 4*l^2*m*sin(theta)^2))/(4*b^2)]
% [(l*m*r*cos(theta))/2,                                        -(r^2*(4*Jz - b^2*m + 4*Jx*sin(theta)^2 - 4*Jz*sin(theta)^2 + 4*l^2*m*sin(theta)^2))/(4*b^2), (4*Jw*b^2 + 4*Jz*r^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*Jx*r^2*sin(theta)^2 - 4*Jz*r^2*sin(theta)^2 + 4*l^2*m*r^2*sin(theta)^2)/(4*b^2)]

rhs = - [- (Jx*dphi1^2*r^2*sin(2*theta) + Jx*dphi2^2*r^2*sin(2*theta) - Jz*dphi1^2*r^2*sin(2*theta) - Jz*dphi2^2*r^2*sin(2*theta) + 2*b^2*g*l*m*sin(theta) + dphi1^2*l^2*m*r^2*sin(2*theta) + dphi2^2*l^2*m*r^2*sin(2*theta) - 2*Jx*dphi1*dphi2*r^2*sin(2*theta) + 2*Jz*dphi1*dphi2*r^2*sin(2*theta) - 2*dphi1*dphi2*l^2*m*r^2*sin(2*theta))/(2*b^2); ...
         - (4*Jx*dphi2*dtheta*r^2*sin(2*theta) - 4*Jx*dphi1*dtheta*r^2*sin(2*theta) + 4*Jz*dphi1*dtheta*r^2*sin(2*theta) - 4*Jz*dphi2*dtheta*r^2*sin(2*theta) + 2*b^2*dtheta^2*l*m*r*sin(theta) - 4*dphi1*dtheta*l^2*m*r^2*sin(2*theta) + 4*dphi2*dtheta*l^2*m*r^2*sin(2*theta))/(4*b^2); ...
         - (4*Jx*dphi1*dtheta*r^2*sin(2*theta) - 4*Jx*dphi2*dtheta*r^2*sin(2*theta) - 4*Jz*dphi1*dtheta*r^2*sin(2*theta) + 4*Jz*dphi2*dtheta*r^2*sin(2*theta) + 2*b^2*dtheta^2*l*m*r*sin(theta) + 4*dphi1*dtheta*l^2*m*r^2*sin(2*theta) - 4*dphi2*dtheta*l^2*m*r^2*sin(2*theta))/(4*b^2)];
rhs = simplify(rhs)
% (Jx*dphi1^2*r^2*sin(2*theta) + Jx*dphi2^2*r^2*sin(2*theta) - Jz*dphi1^2*r^2*sin(2*theta) - Jz*dphi2^2*r^2*sin(2*theta) + 2*b^2*g*l*m*sin(theta) + dphi1^2*l^2*m*r^2*sin(2*theta) + dphi2^2*l^2*m*r^2*sin(2*theta) - 2*Jx*dphi1*dphi2*r^2*sin(2*theta) + 2*Jz*dphi1*dphi2*r^2*sin(2*theta) - 2*dphi1*dphi2*l^2*m*r^2*sin(2*theta))/(2*b^2)
%                                                                                                              (dtheta*r*(2*Jx*dphi2*r*sin(2*theta) - 2*Jx*dphi1*r*sin(2*theta) + 2*Jz*dphi1*r*sin(2*theta) - 2*Jz*dphi2*r*sin(2*theta) + b^2*dtheta*l*m*sin(theta) - 2*dphi1*l^2*m*r*sin(2*theta) + 2*dphi2*l^2*m*r*sin(2*theta)))/(2*b^2)
%                                                                                                              (dtheta*r*(2*Jx*dphi1*r*sin(2*theta) - 2*Jx*dphi2*r*sin(2*theta) - 2*Jz*dphi1*r*sin(2*theta) + 2*Jz*dphi2*r*sin(2*theta) + b^2*dtheta*l*m*sin(theta) + 2*dphi1*l^2*m*r*sin(2*theta) - 2*dphi2*l^2*m*r*sin(2*theta)))/(2*b^2)

% ddtheta ddphi1 ddphi2
T = [1, 0  ,  0  ; ...
     0, r/2,  r/2; ...
     0, r/b, -r/b]^-1

J = T.' * J * T
% [    m*l^2 + Jy,                                                                                                                                                                                                                            l*m*cos(theta),                                                                                                                                                                                                                                     0]
% [l*m*cos(theta), (2*((4*Jw*b^2 + 4*Jz*r^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*Jx*r^2*sin(theta)^2 - 4*Jz*r^2*sin(theta)^2 + 4*l^2*m*r^2*sin(theta)^2)/(4*b^2*r) - (r*(4*Jz - b^2*m + 4*Jx*sin(theta)^2 - 4*Jz*sin(theta)^2 + 4*l^2*m*sin(theta)^2))/(4*b^2)))/r,                                                                                                                                                                                                                                     0]
% [             0,                                                                                                                                                                                                                                         0, (b*((4*Jw*b^2 + 4*Jz*r^2 + b^2*m*r^2 + 4*b^2*mw*r^2 + 4*Jx*r^2*sin(theta)^2 - 4*Jz*r^2*sin(theta)^2 + 4*l^2*m*r^2*sin(theta)^2)/(8*b*r) + (r*(4*Jz - b^2*m + 4*Jx*sin(theta)^2 - 4*Jz*sin(theta)^2 + 4*l^2*m*sin(theta)^2))/(8*b)))/r]









