clc, clear variables
%% Segway init file
addpath('../../segway_models')
p = get_segway_params;
seg.Rwe = p.R;   % radius of wheel
seg.Mwe = p.M;   % mass of wheel
seg.Mbd = p.m;   % Mass body
seg.L = p.L;     % lenght of body
seg.Jbd= p.Jb;   % inertia of the body
seg.Wwe = .16;   % width of wheel center
seg.Hbd = .1;
%% --> switch "manual switch" to "in1" 
K=zeros(1,4);
[aa,bb,cc,dd]=linmod('segway_simple_rack_pinion');
dum=minreal(ss(aa,bb,cc,dd));
sys=ss2ss(dum,dum.c)
dcgain(sys)

% segway position is allowed to be arbitrary
sys = ss(sys.A(1:3,1:3), sys.B(1:3), eye(3), zeros(3, 1));
eig(sys)

figure(1)
pzmap(sys), grid off
%%
A4 = [sys.a zeros(3,1);-[1 0 0] 0];
B4 = [sys.b;0];
K = place(A4,B4,[-5 -5+5j -5-5j -7]);
% for time Simulation switch back to gain source


%% Model with speed as input
% --> switch "manual switch" to "in1" 
kp2=1;Tn2=1; % (temporary)
Tn=.02;
kp=.005;
Kv=zeros(1,4);
[aa,bb,cc,dd]=linmod('segway_simple_speed_as_input');
dum=minreal(ss(aa,bb,cc,dd));
sys_v=ss2ss(dum,dum.c);
% for time Simulation switch back to gain source
%% balancing controller

Tn2 = .1;
PI = tf([Tn2 1],[Tn2 0]);
rlocus(-PI*sys_v(4))
axis equal
kp2 = 50;
sgrid

s_cl=feedback(sys_v,-kp2*PI,1,4);





