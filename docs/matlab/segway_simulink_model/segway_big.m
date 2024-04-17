clc, clearvars
%Script for setting multibody dimensions and parameters
% Lower base
LowerBase.r = 14; %[mm]
LowerBase.L = 114; %[mm]
LowerBase.mass = 118.59; %[g]

% Battery case
BatCase.x = 80; %[mm]
BatCase.y = 61; %[mm]
BatCase.z = 22; %[mm]
BatCase.mass = 167.26; %[g]

% PCB
PCB.x = 110; %[mm]
PCB.y = 70; %[mm]
PCB.z = 21; %[mm]
PCB.mass = 84.53; %[g]

% Wheel
Wheel.r = 56.6 / 2; %[mm]
Wheel.L = 8; %[mm]
Wheel.mass = 12.49; %[g]

% Shaft
Shaft.x = 3; %[mm]
Shaft.y = 3; %[mm]
Shaft.z = 130; %[mm]
Shaft.mass = 0.01; %[mm]