function params = get_segway_params(version)
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
% R : motor resistance
% km: motor constant
% L : motor inductance
% Kp: motor velocity controller P-gain
% Kv: motor velocity controller F-gain

% changelog v0 -> v1:
% - l is new half of the body length resp. distance from wheel axis to COG
% - 3D model
% - reordering and renaming of parameters
% - added friction coefficients b and br
% - calculate_inertia.m
% - dc_motor/modelling_and_cntrl_tuning_dc_motor_25D.m

    % default params
    if (~exist('version', 'var') || isempty(version))
        version = 1;
    end
    
    switch version
        case 0
            params.R  = 0.028;
            params.L  = 0.15;
            params.M  = 0.01249;
            params.m  = 0.16726;
            params.Jb = 1.0 / 12.0 * params.m*params.L^2;
            params.Jr = 1.0 / 2.0 * params.M*params.R^2;
            params.g  = 9.81;
        case 1
            % mechanical
            params.g  = 9.81;
            params.r  = 0.083 / 2.0;
            params.b  = 0.174;
            params.l  = 0.0482 * 1.5;
            params.m  = 0.78;
            params.Jx = 0.0208 / 31 / 10;
            params.Jy = 0.0113 / 31 / 10; % 3.5697e-04
            params.Jz = 0.0095 / 31 / 10;
            params.mw = 0.095;
            params.Jw = 1.7e-3;
            params.bw = 4.4391e-04;
            params.bg = 3.8402e-04;
            % motor
            params.R = 7.2;
            params.km = 0.546;
            params.L = 0.0075;
            params.Kp = 12.0;
            params.Kf = (0.9 * 60.0 / (170.0 / 12.0));
            params.CPR = 46.85 * 48.0 * 1e6; % counts per turn
            params.u_max = 8;
            params.u_min = -params.u_max;
            params.fcut = 40;
            params.D = sqrt(3.0) / 2.0;
        otherwise
            % should not happen
    end

end