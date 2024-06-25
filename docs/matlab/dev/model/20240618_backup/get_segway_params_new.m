function params = get_segway_params_new(version)
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

% changelog v0 -> v1:
% - L is new half of the body length resp. distance from wheel axis to COG
% - added friction coefficients b and br

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
            params.R  = 0.028;
            params.L  = 0.15 / 2.0;
            params.M  = 0.01249;
            params.m  = 0.16726;
            params.Jb = 1.0 / 3.0 * params.m*params.L^2;
            params.Jr = 1.0 / 2.0 * params.M*params.R^2;
            params.g  = 9.81;
            params.b  = 1e-4;
            params.br = 1e-4;
        otherwise
            % should not happen
    end

end