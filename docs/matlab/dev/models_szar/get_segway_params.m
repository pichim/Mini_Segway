function params = get_segway_params(version)

    if ~exist('version', 'var')
        version = 0
    end
    
    switch version
        case 0
            params.R   = 0.028;
            params.L   = 0.15;
            params.M   = 0.01249;
            params.m   = 0.16726;
            params.Jb  = 1/12*params.m*params.L^2;
            params.Jr  = 1/2*params.M*params.R^2;
            params.g   = 9.81;
        otherwise
            % should not happen
    end

end