function params = get_segway_params(version)

    if ~exist('version', 'var')
        version = 0
    end
    
    switch version
        case 0
            params.R   = 0.0715/2;
            params.L   = 0.0505;
            params.M   = 1.0;
            params.m   = 0.286;
            params.Jr  = 2.5e-5;
            params.Jb  = 0.0063;
            params.g   = 9.81;
        otherwise
            % should not happen
    end

end