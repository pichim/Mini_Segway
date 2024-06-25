function [X, X_w, freq] = calc_fft(x, Ts)

    N = length(x);
    if (mod(N, 2))
        N = N - 1;
        x = x(1:N);
    end

    df = 1 / (N*Ts);
    freq = (0:N-1).' * df;

    N = length(x);

    w = hann(N, 'periodic');
    
    scale = 1 / N;
    scale_w = 1 / sum(w);

    % x = x - mean(x);
    
    X = fft(x) * scale;
    X_w = fft(w .* x) * scale_w;

end

