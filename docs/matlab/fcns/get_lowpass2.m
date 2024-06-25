function G = get_lowpass2(fcut, D, Ts)

    wcut = 2.0 * pi * fcut;
    k1 = 2.0 * D * Ts * wcut;
        
    a1 = 1.0 / (Ts * Ts * wcut * wcut + k1 + 1.0);
    b0 = 1.0 - a1 * (1.0 + k1);
    a0 = b0 - 1.0 - a1;

    B = [b0, 0, 0];
    A = [1, a0, a1];

    G = ss(tf(B, A, Ts));

end

