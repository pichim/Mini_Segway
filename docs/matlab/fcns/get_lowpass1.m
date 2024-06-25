function G = get_lowpass1(fcut, Ts)

    b0 = 1.0 - exp(-Ts * 2.0 * pi * fcut);
    a0 = b0 - 1.0;
    
    B = [b0, 0];
    A = [1, a0];

    G = ss(tf(B, A, Ts));

end