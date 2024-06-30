function G = get_differentiating_lowpass1(fcut, Ts)

    b0 = 1.0 - exp(-Ts * 2.0 * pi * fcut);
    a0 = b0 - 1.0;
    
    B = [b0/Ts, -b0/Ts];
    A = [1, a0];

    G = ss(tf(B, A, Ts));

end