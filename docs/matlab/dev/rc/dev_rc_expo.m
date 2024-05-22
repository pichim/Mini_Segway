clc, clear variables
%%

alpha = 0.3;
beta = 1.1;
scale = 1.0 / (exp(1)^alpha - 1.0);

x = (-1.0:1e-4:1.0).';
y = zeros(size(x));
for i = 1:length(x)
    x_abs  = abs(x(i));
    x_sign = sign(x(i));
    y(i) = scale * x_sign * (exp(x_abs)^alpha - 1.0) * x_abs^beta;
end

figure(1)
subplot(311)
plot(x, y), grid on
ylabel('y')
subplot(312)
plot(x(2:end), diff(y) ./ diff(x), 'color', [0 0.5 0]), grid on
ylabel('\partialy/\partialx')
subplot(313)
plot(x(2:end-1), diff(diff(y) ./ diff(x)) ./ diff(x(2:end)), 'r'), grid on
ylabel('\partial^2y/\partial^2x'), xlabel('x')