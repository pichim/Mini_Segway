% === Load CSV data ===
data = readmatrix('servo_camera_relationship.csv');  % Assumes header row exists
servo_angles = data(:,1);
camera_angles = data(:,2);

% === Fit a polynomial ===
degree = 3;  % You can change to 2, 4, etc.
coeffs = polyfit(servo_angles, camera_angles, degree);

% Create fitted values
x_fit = linspace(min(servo_angles), max(servo_angles), 200);
y_fit = polyval(coeffs, x_fit);

% === Plot the data and fit ===
figure;
plot(servo_angles, camera_angles, 'bo', 'MarkerSize', 8, 'DisplayName', 'Recorded Data');
hold on;
plot(x_fit, y_fit, 'r-', 'LineWidth', 2, 'DisplayName', sprintf('Poly Fit (deg %d)', degree));
xlabel('Servo Angle (°)');
ylabel('Camera Angle (°)');
title('Servo to Camera Angle Mapping');
legend('Location', 'northwest');
grid on;

% === Display Polynomial Equation ===
fprintf('\nPolynomial Coefficients (highest degree first):\n');
disp(coeffs);

% Display the equation in human-readable form
fprintf('Camera_Angle = ');
for i = 1:length(coeffs)
    exponent = degree - (i-1);
    c = coeffs(i);
    if abs(c) > 1e-6
        if c < 0
            fprintf('- ');
            c = -c;
        elseif i > 1
            fprintf('+ ');
        end

        if exponent == 0
            fprintf('%.4f ', c);
        elseif exponent == 1
            fprintf('%.4f * x ', c);
        else
            fprintf('%.4f * x^%d ', c, exponent);
        end
    end
end
fprintf('\n');
