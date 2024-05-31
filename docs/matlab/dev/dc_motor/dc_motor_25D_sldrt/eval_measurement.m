clc, clear variables
%%

load data_battery.mat
data_battery = data;

load data_power_supply.mat
data_power_supply = data;

s = tf('s')
T1 = 1.0 * 32e-3;
T2 = 0.0 * 1e-3;
G_mod = 0.2352 * 1/(T1*s + 1) * 1/(T2*s + 1) * 1/s;

G_mod = c2d(G_mod, Ts);
G_mod = G_mod * Gdiff;

time = (0:Ts:0.4).';
y_mod = step(G_mod, time);

figure(1)
subplot(211)
plot(data_battery.time - 17.2859 + 0.002, -data_battery.signals.values(:,2) / 12), grid on, hold on
plot(data_power_supply.time - 15.1036 + 0.002, -data_power_supply.signals.values(:,2) / 12), hold off
ylabel('Velocity (rps)')
xlim([0 0.4])
subplot(212)
plot(data_battery.time - 17.2859 + 0.002, -data_battery.signals.values(:,2) * 2.82679 / 3.29 / 12), grid on, hold on
plot(data_power_supply.time - 15.1036 + 0.002, -data_power_supply.signals.values(:,2) / 12)
plot(time, y_mod), hold off
xlim([0 0.4])
ylabel('Scaled Velocity (match Battery) (rps)'), xlabel('Time (sec)')
legend('Power Supply', ...
    'Battery', ...
    'Model', ...
    'Location', 'best')