% PI Controller with Transport Delay (No Feedforward)
clear; clc;

% Timing
dt = 0.1;
t = 0:dt:1500;
N = length(t);

% Setpoints
T_setpoint = 4.0;       % [C]
T_in = 0.1;             % [C] inlet temp (constant)

% Fluid properties
fluid.rho = 1000;       % [kg/m^3]
fluid.Cp = 4217;        % [J/kg-K]
fluid.k = 0.6;          % [W/m-K]
fluid.mu = 0.001;       % [Pa-s]

% Heater geometry
cfg.D_tube = 0.003;     % 3mm ID
cfg.L_heater = 0.13;    % 130mm heater length
cfg.C_wall = 5;         % [J/K]

% Transport delay
L_sensor = 0.10;        % 100mm to sensor
V_sensor = pi * (cfg.D_tube/2)^2 * L_sensor;

% Controller
Kp = 5;                 % [W/C]
Ki = 0.1;               % [W/C-s]
P_min = 0;
P_max = 15;

% Initial conditions
T_wall = 0.1;
integral_error = 0;

% Pre-allocate
T_wall_hist = zeros(1,N);
T_out_hist = zeros(1,N);
T_sensor_hist = zeros(1,N);
P_heater_hist = zeros(1,N);
Q_flow_hist = zeros(1,N);
delay_hist = zeros(1,N);
T_out_hist(:) = T_in;

% Simulation
for i = 1:N
    
    % Flow profile: hold -> ramp up -> ramp down -> hold
    if t(i) < 500
        Q_flow = 3;
    elseif t(i) < 800
        Q_flow = 3 + (30 - 3) * (t(i) - 500) / 300;
    elseif t(i) < 1100
        Q_flow = 30 - (30 - 3) * (t(i) - 800) / 300;
    else
        Q_flow = 3;
    end
    
    % Transport delay (flow-dependent)
    Q_m3s = Q_flow * 1e-6 / 60;
    delay_time = V_sensor / Q_m3s;
    delay_samples = round(delay_time / dt);
    
    % Heater outlet temp
    [T_out, ~, ~, ~] = heater_dynamics(T_in, T_wall, Q_flow, 0, fluid, cfg);
    T_out_hist(i) = T_out;
    
    % Sensor sees delayed temperature
    idx_delayed = max(1, i - delay_samples);
    T_sensor = T_out_hist(idx_delayed);
    
    % PI control
    err = T_setpoint - T_sensor;
    integral_error = integral_error + err * dt;
    P_heater = Kp * err + Ki * integral_error;
    
    % Saturate with anti-windup
    if P_heater > P_max
        P_heater = P_max;
        integral_error = integral_error - err * dt;
    elseif P_heater < P_min
        P_heater = P_min;
        integral_error = integral_error - err * dt;
    end
    
    % Store
    T_wall_hist(i) = T_wall;
    T_sensor_hist(i) = T_sensor;
    P_heater_hist(i) = P_heater;
    Q_flow_hist(i) = Q_flow;
    delay_hist(i) = delay_time;
    
    % Update heater wall
    [~, dT_wall_dt, ~, ~] = heater_dynamics(T_in, T_wall, Q_flow, P_heater, fluid, cfg);
    T_wall = T_wall + dT_wall_dt * dt;
end

% Plot
figure(1); clf;
set(gcf, 'Position', [100 100 800 700], 'Color', 'w');

subplot(5,1,1);
plot(t, Q_flow_hist, 'g', 'LineWidth', 1.5);
ylabel('Q [mL/min]');
title('PI Control with Transport Delay (No Feedforward)');
grid on;

subplot(5,1,2);
hold on;
fill([0 t(end) t(end) 0], [T_setpoint-2 T_setpoint-2 T_setpoint+2 T_setpoint+2], ...
     [0.9 1 0.9], 'EdgeColor', 'none');
plot(t, T_out_hist, 'b', 'LineWidth', 1.5);
plot(t, T_sensor_hist, 'c--', 'LineWidth', 1);
yline(T_setpoint, 'k--');
ylabel('T [C]');
legend('Spec', 'T_{out}', 'T_{sensor}', 'Setpoint', 'Location', 'northeast');
grid on;

subplot(5,1,3);
plot(t, T_wall_hist, 'r', 'LineWidth', 1.5);
ylabel('T_{wall} [C]');
grid on;

subplot(5,1,4);
plot(t, P_heater_hist, 'm', 'LineWidth', 1.5);
ylabel('P [W]');
grid on;

subplot(5,1,5);
plot(t, delay_hist, 'k', 'LineWidth', 1.5);
ylabel('Delay [s]');
xlabel('Time [s]');
grid on;

print('-dpng', 'pi_delay_no_ff.png');

% Results
err_hist = T_setpoint - T_out_hist;
ramp_up = (t >= 500) & (t <= 800);
ramp_down = (t >= 800) & (t <= 1100);

fprintf('\nResults\n');
fprintf('  Kp = %.1f W/C, Ki = %.2f W/C-s\n', Kp, Ki);
fprintf('  Sensor distance: %.0f mm\n', L_sensor * 1000);
fprintf('  Delay range: %.1f - %.1f s\n', min(delay_hist), max(delay_hist));
fprintf('  Max undershoot (ramp up):  %.2f C\n', max(err_hist(ramp_up)));
fprintf('  Max overshoot (ramp down): %.2f C\n', -min(err_hist(ramp_down)));