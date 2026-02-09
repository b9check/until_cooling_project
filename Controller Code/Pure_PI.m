% PI Controller - Flow Ramp Test (No Transport Delay)
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
cfg.L_heater = 0.05;    % 50mm heater length
cfg.C_wall = 5;         % [J/K]

% Controller
Kp = 100;               % [W/C]
Ki = 1;                 % [W/C-s]
P_min = 0;
P_max = 15;

% Initial conditions
T_wall = 0.1;
integral_error = 0;

% Pre-allocate
T_wall_hist = zeros(1,N);
T_out_hist = zeros(1,N);
P_heater_hist = zeros(1,N);
Q_flow_hist = zeros(1,N);

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
    
    % Heater outlet temp
    [T_out, ~, ~, ~] = heater_dynamics(T_in, T_wall, Q_flow, 0, fluid, cfg);
    
    % PI control (no delay - sensor at heater outlet)
    err = T_setpoint - T_out;
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
    T_out_hist(i) = T_out;
    P_heater_hist(i) = P_heater;
    Q_flow_hist(i) = Q_flow;
    
    % Update heater wall
    [~, dT_wall_dt, ~, ~] = heater_dynamics(T_in, T_wall, Q_flow, P_heater, fluid, cfg);
    T_wall = T_wall + dT_wall_dt * dt;
end

% Plot
figure(1); clf;
set(gcf, 'Position', [100 100 800 600], 'Color', 'w');

subplot(4,1,1);
plot(t, Q_flow_hist, 'g', 'LineWidth', 1.5);
ylabel('Q [mL/min]');
title('PI Control - No Transport Delay');
grid on;

subplot(4,1,2);
hold on;
fill([0 t(end) t(end) 0], [T_setpoint-2 T_setpoint-2 T_setpoint+2 T_setpoint+2], ...
     [0.9 1 0.9], 'EdgeColor', 'none');
plot(t, T_out_hist, 'b', 'LineWidth', 1.5);
yline(T_setpoint, 'k--');
ylabel('T [C]');
legend('Spec', 'T_{out}', 'Setpoint', 'Location', 'northeast');
grid on;

subplot(4,1,3);
plot(t, T_wall_hist, 'r', 'LineWidth', 1.5);
ylabel('T_{wall} [C]');
grid on;

subplot(4,1,4);
plot(t, P_heater_hist, 'm', 'LineWidth', 1.5);
ylabel('P [W]');
xlabel('Time [s]');
grid on;

print('-dpng', 'pi_no_delay.png');

% Results
err_hist = T_setpoint - T_out_hist;
ramp_up = (t >= 500) & (t <= 800);
ramp_down = (t >= 800) & (t <= 1100);

fprintf('\nResults\n');
fprintf('  Kp = %.0f W/C, Ki = %.1f W/C-s\n', Kp, Ki);
fprintf('  Max undershoot (ramp up):  %.2f C\n', max(err_hist(ramp_up)));
fprintf('  Max overshoot (ramp down): %.2f C\n', -min(err_hist(ramp_down)));