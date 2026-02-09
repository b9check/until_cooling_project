%% PLOT_HEATER_RESPONSE - Simple heater dynamics visualization
clear; clc; close all;


%% Configuration (matching main.m)
cfg.D_tube = 0.003;         
cfg.L_heater = 0.13;      
cfg.C_wall = 5.0;  

% Fluid properties 
fluid.rho = 1000;           % [kg/m^3]
fluid.Cp = 4217;            % [J/kg-K]
fluid.k = 0.561;            % [W/m-K]
fluid.mu = 0.00167;         % [Pa-s]

% Constant inputs
T_in = 0.1;                 % [C] inlet temperature (from ice bath)
Q_flow = 15;                % [mL/min] flow rate (mid-range)
P_heater = 8;               % [W] heater power


%% Simulation
dt = 0.1;                   % [s] time step
t_end = 120;                % [s] duration
t = 0:dt:t_end;
N = length(t);

T_wall = zeros(N, 1);
T_out = zeros(N, 1);
T_wall(1) = T_in;           % Initial condition

for i = 1:N-1
    [T_out(i), dT_wall_dt, ~, ~] = heater_dynamics(T_in, T_wall(i), Q_flow, P_heater, fluid, cfg);
    T_wall(i+1) = T_wall(i) + dT_wall_dt * dt;
end
[T_out(N), ~, ~, ~] = heater_dynamics(T_in, T_wall(N), Q_flow, P_heater, fluid, cfg);


%% Plot
figure('Position', [100, 100, 700, 400]);
plot(t, T_wall, 'b-', 'LineWidth', 2); hold on;
plot(t, T_out, 'r-', 'LineWidth', 2);
yline(T_in, 'c--', 'LineWidth', 1.5);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Temperature [°C]', 'FontSize', 12);
title(sprintf('Heater Response: Q = %.0f mL/min, P = %.0f W', Q_flow, P_heater), 'FontSize', 14);
legend('T_{wall}', 'T_{out}', 'T_{in}', 'Location', 'southeast', 'FontSize', 11);
grid on;
xlim([0 t_end]);
ylim([0 max(T_wall)*1.1]);

fprintf('Steady-state: T_wall = %.2f°C, T_out = %.2f°C\n', T_wall(end), T_out(end));