%% PLOT_COOLING_RESPONSE - Ice bath coil cooling verification
clear; clc; close all;

%% Configuration (cooling coil in ice bath)
cfg.D_tube = 0.003;         % 3mm ID
cfg.L_heater = 2.0;         % 2m of tubing in ice bath
cfg.C_wall = 5.0;           % Not used - ice bath holds wall at 0

% Fluid properties
fluid.rho = 1000;           % [kg/m^3]
fluid.Cp = 4217;            % [J/kg-K]
fluid.k = 0.561;            % [W/m-K]
fluid.mu = 0.00167;         % [Pa-s]

% Inputs for COOLING (not heating)
T_in = 20;                  % [C] warm fluid entering (worst case)
Q_flow = 30;                % [mL/min] max flow rate (worst case)
P_heater = 0;               % [W] no heater - passive cooling
T_wall = 0;                 % [C] ice bath holds wall at 0°C (constant)


%% Steady-state calculation (no dynamics needed)
[T_out, ~, ~, eps] = heater_dynamics(T_in, T_wall, Q_flow, P_heater, fluid, cfg);

fprintf('=== COOLING VERIFICATION ===\n');
fprintf('Coil length: %.1f m\n', cfg.L_heater);
fprintf('Flow rate: %.0f mL/min (worst case)\n', Q_flow);
fprintf('Fluid in: %.1f°C\n', T_in);
fprintf('Fluid out: %.2f°C\n', T_out);


% Sweep coil length to find minimum
lengths = 0.5:0.5:5;
T_outs = zeros(size(lengths));

for i = 1:length(lengths)
    cfg.L_heater = lengths(i);
    [T_outs(i), ~, ~, ~] = heater_dynamics(T_in, T_wall, Q_flow, P_heater, fluid, cfg);
end

figure('Position', [100, 100, 700, 400]);
plot(lengths, T_outs, 'b-o', 'LineWidth', 2);
yline(4, 'g--', 'LineWidth', 1.5);
yline(0, 'c--', 'LineWidth', 1.5);
xlabel('Coil Length [m]', 'FontSize', 12);
ylabel('T_{out} [°C]', 'FontSize', 12);
title(sprintf('Cooling vs Coil Length (Q = %.0f mL/min, T_{in} = %.0f°C)', Q_flow, T_in), 'FontSize', 14);
legend('T_{out}', 'Target (4°C)', 'Ice bath (0°C)', 'Location', 'northeast');
grid on;