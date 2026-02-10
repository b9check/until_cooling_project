% Perfusion System Simulation
% Mobile CPA Loading System for Porcine Kidney
clear; clc;

% --- Timing ---
cfg.dt = 0.1;                   % [s]
cfg.t_end = 1800;               % 30 min

% --- Protocol ---
cfg.t_equilibrate = 300;        % [s]
cfg.t_ramp = 1000;              % [s]
cfg.cpa_final = 0.7;            % 70% CPA

% --- Vascular resistance ---
cfg.R_initial = 9;             % [mmHg/(mL/min)]
cfg.R_final = 2;

% --- Setpoints & tolerances ---
cfg.T_setpoint = 4.0;           % [C]
cfg.P_setpoint = 30.0;          % [mmHg]
cfg.T_tolerance = 2.0;          % [C]
cfg.P_tolerance = 3.0;          % [mmHg]

% --- Flow limits ---
cfg.Q_min = 3;                  % [mL/min]
cfg.Q_max = 30;

% --- Geometry ---
cfg.D_tube = 0.003;             % 3mm ID
cfg.L_heater = 0.13;            % 130mm heated section
cfg.L_sensor = 0.08;            % 80mm heater-to-sensor

% --- Heater ---
cfg.C_wall = 5.0;               % [J/K]
cfg.P_heater_min = 0;
cfg.P_heater_max = 15;          % [W]

% --- Carrier fluid ---
cfg.rho_carrier = 1000;         % [kg/m^3]
cfg.Cp_carrier = 4217;          % [J/kg-K]
cfg.k_carrier = 0.561;          % [W/m-K]
cfg.mu_carrier = 0.00167;       % [Pa-s]

% --- CPA fluid ---
cfg.rho_cpa = 1100;
cfg.Cp_cpa = 3500;
cfg.k_cpa = 0.50;
cfg.mu_cpa = 0.005;

% --- Temperature controller ---
cfg.Kp_T = 0.1;                 % [W/C]
cfg.Ki_T = 0.001;               % [W/C-s]
cfg.use_feedforward = true;

% --- Pressure controller ---
cfg.Kp_P = 0.05;                % [mL/min/mmHg]
cfg.Ki_P = 0.1;

% --- Sensors ---
cfg.T_bias = 0.1;               % [C]
cfg.T_noise = 0.1;              % [C]
cfg.P_bias = 0.5;               % [mmHg]
cfg.P_noise = 0.3;              % [mmHg]

% --- Ice bath ---
cfg.T_in_nominal = 0.1;         % [C]
cfg.T_in_drift = 0.0005;        % [C/s]
cfg.T_in_noise = 0.02;          % [C]

% --- Run ---
fprintf('Running...\n');
results = run_simulation(cfg);
fprintf('Done.\n\n');

% --- Print results ---
V_delay = pi * (cfg.D_tube/2)^2 * cfg.L_sensor * 1e6;

fprintf('Protocol\n');
fprintf('  Duration:        %d min\n', cfg.t_end/60);
fprintf('  CPA ramp:        %d s (0%% -> %.0f%%)\n', cfg.t_ramp, cfg.cpa_final*100);
fprintf('  R_vascular:      %.0f -> %.0f mmHg/(mL/min)\n', cfg.R_initial, cfg.R_final);
fprintf('  Transport delay: %.1f - %.1f s\n\n', V_delay/(30/60), V_delay/(3/60));

fprintf('Temperature\n');
fprintf('  Max error:  %.2f C', results.T_error_max);
if results.T_error_max <= cfg.T_tolerance
    fprintf('  PASS\n');
else
    fprintf('  FAIL\n');
end
fprintf('  Mean error: %.2f C\n\n', results.T_error_mean);

fprintf('Pressure\n');
fprintf('  Max error:  %.2f mmHg', results.P_error_max);
if results.P_error_max <= cfg.P_tolerance
    fprintf('  PASS\n');
else
    fprintf('  FAIL\n');
end
fprintf('  Mean error: %.2f mmHg\n\n', results.P_error_mean);

fprintf('Heater: %.1f W max (%.0f%% utilization)\n\n', ...
        results.P_heater_max, 100*results.P_heater_max/cfg.P_heater_max);
fprintf('Heater mean: %.2f W\n\n', results.P_heater_mean);

% --- Plot ---
t = results.t;

figure(1); clf;
set(gcf, 'Position', [100 100 900 800], 'Color', 'w');

subplot(4,1,1);
plot(t, results.R_vascular, 'r', 'LineWidth', 1.5);
ylabel('R [mmHg/(mL/min)]');
title('Inputs & Actuators');
ylim([0 12]); grid on;

subplot(4,1,2);
plot(t, results.T_in, 'c', 'LineWidth', 1.5);
ylabel('T_{in} [C]');
ylim([-0.5 2]); grid on;

subplot(4,1,3);
plot(t, results.P_heater, 'm', 'LineWidth', 1.5);
ylabel('P [W]');
ylim([0 12]); grid on;

subplot(4,1,4);
plot(t, results.Q_carrier, 'b', 'LineWidth', 1.5); hold on;
plot(t, results.Q_cpa, 'r', 'LineWidth', 1.5);
plot(t, results.Q_total, 'k--', 'LineWidth', 1.5);
ylabel('Q [mL/min]');
xlabel('Time [s]');
ylim([0 35]); grid on;
legend('Carrier', 'CPA', 'Total', 'Location', 'east');

print('-dpng', '-r150', 'fig1_inputs.png');

figure(2); clf;
set(gcf, 'Position', [100 100 900 700], 'Color', 'w');

subplot(3,1,1);
hold on;
fill([t(1) t(end) t(end) t(1)], ...
     [cfg.T_setpoint-cfg.T_tolerance cfg.T_setpoint-cfg.T_tolerance ...
      cfg.T_setpoint+cfg.T_tolerance cfg.T_setpoint+cfg.T_tolerance], ...
     [0.9 1 0.9], 'EdgeColor', 'none');
plot(t, results.T_out, 'b', 'LineWidth', 1.5);
plot(t, results.T_sensor, 'c--', 'LineWidth', 1);
yline(cfg.T_setpoint, 'k--');
ylabel('T [C]');
title('Outputs');
ylim([0 8]); grid on;
legend('Spec', 'True', 'Sensor', 'Location', 'northeast');

subplot(3,1,2);
hold on;
fill([t(1) t(end) t(end) t(1)], ...
     [cfg.P_setpoint-cfg.P_tolerance cfg.P_setpoint-cfg.P_tolerance ...
      cfg.P_setpoint+cfg.P_tolerance cfg.P_setpoint+cfg.P_tolerance], ...
     [0.9 1 0.9], 'EdgeColor', 'none');
plot(t, results.pressure, 'b', 'LineWidth', 1.5);
plot(t, results.P_sensor, 'c--', 'LineWidth', 1);
yline(cfg.P_setpoint, 'k--');
ylabel('P [mmHg]');
ylim([20 40]); grid on;
legend('Spec', 'True', 'Sensor', 'Location', 'northeast');

subplot(3,1,3);
plot(t, results.cpa_ratio * 100, 'm', 'LineWidth', 1.5);
ylabel('CPA [%]');
xlabel('Time [s]');
ylim([0 80]); grid on;

print('-dpng', '-r150', 'fig2_outputs.png');

fprintf('Figures saved.\n');