function results = run_simulation(cfg)
% Main simulation loop for perfusion system

    % Setup
    dt = cfg.dt;
    t = 0:dt:cfg.t_end;
    N = length(t);
    
    % Transport delay volume
    V_sensor = pi * (cfg.D_tube/2)^2 * cfg.L_sensor;
    
    % Initial states
    T_wall = cfg.T_in_nominal;
    T_integral = 0;
    P_integral = 0;
    
    % Pre-allocate
    results.t = t;
    results.T_out = zeros(1,N);
    results.T_sensor = zeros(1,N);
    results.T_wall = zeros(1,N);
    results.T_in = zeros(1,N);
    results.P_heater = zeros(1,N);
    results.Q_total = zeros(1,N);
    results.Q_cpa = zeros(1,N);
    results.Q_carrier = zeros(1,N);
    results.pressure = zeros(1,N);
    results.P_sensor = zeros(1,N);
    results.cpa_ratio = zeros(1,N);
    results.R_vascular = zeros(1,N);
    results.T_out(:) = cfg.T_in_nominal;
    
    % Main loop
    for i = 1:N
        ti = t(i);
        
        % --- Protocol: CPA ratio and vascular resistance ---
        if ti < cfg.t_equilibrate
            % Equilibration with pure carrier
            cpa_ratio = 0;
            R = cfg.R_initial;
        elseif ti < cfg.t_equilibrate + cfg.t_ramp
            % Ramp CPA, resistance drops
            frac = (ti - cfg.t_equilibrate) / cfg.t_ramp;
            cpa_ratio = cfg.cpa_final * frac;
            R = cfg.R_initial + (cfg.R_final - cfg.R_initial) * frac;
        else
            % Hold
            cpa_ratio = cfg.cpa_final;
            R = cfg.R_final;
        end
        results.cpa_ratio(i) = cpa_ratio;
        results.R_vascular(i) = R;
        
        % --- Fluid properties (vary with CPA concentration) ---
        fluid.rho = cfg.rho_carrier + (cfg.rho_cpa - cfg.rho_carrier) * cpa_ratio;
        fluid.Cp = cfg.Cp_carrier + (cfg.Cp_cpa - cfg.Cp_carrier) * cpa_ratio;
        fluid.k = cfg.k_carrier + (cfg.k_cpa - cfg.k_carrier) * cpa_ratio;
        fluid.mu = cfg.mu_carrier + (cfg.mu_cpa - cfg.mu_carrier) * cpa_ratio;
        
        % --- Ice bath ---
        T_in = cfg.T_in_nominal + cfg.T_in_drift * ti;
        if cfg.T_in_noise > 0
            T_in = T_in + cfg.T_in_noise * randn();
        end
        results.T_in(i) = T_in;
        
        
        % --- Pressure control -> flow (PI feedback only) ---
        if i > 1
            P_true = results.Q_total(i-1) * R;  
        else
            P_true = cfg.P_setpoint;
        end
        
        P_sensor = P_true + cfg.P_bias;
        if cfg.P_noise > 0
            P_sensor = P_sensor + cfg.P_noise * randn();
        end
        results.P_sensor(i) = P_sensor;
        
        P_error = cfg.P_setpoint - P_sensor;
        P_integral = P_integral + P_error * dt;
        Q_nominal = cfg.P_setpoint / cfg.R_initial;
        Q_total = Q_nominal + cfg.Kp_P * P_error + cfg.Ki_P * P_integral;
        
        % Clamp with anti-windup
        if Q_total > cfg.Q_max
            Q_total = cfg.Q_max;
            P_integral = P_integral - P_error * dt;
        elseif Q_total < cfg.Q_min
            Q_total = cfg.Q_min;
            P_integral = P_integral - P_error * dt;
        end
        
        results.Q_total(i) = Q_total;
        results.pressure(i) = Q_total * R;
        
        % --- Split flow between pumps ---
        results.Q_cpa(i) = Q_total * cpa_ratio;
        results.Q_carrier(i) = Q_total * (1 - cpa_ratio);
        
        % --- Transport delay ---
        Q_m3s = Q_total * 1e-6 / 60;
        if Q_m3s > 1e-12
            delay_samples = round(V_sensor / Q_m3s / dt);
        else
            delay_samples = 1000;
        end
        
        % --- Heater dynamics ---
        [T_out, ~, ~, ~] = heater_dynamics(T_in, T_wall, Q_total, 0, fluid, cfg);
        results.T_out(i) = T_out;
        
        % --- Temperature sensor (delayed + noise) ---
        idx_delayed = max(1, i - delay_samples);
        T_sensor = results.T_out(idx_delayed) + cfg.T_bias;
        if cfg.T_noise > 0
            T_sensor = T_sensor + cfg.T_noise * randn();
        end
        results.T_sensor(i) = T_sensor;
        
        % --- Temperature control -> heater power ---
        if cfg.use_feedforward
            m_dot = fluid.rho * Q_m3s;
            P_ff = m_dot * fluid.Cp * (cfg.T_setpoint - T_in);
        else
            P_ff = 0;
        end
        
        T_error = cfg.T_setpoint - T_sensor;
        T_integral = T_integral + T_error * dt;
        P_fb = cfg.Kp_T * T_error + cfg.Ki_T * T_integral;
        
        P_heater = P_ff + P_fb;
        
        % Clamp with anti-windup
        if P_heater > cfg.P_heater_max
            P_heater = cfg.P_heater_max;
            T_integral = T_integral - T_error * dt;
        elseif P_heater < cfg.P_heater_min
            P_heater = cfg.P_heater_min;
            T_integral = T_integral - T_error * dt;
        end
        results.P_heater(i) = P_heater;
        
        % --- Update heater wall state ---
        [~, dT_wall_dt, ~, ~] = heater_dynamics(T_in, T_wall, Q_total, P_heater, fluid, cfg);
        T_wall = T_wall + dT_wall_dt * dt;
        results.T_wall(i) = T_wall;
    end
    
    % --- Compute metrics ---
    
    % Exclude startup transient
    t_startup = 0.2 * cfg.t_equilibrate;
    op_idx = t >= t_startup;
    
    % Temperature
    T_error = cfg.T_setpoint - results.T_out;
    results.T_error_max = max(abs(T_error(op_idx)));
    results.T_error_mean = mean(abs(T_error(op_idx)));
    results.T_in_spec_pct = 100 * mean(abs(T_error(op_idx)) <= cfg.T_tolerance);
    
    % Pressure
    P_error = cfg.P_setpoint - results.pressure;
    results.P_error_max = max(abs(P_error(op_idx)));
    results.P_error_mean = mean(abs(P_error(op_idx)));
    results.P_in_spec_pct = 100 * mean(abs(P_error(op_idx)) <= cfg.P_tolerance);
    
    % Power
    results.P_heater_max = max(results.P_heater);
    results.P_heater_mean = mean(results.P_heater(op_idx));
end