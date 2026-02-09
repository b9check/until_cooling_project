function [T_out, dT_wall_dt, Q_to_fluid, eps] = heater_dynamics(T_in, T_wall, Q_flow, P_heater, fluid, cfg)
% HEATER_DYNAMICS - NTU-effectiveness model for inline heater
    % Geometry
    D = cfg.D_tube;
    L = cfg.L_heater;
    C_wall = cfg.C_wall;
    
    % Convert flow rate
    Q = Q_flow * 1e-6 / 60;         % [mL/min] -> [m^3/s]
    m_dot = fluid.rho * Q;          % [kg/s]
    
    % Handle zero flow
    if Q < 1e-12
        T_out = T_in;
        Q_to_fluid = 0;
        dT_wall_dt = P_heater / C_wall;
        eps = 0;
        return;
    end
    
    % Dimensionless numbers
    Re = (4 * fluid.rho * Q) / (pi * D * fluid.mu);
    Pr = (fluid.mu * fluid.Cp) / fluid.k;
    Gz = (D / L) * Re * Pr;
    Nu = 3.66 + (0.0668 * Gz) / (1 + 0.04 * Gz^(2/3));
    
    % Heat transfer
    h = Nu * fluid.k / D;
    NTU = (h * pi * D * L) / (m_dot * fluid.Cp);
    eps = 1 - exp(-NTU);
    
    Q_to_fluid = m_dot * fluid.Cp * eps * (T_wall - T_in);
    T_out = T_in + eps * (T_wall - T_in);
    dT_wall_dt = (P_heater - Q_to_fluid) / C_wall;
end
