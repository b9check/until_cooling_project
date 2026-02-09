# Organ Perfusion Thermal Control

Simulation of a mobile perfusion system for loading cryoprotective agents (CPA) into porcine kidneys during transport. Developed for Until Labs.

## Problem

Maintain 4°C ± 2°C at the kidney inlet while flow varies 10× (3-30 mL/min) due to pressure control. A flow-dependent transport delay (1-11s) between heater and sensor makes pure PI control fail.

## Solution

Feedforward + PI: feedforward computes heater power from flow rate (P = ṁ·Cp·ΔT), PI trims residual error.

## Results

- Temp max error: 0.23°C (spec ±2°C) ✓
- Pressure max error: 0.87 mmHg (spec ±3 mmHg) ✓

## Files

- main.m - Full 30-min simulation
- run_simulation.m - Physics + control loop
- heater_dynamics.m - NTU-effectiveness thermal model
- Pure_PI.m - PI without transport delay (baseline)
- Pure_PI_Lag.m - PI with delay, no feedforward (fails)
- plot_dynamics.m - Heater response visualization

## Author

Brian Check - Stanford MS Aero/Astro
