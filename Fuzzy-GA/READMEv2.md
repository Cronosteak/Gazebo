
# Fuzzy-GA: Fuzzy Controller optimized with Genetic Algorithm for Quadruped Robot

This repository implements a **fuzzy controller** for the locomotion and stabilization of a simulated quadruped robot (Laikago model in PyBullet). It also uses a **Genetic Algorithm (GA)** to automatically optimize the membership function parameters of the controller, thus improving the robot's performance when walking from point A to point B.

## How does it work?

1. The fuzzy controller generates commands for the robot's legs based on the state and forward error.
2. The PyBullet simulator runs the dynamics and records performance metrics.
3. A Genetic Algorithm adjusts the controller parameters to minimize an objective function based on locomotion quality.

## Basic usage

1. Install the main dependencies:
   ```bash
   pip install -r requirements.txt
   ```
2. Run a base simulation:
   ```bash
   python3 simular_fuzzy_metricas.py
   ```
3. Run the GA optimization:
   ```bash
   python3 optimizacion_fuzzy_ga.py
   ```
4. Analyze and compare results:
   ```bash
   python3 analisis_metricas.py
   ```

Author: Breinner
