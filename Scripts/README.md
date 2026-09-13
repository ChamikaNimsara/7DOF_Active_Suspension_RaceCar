# MATLAB Scripts

Run `setup_project.m` from the repository root before using the model or analysis scripts. Scripts are grouped in engineering workflow order:

1. `01_Parameters` — vehicle parameters and 14-state LQR plant construction.
2. `02_Passive_Validation` — bump, kerb, frequency-response and passive baseline analysis.
3. `03_Skyhook` — actuator-interface checks, Skyhook tuning and validation.
4. `04_LQR` — state-space checks, controllability, LQR design/tuning and stability verification.
5. `05_Operating_Envelope` — speed, high-speed kerb, working-range and robustness studies.
6. `06_Final_Validation` — final controller benchmark and consolidated validation summary.

Scripts derive repository paths from `mfilename('fullpath')`. Generated evidence is written to the matching category under `Results/` and `Plots/`; scripts should not be placed in those output folders.
