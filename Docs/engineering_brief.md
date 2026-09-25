# Engineering Brief — 7-DOF Active Suspension for a Race Vehicle

> **Outcome:** A tuned full-state LQR improved the evaluated body-motion and settling metrics while using 42–45% less peak actuator force than optimized Skyhook. The main operating-envelope concern was tyre unloading during a severe high-speed kerb, where the linear contact model became non-physical.

| Project snapshot | |
|---|---|
| **Objective** | Compare passive, Skyhook and LQR suspension control on a full-car vertical-dynamics model. |
| **Tools** | MATLAB, Simulink, Control System Toolbox |
| **Plant** | 7 DOF: sprung-body heave, pitch and roll plus four wheel-hop modes; 14-state LQR representation |
| **Inputs / control** | Four independent road inputs and four active corner forces, each limited to ±3000 N |
| **Evaluation** | Symmetric bump, asymmetric kerb, frequency response, speed sensitivity, parameter uncertainty, pole stability and working range |

## Engineering approach

The model couples front/rear suspension stiffness and damping, linear tyre vertical stiffness and vehicle geometry across all four corners. A passive baseline was established first. The four-corner actuator interface was then verified with zero commands, reproducing the passive headline metrics with **0.000% error**. This check confirmed the equal-and-opposite actuator-force convention before controller tuning.

Optimized Skyhook uses body-velocity feedback with a selected gain of **6000 N·s/m**. The LQR uses normalized full-state feedback. Controllability was checked with a PBH test because the conventional controllability matrix was poorly conditioned; every eigenvalue returned rank **14/14**. A bump and kerb weight sweep selected a final Q/R scaling ratio of **0.125**, balancing body control, settling and force demand.

## Results

| Test metric | Passive | Final LQR | Change vs passive |
|---|---:|---:|---:|
| Bump peak heave | 2.3982 mm | 2.1057 mm | **−12.20%** |
| Bump peak pitch | 0.16387° | 0.11169° | **−31.84%** |
| Bump settling time | 2.106 s | 1.431 s | **−32.05%** |
| Kerb peak heave | 1.7215 mm | 1.4934 mm | **−13.25%** |
| Kerb peak pitch | 0.11662° | 0.07873° | **−32.49%** |
| Kerb peak roll | 0.20380° | 0.14662° | **−28.06%** |
| Kerb roll settling time | 1.844 s | 1.308 s | **−29.07%** |

Peak LQR force was **937.9 N** for the bump and **1109.1 N** for the kerb, versus **1720.7 N** and **1903.0 N** for Skyhook. LQR mean response sensitivity was **2.697%** across the tested parameter variations, slightly below Skyhook’s **2.857%**. All tested LQR cases remained stable; the pole-based stability margin was **19.149 s⁻¹** nominally and **17.076 s⁻¹** in the worst tested case.

## Engineering judgement

LQR was selected as the best overall compromise, not because it won every metric. Skyhook produced lower pitch and acceleration in several comparisons. Relative to passive suspension, LQR increased symmetric-bump RMS heave acceleration by about **3.4%** and RMS pitch acceleration by about **1.4%**. This is the central tuning trade-off: stronger platform control and faster settling can transmit sharper force changes and slightly worsen acceleration response.

At 144 km/h on the asymmetric kerb, actuator utilization remained only **21.87%**, but the linear tyre model predicted a minimum normal load of **−6603 N** for a brief interval. A tyre cannot sustain tensile road contact, so this value marks failure of the bilateral linear-contact assumption; it does not prove physical wheel lift or controller instability.

## Scope and next step

Results are simulation-based. The model excludes unilateral/nonlinear tyre contact, nonlinear dampers and end stops, actuator bandwidth and power, sensor noise and state estimation, coupled lateral/longitudinal dynamics, and physical validation. The next priority is a unilateral nonlinear tyre-contact model, followed by actuator dynamics and aerodynamic platform coupling before hardware-in-the-loop or track claims.

**Evidence:** [system architecture](Figures/system_architecture.svg) · [final validation summary](../Results/Final/final_validation_summary.txt) · [controller benchmark](../Results/Final/final_controller_benchmark.txt) · [full engineering summary](Technical_Report/final_engineering_summary.md)

