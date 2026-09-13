# Final Showcase Figures

This folder contains the strongest figures for rapid technical and portfolio review. The plots retain their original validated filenames; no figures were regenerated solely for packaging.

## Controller comparison

- `final_validation_bump_summary.png` — normalized symmetric-bump metrics.
- `final_validation_kerb_summary.png` — normalized asymmetric-kerb metrics.
- `final_controller_actuator_comparison.png` — Skyhook and LQR peak-force demand.
- `final_validation_lqr_improvements.png` — LQR improvements relative to passive suspension.

## Robustness and stability

- `final_validation_robustness_stability.png` — consolidated sensitivity and stability margins.
- `final_lqr_open_vs_closed_poles.png` — passive/open-loop and LQR closed-loop poles.
- `controller_robustness_sensitivity.png` — mean response sensitivity under parameter variation.

## Operating envelope

- `final_validation_operating_envelope.png` — high-speed working-range summary.
- `lqr_high_speed_kerb_body_motion.png` — LQR body motion at 72 and 144 km/h.
- `lqr_working_range_tyre_load.png` — predicted tyre normal loads during the 144 km/h kerb.

Editable MATLAB `.fig` versions are retained alongside the PNG files. `final_controller_bump_benchmark` and `final_controller_kerb_benchmark` provide the absolute benchmark views used to support the normalized summaries.
