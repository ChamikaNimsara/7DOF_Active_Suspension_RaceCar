# Final Engineering Summary

## 1. Problem definition

Race-vehicle suspension design requires a compromise between body control, wheel control, vertical acceleration, tyre loading and available suspension travel. Passive springs and dampers must address all of these objectives with fixed mechanical characteristics. Increasing damping may reduce body oscillation and settling time but transmit more high-frequency acceleration; reducing compliance may improve platform control but worsen wheel-load variation over short road inputs. Active suspension introduces an additional force at each corner, allowing the effective response to change independently of the passive hardware, but it also introduces requirements for state information, actuator capacity, control bandwidth, power and stability.

This project examined that compromise using a MATLAB/Simulink full-car vertical-dynamics model. The work compared a validated passive baseline with optimized Skyhook control and full-state linear-quadratic regulator control. Evaluation extended beyond a single bump response to include asymmetric kerb excitation, frequency response, controller tuning sweeps, vehicle-speed sensitivity, parameter uncertainty, closed-loop pole analysis, suspension working range and tyre normal-load indicators.

The selected controller was the Final Tuned LQR. Selection was based on its overall balance of heave, pitch, roll, settling, robustness and actuator force. It was not selected because it was best in every metric, and the conclusions do not assume that reduced body motion automatically produces greater tyre grip.

## 2. Vehicle and suspension model

The model represents seven vertical-dynamics degrees of freedom. The sprung mass has heave, pitch and roll motion. Each of the four unsprung corner masses has one vertical wheel-hop degree of freedom. The corresponding LQR representation contains 14 states: seven positions or angular displacements followed by their seven velocities.

The nominal vehicle mass is 850 kg, comprising a 785 kg sprung mass, 15 kg unsprung mass at each front corner and 17.5 kg at each rear corner. The wheelbase is 3.50 m, with the sprung-mass centre of gravity 1.80 m behind the front axle. Front and rear track widths are 1.75 m and 1.85 m. The model uses a roll inertia of 300 kg·m² and pitch inertia of 1350 kg·m².

Front suspension stiffness is 67,500 N/m per corner with 1400 N·s/m damping. Rear suspension stiffness is 100,000 N/m per corner with 1800 N·s/m damping. Tyres are represented by a linear vertical stiffness of 400,000 N/m per corner. The model includes front and rear anti-roll stiffness and maps the four suspension-corner forces into sprung-mass heave, pitch and roll through the axle and track geometry.

Four independent road-displacement inputs act beneath the tyres. This permits symmetric excitation, where corresponding left/right inputs are equal, and asymmetric left-side kerb excitation, which generates coupled heave, pitch and roll. Vehicle speed determines road-input duration and the time delay between front- and rear-axle excitation for a fixed-length feature.

An actuator is included between the sprung and unsprung masses at each corner. The equal-and-opposite reaction on the two masses is essential to the sign convention. Each command is saturated at ±3000 N. Internal hydraulic or electric actuator dynamics, force-rate limits, delay, power consumption and thermal behaviour are not represented.

## 3. State-space and control approach

The full-state model uses sprung heave, pitch and roll; four unsprung displacements; sprung heave, pitch and roll rates; and four unsprung velocities. The input vector contains front-left, front-right, rear-left and rear-right actuator forces. Four tyre road displacements form the disturbance vector.

The passive configuration sets all active forces to zero. Before controller development, the actuator interface was simulated in its disabled state and compared with the stored passive baseline. Peak heave, peak pitch, RMS heave acceleration and RMS pitch acceleration all showed 0.000% error. This check demonstrated that adding the actuator paths had not modified the passive plant when no force was requested.

Skyhook control was used as an interpretable active baseline. A gain sweep evaluated peak heave, peak pitch, RMS heave acceleration, RMS pitch acceleration, settling time and actuator effort. A weighted objective selected a gain of 6000 N·s/m. Under the nominal symmetric bump this controller used 1720.7 N peak actuator force without saturation.

The LQR controller was designed from the 14-state continuous-time model. Mechanical states have substantially different numerical scales, so controller design used normalized coordinates. Controllability required careful interpretation. A direct controllability-matrix calculation returned a numerical rank of 8/14 with a condition number of 1.13 × 10¹⁹. Scaling reduced the condition number to 2.32 × 10¹⁷, although the default rank estimate remained tolerance-sensitive. A PBH test evaluated controllability at each system eigenvalue and returned a minimum rank of 14/14 before and after scaling. The system was therefore treated as controllable, with the lower direct rank attributed to numerical conditioning rather than inaccessible physical modes.

The LQR Q/R scaling was investigated systematically. Symmetric-bump candidates from 0.125 to 1.0 showed the expected trade: higher state weighting slightly reduced some peak motions but increased acceleration and actuator demand. For example, increasing the ratio from 0.125 to 1.0 increased peak bump force from 937.9 N to 2539.8 N. The two strongest bump candidates were then compared on the asymmetric kerb. Ratios 0.125 and 0.250 produced combined objectives of 0.7972 and 0.8118, respectively. The 0.125 case was retained as the Final Tuned LQR because it achieved the lower combined objective and required less force.

## 4. Validation methodology

The nominal symmetric bump was 25 mm high and 0.50 m long at 72 km/h. This excitation was used to quantify peak sprung-mass heave and pitch, RMS heave and pitch acceleration, settling time and maximum absolute actuator force. A passive frequency sweep from 0.5 to 25 Hz identified a dominant body response near 2.85 Hz and a dominant wheel response near 23.75 Hz, providing a dynamic check beyond transient metrics.

The asymmetric test used a 30 mm-high, 0.60 m-long left-side kerb. Independent corner inputs excited roll as well as heave and pitch. Metrics included peak heave, pitch and roll, RMS roll acceleration, roll settling and actuator demand. Passive, Skyhook and LQR cases used common road geometry, simulation windows, signal definitions and saturation limits.

Controller robustness was assessed at 72 km/h using nominal parameters, +10% sprung mass, −10% suspension stiffness, −10% damping and −10% tyre stiffness. Stability was checked separately by evaluating closed-loop eigenvalues for the same parameter changes. Operating-envelope analysis included fixed-length bump sensitivity from 36 to 144 km/h, comparison of the asymmetric kerb at 72 and 144 km/h, and detailed suspension, tyre and actuator histories at 144 km/h.

## 5. Controller comparison

For the symmetric bump, passive suspension produced 2.3982 mm peak heave, 0.16387° peak pitch and a 2.106 s settling time. Optimized Skyhook reduced these values to 2.1434 mm, 0.082475° and 1.397 s, with 1720.7 N peak force. Final Tuned LQR produced 2.1057 mm peak heave, 0.11169° peak pitch and a 1.431 s settling time with 937.9 N peak force.

Relative to passive suspension, LQR reduced bump peak heave by 12.20%, peak pitch by 31.84% and settling time by 32.05%. Skyhook achieved the lower peak pitch and slightly shorter settling time, but its peak force was approximately 83% greater than LQR. LQR also did not improve the symmetric-bump acceleration metrics: RMS heave acceleration increased from 0.94096 to 0.97314 m/s² and RMS pitch acceleration increased from 55.858 to 56.628 deg/s². Skyhook produced the lowest acceleration values of the three cases.

For the asymmetric kerb, the passive response reached 1.7215 mm heave, 0.11662° pitch, 0.20380° roll and 1.844 s roll settling. Optimized Skyhook produced 1.5172 mm, 0.057550°, 0.14509° and 1.455 s with 1903.0 N peak force. LQR produced 1.4934 mm heave, 0.078727° pitch, 0.14662° roll and 1.308 s roll settling with 1109.1 N peak force.

The LQR improvements relative to passive were 13.25% in peak heave, 32.49% in peak pitch, 28.06% in peak roll and 29.07% in roll settling. Skyhook remained marginally better in peak roll and substantially better in peak pitch, but LQR achieved the lowest heave, shortest roll settling and approximately 42% lower peak force than Skyhook.

These results support the LQR selection as a compromise controller. It improves all headline displacement and settling metrics relative to passive suspension while using substantially less force than Skyhook. The symmetric-bump acceleration penalty remains a clear limitation of the selected weighting.

## 6. Robustness and closed-loop stability

Both active controllers remained stable in all tested parameter cases and neither reached actuator saturation. Final Tuned LQR had a mean absolute response sensitivity of 2.6966%, compared with 2.8570% for optimized Skyhook. The largest LQR force observed during the robustness sweep was 959.89 N; the corresponding worst Skyhook value was 1720.7 N.

Nominal LQR closed-loop poles all had negative real parts. The maximum real part was −19.1488 s⁻¹, giving a nominal stability margin of 19.1488 s⁻¹. The +10% sprung-mass case was the least stable tested variation, but its maximum pole real part remained −17.0761 s⁻¹. The resulting worst-case margin of 17.0761 s⁻¹ confirms stability for the defined uncertainty set. These results are local linear-model results and should not be extended to unmodelled nonlinearities, actuator dynamics or arbitrary parameter variation.

## 7. Operating-envelope assessment

For a fixed-length road feature, increasing speed shortens the excitation duration and front-to-rear delay. In the symmetric-bump sweep from 36 to 144 km/h, peak heave, peak pitch and actuator demand reduced as speed increased. At 144 km/h, bump peak force was 542.2 N compared with 1276.9 N at 36 km/h. This trend does not mean that the faster event is mechanically less severe; the shorter input produces higher wheel and suspension velocities even when low-frequency body displacement is smaller.

The high-speed kerb showed the same distinction. At 144 km/h, LQR peak body motion and force were lower than at 72 km/h, with 0.9224 mm peak heave, 0.04074° peak pitch, 0.08129° peak roll and 656.0 N peak force. Detailed working-range results showed maximum suspension deflection of 20.298 mm, maximum relative suspension velocity of 2.668 m/s, maximum tyre deflection of 26.392 mm and maximum actuator utilization of 21.865%.

The tyre-load calculation identified the principal operating-envelope concern. The front-left and rear-left tyres, which crossed the kerb, produced minimum predicted normal loads of −6603.4 N and −5569.8 N. The longest zero-or-negative-load indication was 0.25995% of the simulation. A negative normal load cannot be produced by a real tyre-road contact; it indicates that the bilateral linear tyre spring has extrapolated beyond physical contact. Consequently, the result is reported as predicted transient unloading/contact loss and a model-validity warning, not as measured wheel lift or a quantified loss of grip.

## 8. Limitations and future development

The model is linear in its suspension and tyre force relationships. It does not include nonlinear damper curves, bump stops, droop limits, friction, suspension geometry variation, compliance, tyre enveloping or unilateral tyre contact. Lateral, longitudinal and yaw dynamics are excluded, so the model cannot predict combined-load tyre forces, handling balance or lap time. Aerodynamic load coupling is limited, and no actuator dynamics, state estimator, sensor noise or implementation delay is included. Results have not yet been correlated against rig measurements, telemetry or track testing.

The next modelling priority is unilateral tyre contact with nonlinear vertical stiffness, followed by nonlinear damper and end-stop behaviour. An actuator model should introduce bandwidth, force-rate, delay, power and thermal constraints. A state observer or Kalman filter would replace the ideal full-state assumption. Further controller work could then compare semi-active strategies, model-predictive control and adaptive control using identical constraints. Coupling the vertical model to aerodynamics, nonlinear tyre forces and lap simulation would permit investigation of platform control and tyre loading within a wider vehicle-performance context. Hardware-in-the-loop and physical validation would be required before implementation claims.

## 9. Final conclusion

The project demonstrates, within the validated linear simulation scope, that a full-state LQR controller can improve race-car body control while requiring substantially less actuator effort than optimized Skyhook control. Final Tuned LQR reduced heave, pitch, roll and settling metrics relative to the passive baseline, remained stable across all tested parameter variations and stayed far below the ±3000 N actuator limit.

The controller is not superior in every response. Skyhook achieved better pitch and acceleration results in several tests, and LQR slightly worsened the symmetric-bump RMS acceleration metrics relative to passive suspension. LQR was selected because it provided the strongest combined balance of body-motion reduction, transient settling, robustness and actuator efficiency.

The severe 144 km/h kerb case showed that actuator saturation was not the limiting factor. Predicted tyre unloading was the more important concern and exposed the boundary of the linear tyre model. The appropriate conclusion is therefore conditional: the active controller improves the evaluated body-motion compromise, but higher-fidelity tyre/contact modelling and physical validation are necessary before drawing conclusions about grip or real-track performance.
