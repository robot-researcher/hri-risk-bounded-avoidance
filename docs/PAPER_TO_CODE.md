# Paper-to-code and evidence guide

Companion to *Offline-Certified Risk-Bounded Collision Avoidance for Robotic Manipulators Under Encoder Uncertainty*, manuscript in preparation.

## Function responsibilities

| Component | Entry point | Responsibility |
| --- | --- | --- |
| Arm model | `PlanarArm3DOF.joint_positions`, `endpoint_jacobian`, `point_jacobian` in `kinematics.py` | Planar link positions and Jacobians used in geometry and directional uncertainty propagation. |
| Primitive generation | `quintic_primitive`, `generate_primitives` in `primitives.py` | Generate repeatable sampled joint-space motion candidates. |
| Collision geometry | `CapsuleArmGeometry.minimum_clearance`, `collides` in `geometry.py` | Capsule-link clearance against circular obstacles. |
| Nominal swept clearance | `certify_swept_clearance` in `swept.py` | Account for motion between samples using conservative link displacement margins. |
| Complete-link risk model | `RiskCertifier.continuous_swept_risk`, `build_library` in `risk.py` | Precompute risk values indexed by primitive and clearance bin. |
| Directional comparison | `DirectionalRiskCertifier.configuration_risk` in `risk.py` | First-order directional risk surrogate, separate from the complete-link model. |
| Online selection | `PrimitiveSelector.select` in `selector.py` | Select from risk-feasible candidates using a conservative bin and bounded goal ranking. |
| Policy | `adaptive_risk_threshold` in `risk.py` | Compute the threshold from modeled clearance and covariance. |
| Replay guards | `SafetyRuntime.step` in `runtime.py` | Hold on tested stale observations, infeasibility and incorrect primitive start state. |
| Simulation boundary | `SimulatedRobotAdapter` in `runtime.py` | Record simulated execute/hold decisions; does not command physical hardware. |
| Persistence | `save_library`, `load_library` in `storage.py` | Serialize and reload the certificate library. |

All source references above are under `src/risk_bounded_collision/`. The selector receives a scalar clearance bound; validating future-path clearance and covariance/library consistency requires integration beyond the timed lookup. Do not interpret the replay wrapper as a complete physical safety controller.

## Coverage of manuscript results

| Manuscript evidence | Included in this release | Reproduction status |
| --- | --- | --- |
| Paired Jetson lookup/recomputation, 105 cases | JSON summary, JSONL paired records, experiment script | Archived values can be checked; computation can be rerun on a new host. |
| Hardware-informed covariance and policy replay | Stationary encoder samples, full replay JSON, experiment script | Replay can be rerun; amplified covariance regimes are synthetic. |
| Calibration repeatability ablation | Derived numerical summary | Raw lab images and private fitting script are excluded; full refitting is unavailable here. |
| Stationary tabletop sensing and depth dropout | Derived result files | Historical evidence; original acquisition is not reproduced by this package. |
| Historical 1.69 million Monte Carlo trials, including 600,000 stress trials | All four suite result files, 24-condition CSV and simulation runners | Archived numerical results are included; full Monte Carlo reruns are computationally expensive. |
| Historical 2,500-scene dense swept-geometry comparison | Summary, seed, parameters, swept runner and unit tests | The seeded experiment can be rerun; the saved summary is not a per-scene trace. |

The manuscript distinguishes historical first-order directional experiments from the complete-link displacement model. The 23/24 stress-surrogate coverage result must not be restated as universal conservatism or as validation of every implementation path.

## Interpreting success

The supported conclusion is that offline precomputation substantially reduced the measured online computation while preserving the tested decisions, and the evaluated software replay guards produced holds. Unit tests and archived-record checks make these outcomes inspectable.

This evidence does not establish physical collision avoidance, end-to-end response time, robustness around moving people, or a universal risk guarantee. The manuscript's Gaussian quasi-static joint-offset assumptions and externally supplied path-clearance requirement remain essential.
