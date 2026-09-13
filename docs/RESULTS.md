# Experimental results and claim boundaries

The final embedded comparisons below use the manuscript's **0.4, 0.3, 0.2 m planar links**, 64 primitives and **40 clearance bins** for this additional experiment. They are distinct from the manuscript's earlier 20-bin evaluation and from the physical JetArm's dimensions. Do not merge them into one protocol description.

## Paired embedded computation baseline

Both implementations ran on the same Jetson Nano / Python 3.6.9. The baseline recomputes the identical continuous swept-risk expression online for all 64 primitives; the proposed implementation uses precomputed bounds. Both use the same covariance, conservative bin, adaptive threshold, risk ranking and 16-candidate decision rule. We evaluated 21 unique clearance/goal queries over five shuffled repetitions, alternating which method ran first.

| Method | Median | 95th percentile | 99th percentile |
| --- | ---: | ---: | ---: |
| Precomputed lookup | 0.1865 ms | 0.2157 ms | 0.2224 ms |
| Uncached online recomputation | 172.5978 ms | 174.7046 ms | 309.7605 ms |

All **105 paired decisions matched**, including selected primitive and risk value. Offline library construction took **7.048 seconds**; that cost is excluded from online timings. The median ratio is approximately 925, attributable to amortizing this explicit recomputation baseline. This is not a comparison against an optimized competing planner, and not an end-to-end robot latency or stopping-time result. The existing vendor camera and driver services remained active; real-time isolation and power/thermal control were not established.

Evidence: `results/paired_jetson_paper_geometry.json` and its `.jsonl` paired timings. Prototype-geometry and Windows smoke-test files are retained with different names and must not be substituted for these results.

## Hardware-informed uncertainty-policy comparison

Thirty actual stationary readings of servo IDs 2, 3 and 4 supplied the sample covariance. We added a separately documented one-tick quantization allowance. Covariance multipliers 4 and 16 are synthetic sensitivity regimes; the library was rebuilt for each covariance. Each policy faced the same 40 clearances and three goals (120 cases per regime).

| Covariance multiplier | Adaptive threshold: accepted / 120 | Fixed threshold 0.05: accepted / 120 |
| --- | ---: | ---: |
| 1 | 114 | 117 |
| 4 (synthetic) | 108 | 111 |
| 16 (synthetic) | 99 | 102 |

Acceptance decreased as uncertainty increased. The adaptive policy was more restrictive than the fixed policy in these cases; this alone does not demonstrate a lower physical collision rate. Separate runtime replays produced **360/360 holds for stale observations**. All 360 wrong-start cases also held: **321 explicitly failed the start-state check**, while **39 had already failed risk feasibility**. These 720 checks are software replay cases, not 720 physical robot trials.

Evidence: `results/measured_uncertainty_jetson_paper_geometry.json` and `experiments/stationary_encoder_ticks.json`.

## Measured-feedback calibration ablation

Two calibrations were fitted to the same first 20 fixed-board captures: one using commanded positions, the other using measured encoders. Each was tested unchanged on the same second sequence of 20 captures.

| Calibration input | Repeat position RMS | Maximum deviation | Repeat orientation RMS |
| --- | ---: | ---: | ---: |
| Command cache | 14.628 mm | 25.458 mm | 1.541 degrees |
| Measured feedback | 4.678 mm | 8.544 mm | 0.674 degrees |

Measured feedback reduced position inconsistency by about **68%** on these captures. This is fixed-board consistency, not independent absolute robot accuracy. All 40 captures detected the 70 checkerboard corners; metric results assume 20 mm squares. Raw lab images are excluded from the public draft because they include identifiable people. Evidence: `results/feedback_calibration_ablation.json`. The private analysis script and raw captures are retained in the laboratory workspace.

## Sensor status and unresolved accuracy

The manually rechecked camera-to-board distance is approximately **52.5 cm**, superseding the earlier 45 cm estimate. Native central depth was approximately **53 cm**; RGB checkerboard pose estimated approximately **47–48 cm**. Two 30-frame audits reproduced about **57 mm RGB-PnP/depth plane disagreement**. Native depth is the provisional distance source because it agrees more closely at this reference distance. No global correction or multi-distance accuracy claim is justified by that one approximate check.

In the later dark-room session, RGB mean intensity was approximately 3.08/255 and checkerboard detection failed. Depth remained live with approximately 72.6% valid pixels in the 0.1–5 m range. This is an observational diagnostic, not a controlled lighting study. No arm motion was issued during the night-session computation tests.

## Stationary tabletop RGB-D obstacle sensing

We evaluated stationary RGB-D sensing feasibility across six 60-frame depth sequences (360 frames total, 16UC1, 640×400) acquired on the JetArm's tilted camera with arm joints stationary (wrist tilt ID 4 settled at 191 ticks; maximum encoder drift during acquisition $\le 1$ tick). A fixed lower-middle native-depth ROI ($x \in [240, 400), y \in [320, 390)$, 11,200 pixels) was evaluated against a 50 mm foreground threshold relative to the initial background reference (716.0 mm).

| Sequence | Role | Frames | ROI median depth | Valid depth fraction | Foreground triggers | Separation from background |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `20260911T115632Z` | Initial obstacle placement | 60 | 359.0 mm | 99.999% | 60 / 60 | 354.0 mm |
| `20260911T115858Z` | Background reference | 60 | 716.0 mm | 99.996% | 0 / 60 | 0.0 mm |
| `20260911T120444Z` | Unguided repositioning | 60 | 364.0 mm | 99.849% | 60 / 60 | 350.0 mm |
| `20260911T120830Z` | Held-out background | 60 | 716.0 mm | 99.989% | 0 / 60 | -1.0 mm |
| `20260911T121504Z` | Marked 35 cm placement | 60 | 359.0 mm | 99.999% | 60 / 60 | 357.0 mm |
| `20260912T080201Z` | Overnight stability | 60 | 356.0 mm | 100.000% | 60 / 60 | 358.0 mm |

Key sensing outcomes:

- **Foreground separation**: The obstacle produced 350–358 mm separation from the 716 mm tabletop background. 100% of jointly valid pixels in the ROI were nearer by $>50\text{ mm}$ across all 240 obstacle frames.
- **Background stability**: The held-out background matched the reference background with identical 716.0 mm median depth, a median pixelwise difference of 1.0 mm, and a maximum absolute pixelwise difference of 10.0 mm across all 11,200 pixels.
- **Repeatability**: Unguided repositioning differed from the initial placement by 5.0 mm (364 vs 359 mm), marked replacement reproduced the initial depth exactly (359.0 vs 359.0 mm, 0.0 mm delta), and overnight settling remained within 3.0 mm (356.0 mm).
- **False-trigger sanity**: 240/240 obstacle frames triggered foreground detection; 0/120 background frames triggered false detections. Zero frames had invalid/unknown depth.
- **Sampling latency**: Mean frame interval was 100.83 ms ($\approx 9.92\text{ Hz}$, std 9.27 ms, p05 97.13 ms, p95 126.74 ms).

Evidence: `results/tabletop_obstacle_sensing.json`.

## Explicit claim boundaries and limitations

To prevent conflation of algorithmic properties with physical capabilities, results are divided into four strictly separated categories:

1. **Offline and replay safety tests**:
   The 720 replay checks (360 stale holds and 360 wrong-start holds) demonstrate software guard enforcement on recorded joint state vectors. They do not demonstrate physical collision prevention or dynamic real-time obstacle avoidance.
2. **Stationary physical sensing**:
   The tabletop depth experiments evaluate sensor noise, validity, background stability, and spatial separation in a fixed camera pose. They do not provide 3D camera-to-base obstacle coordinates or evaluate obstacle perception under arm ego-motion.
3. **Kinematic calibration consistency**:
   The 4.678 mm repeat RMS achieved by measured feedback demonstrates repeatable kinematic loop closure on a static checkerboard. It does not certify absolute world-coordinate end-effector accuracy across the entire workspace.
4. **Unvalidated physical collision avoidance**:
   Physical closed-loop collision avoidance has **not** been validated. Camera-to-base registration, arm swept-volume checking, dynamic braking distance, and physical collision-rate experiments are incomplete. No claim is made that the physical JetArm avoided moving or static obstacles in closed-loop operation.

- The isolated core test suite passed 38 tests before these comparisons.
- Covariance, source hashes, query sets, timing records and explicit geometry are saved with the results.
- Stationary encoder scatter does not identify dynamic tracking error, bias or Gaussian tail behavior.
- The continuous swept-bound implementation assumes quasi-static Gaussian joint offsets over a primitive; it does not cover arbitrary moving obstacles or actuator dynamics.
- The manuscript's description of first-order propagation must be reconciled with the released nonlinear swept-bound implementation before submission.
