# Experimental results catalog

Snapshot: 13 September 2026. Includes completed numerical results located in the research workspace and the task **Plan HRI robot experiments**. Original unfavorable results, corrected-label acquisitions and reviewed calibration outputs are retained separately.

## Simulation and algorithm evaluation

| Study | Dataset / result | Interpretation |
| --- | --- | --- |
| Initial directional calibration | [90,000 trials](../results/archive/artifacts/calibration_results.json) | Nine scenarios; six with collision events; finite-sample resolution limits apply to zero-event scenarios. |
| Held-out directional calibration | [100,000 trials](../results/archive/artifacts/heldout_validation_results.json) | Four scenes; guarded estimate covered pointwise Wilson upper limits in 4/4. |
| Configured risk levels | [900,000 trials](../results/archive/artifacts/risk_level_validation_results.json) | Nine conditions across 0.001, 0.01, 0.05; all nine pointwise upper limits met their thresholds. |
| Stress matrix | [600,000 trials](../results/archive/artifacts/stress_validation_results.json), [24-condition CSV](../results/archive/stress_condition_counts_2026-09-13.csv) | All 24 pointwise upper limits met thresholds, but the directional surrogate covered only **23/24**. |
| Nominal swept geometry | [2,500 scenes](../results/archive/artifacts/swept_validation_results.json) | Detected all 658 between-sample collisions; 340 conservative rejections; dense interpolation is an empirical reference. |
| Library conservatism | [10,000 scenes](../results/archive/artifacts/certificate_conservatism_results.json) | Coverage and over-rejection relative to the directional model, not physical ground truth. |
| Reduced HSC baseline | [20 runs / 100,000 test queries](../results/archive/artifacts/hsc_reduced_baseline_results.json) | A reduced Python query kernel; **one false-free result remained after rectification**. Not the authors' optimized planner. |
| Original host benchmark | [Timing and configuration](../results/archive/artifacts/benchmark_results.json) | Development-host latency; not Jetson or end-to-end latency. |
| Simulated runtime demo | [CSV](../results/archive/artifacts/live_sim_demo.csv) | Simulated decisions, not executed physical avoidance. |

The four Monte Carlo suites total **1,690,000 configuration-level trials**. They do not constitute 1.69 million executed trajectories or human-interaction trials.

## Embedded computation and replay

- [Original Jetson microbenchmark](../results/archive/data/selector_benchmark_20260910T103158Z/summary.json): three runs of 5,400 calls; separate from the later paired experiment.
- [Paired Jetson experiment](../results/paired_jetson_paper_geometry.json): 105 matching decisions; median 0.1865 ms lookup versus 172.5978 ms recomputation.
- [Hardware-informed policy and replay](../results/measured_uncertainty_jetson_paper_geometry.json): adaptive acceptance 114/108/99 out of 120 at covariance factors 1/4/16; all 720 recorded hold cases pass.
- Prototype-geometry and Windows smoke results remain in `results/` with their original filenames. Do not combine those timings with the paper-geometry protocol.

## Physical calibration and sensing

| Study | Evidence | Result and caveat |
| --- | --- | --- |
| Initial RGB calibration | [Original fit](../results/archive/artifacts/rgb_calibration_20260910/calibration.json) | 49/50 detected images; RMS 3.334 px; retained as earlier exploratory calibration. |
| Rigid-board RGB calibration | [Improved fit](../results/archive/artifacts/rgb_rigid_20260910_132510/calibration.json) | 50/50 detected images; RMS 0.824 px; candidate parameters, not installed control calibration. |
| Stationary joint feedback | [Summary](../results/archive/artifacts/stationary_feedback_20260910/summary.json) | 30 snapshots, six servo replies each; recorded scatter is not dynamic tracking uncertainty. |
| Hand-eye calibration | [Original](../results/archive/artifacts/handeye_20260910T101550Z/handeye_results.json), [reviewed](../results/archive/artifacts/handeye_20260910T101550Z_reviewed/handeye_results.json) | Park/Horaud held-out RMS 3.393 mm; Tsai insufficient-rotation failure retained in original evidence and excluded from reviewed estimates. |
| Frozen calibration repeat | [Repeat](../results/archive/artifacts/handeye_repeat_20260910T102033Z/results.json) | 20 repeat poses; position RMS 4.678 mm, maximum 8.544 mm; orientation RMS 0.674 degrees. |
| Measured-feedback ablation | [Comparison](../results/feedback_calibration_ablation.json) | Repeat RMS 14.628 to 4.678 mm; consistency rather than absolute accuracy. |
| RGB/depth audit | [First audit](../results/archive/data/depth_audit_20260910T102802Z/summary.json), [repeat audit](../results/archive/data/depth_audit_20260910T104810Z/summary.json) | About 57 mm RGB-plane/depth disagreement remains unresolved. The operator corrected the approximate camera-board distance from 45 to 52.5 cm. |
| Box present/removed | [Matched pair](../results/archive/artifacts/tabletop_pair_20260911/results.json) | ROI medians 359 versus 716 mm. |
| Replacement and background repeat | [Replacement](../results/archive/artifacts/tabletop_repositioned_20260911/results.json), [sequence](../results/archive/artifacts/box_sequence_20260911/results.json) | Replacement 364 mm; repeated background 716 mm; changed placement is not sensor error. |
| Marked approximate 35 cm placement | [Results](../results/archive/artifacts/tabletop_marked35cm_20260911/results.json) | ROI median 359 mm; approximate manual distance is not calibrated axial ground truth. |
| Retained overnight scene | [Suite](../results/archive/artifacts/obstacle_sensing_suite_20260911/results.json) | 356 mm median; not an independent replacement or general stability study. |
| Later retained-scene observation | [September 12 repeat](../results/archive/artifacts/tabletop_remote_20260912T082901Z/results.json) | 356 mm median, 99.9982% valid ROI samples; same retained scene. |
| Synthetic depth dropout | [Replay](../results/archive/artifacts/depth_dropout_replay_20260912/results.json) | Two recordings with 0/5/20/100% invalidation; all 240 frames at 20/100% become UNKNOWN. Analysis-rule behavior, not physical stopping. |

## Corrections and completeness

The early `20260911T074458Z_background` and `20260911T112540Z_obstacle` captures used incompatible camera/scene conditions and are retained as exploratory metadata, not a matched detection success. The `20260912T080201Z_background` acquisition was subsequently identified as a retained-obstacle scene. Its renamed `obstacle_overnight_stability` copy is the same acquisition: **do not count it twice**. The ledger preserves both numerical metadata versions.

Tabletop frame intervals near 100 ms were deliberately paced. No completed native-rate timing result was found for the latest planned experiment in the referenced task; planned or interrupted work is not recorded as a completed result. Camera previews and setup movements are not physical avoidance trials.

[Provenance ledger](../results/archive/PROVENANCE.json) lists every imported file and its original/exported SHA-256. Numerical summaries and per-frame/per-pose records are included where available. Raw lab photographs, original depth-image arrays, access logs and credentials are excluded. The registration illustration contains a bystander, so it is not published; its numerical audit remains included. This limits full refitting/reacquisition reproducibility.

## Reproduce historical simulation studies

Run from the repository after installing the package. Use a fresh output location; the original archived results remain unchanged.

```bash
python -m risk_bounded_collision.calibration_cli --trials 10000 --output results/new_calibration.json
python -m risk_bounded_collision.heldout_cli --trials 25000 --output results/new_heldout.json
python -m risk_bounded_collision.risk_level_cli --trials 100000 --output results/new_risk_levels.json
python -m risk_bounded_collision.stress_validation_cli --trials 25000 --output results/new_stress.json
python -m risk_bounded_collision.swept_validation_cli --scenes 2500 --dense-steps 100 --seed 8101 --output results/new_swept.json
python -m risk_bounded_collision.certificate_analysis_cli --scenes 10000 --output results/new_conservatism.json
python -m risk_bounded_collision.hsc_baseline_cli --runs 20 --training-queries 1000 --test-queries 5000 --output results/new_hsc.json
```

The current runners and saved historical summaries are both included. Code availability is not a claim that every historical suite was rerun during packaging. Seeds, parameters and saved outcomes must be checked when comparing a fresh run.
