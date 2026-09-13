# Robot and experimental setup

These specifications describe the hardware recorded during the September 2026 experiments, rather than a generic product listing.

| Item | Recorded specification | Evidence status |
| --- | --- | --- |
| Robot | Hiwonder JetArm-style serial arm | Installed JetArm workspace, SDK and observed joint IDs; initially referred to as JetMax by the operator. Exact retail SKU not verified. |
| Articulation | Five positioning joints plus gripper | Servo IDs **1, 2, 3, 4, 5, 10**; ID 10 is the gripper. No ID 6 demonstrated. |
| Embedded computer | NVIDIA Jetson Nano, aarch64; **4 GB RAM user-identified** | Embedded runtime recorded in benchmark JSON; RAM capacity comes from the operator's inventory. |
| OS / runtime | Ubuntu 18.04.6 LTS, kernel 4.9.337-tegra, NVIDIA L4T R32.7.6, Python 3.6.9 | Recorded robot inventory and experiment outputs. Portable release targets Python 3.10+. |
| Middleware | ROS 1 Melodic; JetArm SDK | Recorded installed driver baseline. |
| RGB-D camera | Arm-mounted RGB-D camera; vendor configuration named **Gemini** | Exact sensor SKU unverified. Earlier Astra references are not treated as verified identification. |
| RGB stream | 640 x 480 pixels | Published CameraInfo. |
| Native depth stream | 640 x 400 pixels, 16UC1 in the stationary trials | Recorded acquisition metadata. |
| Observed delivery rate | About 30 Hz RGB and depth in brief topic-rate checks | Not sustained throughput. Tabletop acquisition was deliberately paced near 10 Hz. |
| Controller | USB serial, vendor Board SDK, `/dev/rrc`, historically configured at 1,000,000 baud | Controller MCU model unverified. |
| Joint feedback | Actual position queries through the existing driver; 30 snapshots, 180 replies | [Stationary feedback summary](../results/archive/artifacts/stationary_feedback_20260910/summary.json). Cached `/servo_states` values were commands, not measured feedback. |
| Modeled tick conversion | 0.004188790204786391 rad/tick, approximately 0.24 degrees/tick | Conversion used by the experiment; not an angular-accuracy specification. |
| Calibration target | 10 x 7 inner corners (11 x 8 squares); assumed 20 mm square side | Metric results depend on that reported target scale. |

## Published camera intrinsics

| Stream | fx | fy | cx | cy |
| --- | ---: | ---: | ---: | ---: |
| RGB | 450.7859497 | 450.7859497 | 326.0953674 | 243.2000885 |
| Depth | 476.1980286 | 476.1980286 | 323.6578064 | 198.0494995 |

These are the recorded driver CameraInfo values, not the later experimental RGB recalibration. Alignment and synchronization were disabled in the inspected configuration. Resizing an RGB image does not establish depth registration.

## Physical arm versus paper model

The paper's principal planar model has three joints and **0.4, 0.3, 0.2 m** links. The embedded paired experiment uses 64 primitives and 40 clearance bins; earlier host experiments use a different bin protocol. The generic package demo retains legacy 0.30, 0.25, 0.15 m links.

None of those dimensions establishes the physical JetArm's measured geometry. Physical reach, payload rating, servo model, certified accuracy, calibrated all-link dimensions and stopping distance have not been verified in the available records and are not assigned nominal catalog values here.

Calibration poses were physically acquired; sensing trials were stationary. The portable runtime's adapter is simulated. No physical closed-loop avoidance success rate was measured.

Provenance: locally recorded hardware inventory and corrected continuation notes dated September 9–12, plus the archived acquisition metadata and benchmark files. Access addresses and machine account details are omitted.
