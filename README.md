# Risk-bounded collision avoidance: research prototype

A portable Python implementation of offline motion-primitive risk bounds and online selection for a planar three-joint manipulator. This repository is a draft reproducibility companion for the HRI2027 manuscript; it does not claim publication acceptance or physical safety certification.

The algorithm provides capsule-link geometry, swept-motion margins, Gaussian uncertainty bounds, a certificate library, an adaptive risk threshold and a bounded candidate selector. It is an algorithm, not a trained neural-network model; certificates are generated for an explicitly chosen model and uncertainty assumption.

## Run locally

Requires Python 3.10 or newer. From this repository:

```bash
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows PowerShell instead:
# .venv\Scripts\Activate.ps1
python -m pip install -e .
python -m unittest discover -s tests -v
python -m risk_bounded_collision --output results/example_library.json
python -m risk_bounded_collision.benchmark_cli --iterations 2000 --output results/local_benchmark.json
```

The benchmark records its host platform. The Jetson comparisons in the results documentation use the paired experiment below and exclude camera, ROS and actuator latency.

## Hardware-informed uncertainty experiment

```bash
python -m pip install numpy
python experiments/measured_uncertainty.py --input experiments/stationary_encoder_ticks.json --output results/measured_uncertainty_new_run.json --link-lengths 0.4 0.3 0.2
python experiments/paired_online_baseline.py --input experiments/stationary_encoder_ticks.json --output results/paired_new_run.json --repeats 5 --link-lengths 0.4 0.3 0.2
```

These experiments use stationary encoder variability measured on the robot as an input to the planar research model. Amplified uncertainty regimes are synthetic sensitivity tests, not additional physical measurements. Experiment defaults are the manuscript's 0.4/0.3/0.2 m link lengths; the core class retains its legacy 0.3/0.25/0.15 m defaults. Neither claims equivalence to physical JetArm geometry. Choose a fresh output name when reproducing a run.

## Scope and evidence

See [results](docs/RESULTS.md), [reproduction notes](docs/REPRODUCIBILITY.md), and [release checklist](docs/RELEASE_CHECKLIST.md). Calibration consistency, embedded selector runtime, algorithm behavior and physical collision avoidance are separate outcomes. The supplied code contains no live robot command adapter, camera recordings, credentials or vendor SDK.

This repository accompanies a research manuscript in preparation; no DOI or accepted-paper citation is claimed. Repository quality and reproducibility can help reviewers assess the work; they do not guarantee favorable reviews.

## License

Code is provided for anonymous peer review. No accepted-paper status or DOI is claimed. The final bibliographic citation will be added when author and publication metadata are settled.
