"""Run the seeded geometry-aware Monte Carlo calibration suite."""

from __future__ import annotations

import argparse
import json
from math import pi
from pathlib import Path

from .geometry import CapsuleArmGeometry, CircleObstacle
from .kinematics import PlanarArm3DOF
from .models import Matrix3
from .validation import calibrate_configuration


def _covariance(stddev_degrees: float) -> Matrix3:
    variance = (stddev_degrees * pi / 180.0) ** 2
    return ((variance, 0.0, 0.0), (0.0, variance, 0.0), (0.0, 0.0, variance))


def run_suite(trials: int) -> dict[str, object]:
    geometry = CapsuleArmGeometry(PlanarArm3DOF())
    joint_angles = (0.0, 0.0, 0.0)
    scenarios = (
        ("tight-clearance", CircleObstacle((0.50, 0.035), 0.015), 101),
        ("moderate-clearance", CircleObstacle((0.50, 0.055), 0.015), 202),
        ("open-clearance", CircleObstacle((0.50, 0.085), 0.015), 303),
    )
    results = []
    for noise_index, stddev_degrees in enumerate((0.5, 1.0, 2.0)):
        covariance = _covariance(stddev_degrees)
        for name, obstacle, base_seed in scenarios:
            result = calibrate_configuration(
                f"{name}-{stddev_degrees:g}deg",
                geometry,
                joint_angles,
                obstacle,
                covariance,
                trials=trials,
                seed=base_seed + 1000 * noise_index,
                guard_sigma_multiplier=1.10,
            ).to_dict()
            result["joint_noise_stddev_degrees"] = stddev_degrees
            results.append(result)
    event_results = [result for result in results if result["collisions"] > 0]
    zero_event_upper = next(
        result["wilson_upper_95"] for result in results if result["collisions"] == 0
    )
    return {
        "method": "planar capsule links; first-order normal projection; union bound across links",
        "trials_per_scenario": trials,
        "total_trials": trials * len(results),
        "guard_sigma_multiplier": 1.10,
        "summary": {
            "scenarios": len(results),
            "scenarios_with_collision_events": len(event_results),
            "nominal_mean_absolute_error_on_event_scenarios": sum(
                abs(result["predicted_risk"] - result["observed_rate"])
                for result in event_results
            ) / len(event_results),
            "guarded_covers_wilson_on_event_scenarios": sum(
                bool(result["guarded_covers_wilson"]) for result in event_results
            ),
            "zero_event_scenarios": len(results) - len(event_results),
            "zero_event_wilson_resolution_95": zero_event_upper,
        },
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=10_000)
    parser.add_argument("--output", type=Path, default=Path("artifacts/calibration_results.json"))
    arguments = parser.parse_args()
    payload = run_suite(arguments.trials)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
