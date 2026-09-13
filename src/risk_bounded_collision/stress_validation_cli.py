"""Run a pre-specified stress matrix across poses, noise, risk, and obstacles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .geometry import CapsuleArmGeometry
from .heldout_cli import _correlated_covariance, _obstacle_at_link_clearance
from .kinematics import PlanarArm3DOF
from .risk import DirectionalRiskCertifier
from .validation import calibrate_scene


RISK_LEVELS = (0.001, 0.01, 0.05)
TARGET_FRACTION = 0.55


SCENES = (
    ("bent-distal", (0.45, -0.75, 0.35), ((2, 0.55, 1.0),), (0.8, 1.2, 0.6), (0.45, -0.20, 0.25)),
    ("folded-distal", (0.80, -1.10, 0.65), ((2, 0.70, -1.0),), (1.2, 0.7, 1.0), (-0.35, 0.20, 0.15)),
    ("near-singular-middle", (0.03, -0.04, 0.02), ((1, 0.50, 1.0),), (0.9, 1.1, 0.8), (0.60, 0.10, 0.25)),
    ("upright-proximal", (1.10, -0.40, -0.50), ((0, 0.55, -1.0),), (0.7, 1.0, 1.3), (-0.20, 0.15, 0.35)),
    ("zigzag-distal", (-0.70, 1.15, -0.80), ((2, 0.40, 1.0),), (1.4, 0.6, 0.9), (0.25, -0.30, 0.10)),
    ("asymmetric-middle", (-1.00, 0.35, 0.90), ((1, 0.65, -1.0),), (0.6, 1.4, 0.8), (0.15, 0.30, -0.25)),
    ("bent-two-obstacle", (0.35, -0.65, 0.45), ((1, 0.35, 1.0), (2, 0.65, -1.0)), (1.0, 0.8, 1.3), (0.30, -0.25, 0.40)),
    ("folded-two-obstacle", (0.75, -1.20, 0.55), ((0, 0.70, -1.0), (2, 0.50, 1.0)), (1.1, 0.9, 1.2), (-0.25, 0.20, 0.30)),
)


def _obstacle_for_target(
    geometry,
    joint_angles,
    link_index,
    fraction,
    side,
    covariance,
    target_risk,
):
    certifier = DirectionalRiskCertifier(geometry, sigma_multiplier=1.10)
    lower, upper = 1e-5, 0.30
    for _ in range(70):
        clearance = 0.5 * (lower + upper)
        obstacle = _obstacle_at_link_clearance(
            geometry, joint_angles, link_index, fraction, clearance, side=side
        )
        risk = certifier.configuration_risk(joint_angles, obstacle, covariance)
        if risk > target_risk:
            lower = clearance
        else:
            upper = clearance
    return _obstacle_at_link_clearance(
        geometry, joint_angles, link_index, fraction, upper, side=side
    )


def run_suite(trials: int = 25_000) -> dict[str, object]:
    if trials <= 0:
        raise ValueError("trials must be positive")
    geometry = CapsuleArmGeometry(PlanarArm3DOF())
    results = []
    for level_index, delta in enumerate(RISK_LEVELS):
        for scene_index, (name, angles, placements, stddevs, correlations) in enumerate(SCENES):
            covariance = _correlated_covariance(stddevs, correlations)
            per_obstacle_target = TARGET_FRACTION * delta / len(placements)
            obstacles = tuple(
                _obstacle_for_target(
                    geometry,
                    angles,
                    link_index,
                    fraction,
                    side,
                    covariance,
                    per_obstacle_target,
                )
                for link_index, fraction, side in placements
            )
            result = calibrate_scene(
                f"{name}-delta-{delta:g}",
                geometry,
                angles,
                obstacles,
                covariance,
                trials=trials,
                seed=6100 + 100 * level_index + scene_index,
                guard_sigma_multiplier=1.10,
            ).to_dict()
            result.update(
                {
                    "configured_delta": delta,
                    "design_target_fraction": TARGET_FRACTION,
                    "joint_angles_rad": angles,
                    "joint_noise_stddev_degrees": stddevs,
                    "joint_noise_correlations": correlations,
                    "nominal_minimum_clearances_m": [
                        geometry.minimum_clearance(angles, obstacle).clearance_m
                        for obstacle in obstacles
                    ],
                    "wilson_meets_configured_delta": result["wilson_upper_95"] <= delta,
                    "wilson_to_delta_ratio": result["wilson_upper_95"] / delta,
                }
            )
            results.append(result)

    per_level = {}
    for delta in RISK_LEVELS:
        rows = [row for row in results if row["configured_delta"] == delta]
        per_level[f"{delta:g}"] = {
            "scenarios": len(rows),
            "scenarios_meeting_delta": sum(row["wilson_meets_configured_delta"] for row in rows),
            "guarded_covers_wilson": sum(row["guarded_covers_wilson"] for row in rows),
            "maximum_wilson_to_delta_ratio": max(row["wilson_to_delta_ratio"] for row in rows),
            "total_collisions": sum(row["collisions"] for row in rows),
        }
    return {
        "protocol": "predeclared eight-scene stress matrix; independent seeds; no guard refitting",
        "scope": "sampled-state planar capsules with Gaussian correlated encoder noise",
        "risk_levels": RISK_LEVELS,
        "scene_count": len(SCENES),
        "trials_per_scenario": trials,
        "total_trials": trials * len(results),
        "guard_sigma_multiplier": 1.10,
        "design_target_fraction": TARGET_FRACTION,
        "summary": {
            "scenarios": len(results),
            "scenarios_meeting_configured_delta": sum(
                row["wilson_meets_configured_delta"] for row in results
            ),
            "guarded_covers_wilson": sum(row["guarded_covers_wilson"] for row in results),
            "per_level": per_level,
        },
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=25_000)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/stress_validation_results.json"),
    )
    arguments = parser.parse_args()
    payload = run_suite(arguments.trials)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
