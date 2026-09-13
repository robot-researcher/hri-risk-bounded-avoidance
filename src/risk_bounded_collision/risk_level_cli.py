"""Validate configured collision-risk thresholds on pre-declared planar scenes."""

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
TARGET_FRACTION = 0.65


def _clearance_for_guarded_risk(
    geometry: CapsuleArmGeometry,
    joint_angles: tuple[float, float, float],
    link_index: int,
    fraction: float,
    side: float,
    covariance,
    target_risk: float,
) -> tuple[float, object]:
    """Binary-search a declared link clearance for a target guarded estimate."""
    certifier = DirectionalRiskCertifier(geometry, sigma_multiplier=1.10)
    lower = 1e-5
    upper = 0.25
    for _ in range(70):
        clearance = 0.5 * (lower + upper)
        obstacle = _obstacle_at_link_clearance(
            geometry,
            joint_angles,
            link_index,
            fraction,
            clearance,
            side=side,
        )
        risk = certifier.configuration_risk(joint_angles, obstacle, covariance)
        if risk > target_risk:
            lower = clearance
        else:
            upper = clearance
    clearance = upper
    obstacle = _obstacle_at_link_clearance(
        geometry,
        joint_angles,
        link_index,
        fraction,
        clearance,
        side=side,
    )
    return clearance, obstacle


def run_suite(trials: int = 100_000) -> dict[str, object]:
    if trials <= 0:
        raise ValueError("trials must be positive")
    geometry = CapsuleArmGeometry(PlanarArm3DOF())
    scene_specs = (
        {
            "name": "bent",
            "joint_angles": (0.45, -0.75, 0.35),
            "link_index": 2,
            "fraction": 0.55,
            "side": 1.0,
            "stddev_degrees": (0.8, 1.2, 0.6),
            "correlations": (0.45, -0.20, 0.25),
            "seed_base": 5100,
        },
        {
            "name": "folded",
            "joint_angles": (0.80, -1.10, 0.65),
            "link_index": 2,
            "fraction": 0.70,
            "side": -1.0,
            "stddev_degrees": (1.2, 0.7, 1.0),
            "correlations": (-0.35, 0.20, 0.15),
            "seed_base": 5200,
        },
        {
            "name": "near-singular",
            "joint_angles": (0.03, -0.04, 0.02),
            "link_index": 1,
            "fraction": 0.50,
            "side": 1.0,
            "stddev_degrees": (0.9, 1.1, 0.8),
            "correlations": (0.60, 0.10, 0.25),
            "seed_base": 5300,
        },
    )

    results = []
    for level_index, delta in enumerate(RISK_LEVELS):
        target_risk = TARGET_FRACTION * delta
        for scene_index, spec in enumerate(scene_specs):
            covariance = _correlated_covariance(
                spec["stddev_degrees"], spec["correlations"]
            )
            clearance, obstacle = _clearance_for_guarded_risk(
                geometry,
                spec["joint_angles"],
                spec["link_index"],
                spec["fraction"],
                spec["side"],
                covariance,
                target_risk,
            )
            result = calibrate_scene(
                f"{spec['name']}-delta-{delta:g}",
                geometry,
                spec["joint_angles"],
                (obstacle,),
                covariance,
                trials=trials,
                seed=spec["seed_base"] + 100 * level_index + scene_index,
                guard_sigma_multiplier=1.10,
            ).to_dict()
            result.update(
                {
                    "configured_delta": delta,
                    "design_target_risk": target_risk,
                    "design_target_fraction": TARGET_FRACTION,
                    "nominal_clearance_m": clearance,
                    "joint_angles_rad": spec["joint_angles"],
                    "joint_noise_stddev_degrees": spec["stddev_degrees"],
                    "joint_noise_correlations": spec["correlations"],
                    "wilson_meets_configured_delta": (
                        result["wilson_upper_95"] <= delta
                    ),
                    "wilson_to_delta_ratio": result["wilson_upper_95"] / delta,
                }
            )
            results.append(result)

    per_level = {}
    for delta in RISK_LEVELS:
        rows = [row for row in results if row["configured_delta"] == delta]
        per_level[f"{delta:g}"] = {
            "scenarios": len(rows),
            "scenarios_meeting_delta": sum(
                bool(row["wilson_meets_configured_delta"]) for row in rows
            ),
            "maximum_wilson_to_delta_ratio": max(
                row["wilson_to_delta_ratio"] for row in rows
            ),
            "mean_observed_rate": sum(row["observed_rate"] for row in rows)
            / len(rows),
            "total_collisions": sum(row["collisions"] for row in rows),
        }

    return {
        "protocol": (
            "pre-declared poses and risk levels; clearance selected from the "
            "guarded analytic model at 65% of delta; independent seeded Monte Carlo"
        ),
        "risk_levels": RISK_LEVELS,
        "design_target_fraction": TARGET_FRACTION,
        "guard_sigma_multiplier": 1.10,
        "trials_per_scenario": trials,
        "total_trials": trials * len(results),
        "summary": {
            "scenarios": len(results),
            "scenarios_meeting_configured_delta": sum(
                bool(row["wilson_meets_configured_delta"]) for row in results
            ),
            "per_level": per_level,
        },
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=100_000)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/risk_level_validation_results.json"),
    )
    arguments = parser.parse_args()
    payload = run_suite(arguments.trials)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
