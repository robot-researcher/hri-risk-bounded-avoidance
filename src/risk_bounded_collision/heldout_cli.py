"""Run held-out collision-risk validation on challenging planar scenes."""

from __future__ import annotations

import argparse
import json
from math import hypot, pi
from pathlib import Path

from .geometry import CapsuleArmGeometry, CircleObstacle
from .kinematics import PlanarArm3DOF
from .models import Matrix3, Vector3
from .validation import calibrate_scene


def _correlated_covariance(
    stddev_degrees: Vector3,
    correlations: tuple[float, float, float],
) -> Matrix3:
    standard_deviations = tuple(value * pi / 180.0 for value in stddev_degrees)
    rho12, rho13, rho23 = correlations
    return (
        (
            standard_deviations[0] ** 2,
            rho12 * standard_deviations[0] * standard_deviations[1],
            rho13 * standard_deviations[0] * standard_deviations[2],
        ),
        (
            rho12 * standard_deviations[0] * standard_deviations[1],
            standard_deviations[1] ** 2,
            rho23 * standard_deviations[1] * standard_deviations[2],
        ),
        (
            rho13 * standard_deviations[0] * standard_deviations[2],
            rho23 * standard_deviations[1] * standard_deviations[2],
            standard_deviations[2] ** 2,
        ),
    )


def _obstacle_at_link_clearance(
    geometry: CapsuleArmGeometry,
    joint_angles: Vector3,
    link_index: int,
    fraction: float,
    clearance_m: float,
    *,
    side: float = 1.0,
    obstacle_radius_m: float = 0.015,
) -> CircleObstacle:
    """Place an obstacle normal to an interior point at a declared clearance."""
    points = geometry.arm.joint_positions(joint_angles)
    start = points[link_index]
    end = points[link_index + 1]
    point = (
        start[0] + fraction * (end[0] - start[0]),
        start[1] + fraction * (end[1] - start[1]),
    )
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = hypot(dx, dy)
    normal = (-side * dy / length, side * dx / length)
    offset = obstacle_radius_m + geometry.link_radii_m[link_index] + clearance_m
    return CircleObstacle(
        (point[0] + normal[0] * offset, point[1] + normal[1] * offset),
        obstacle_radius_m,
    )


def run_suite(trials: int) -> dict[str, object]:
    geometry = CapsuleArmGeometry(PlanarArm3DOF())
    specifications = (
        {
            "name": "bent-link3-correlated",
            "joint_angles": (0.45, -0.75, 0.35),
            "obstacles": ((2, 0.55, 0.005, 1.0),),
            "stddev_degrees": (0.8, 1.2, 0.6),
            "correlations": (0.45, -0.20, 0.25),
            "seed": 4101,
        },
        {
            "name": "folded-link3-correlated",
            "joint_angles": (0.80, -1.10, 0.65),
            "obstacles": ((2, 0.70, 0.012, -1.0),),
            "stddev_degrees": (1.2, 0.7, 1.0),
            "correlations": (-0.35, 0.20, 0.15),
            "seed": 4202,
        },
        {
            "name": "near-singular-link2-correlated",
            "joint_angles": (0.03, -0.04, 0.02),
            "obstacles": ((1, 0.50, 0.006, 1.0),),
            "stddev_degrees": (0.9, 1.1, 0.8),
            "correlations": (0.60, 0.10, 0.25),
            "seed": 4303,
        },
        {
            "name": "bent-two-obstacle-correlated",
            "joint_angles": (0.35, -0.65, 0.45),
            "obstacles": (
                (1, 0.35, 0.008, 1.0),
                (2, 0.65, 0.014, -1.0),
            ),
            "stddev_degrees": (1.0, 0.8, 1.3),
            "correlations": (0.30, -0.25, 0.40),
            "seed": 4404,
        },
    )

    results = []
    for specification in specifications:
        joint_angles = specification["joint_angles"]
        obstacles = tuple(
            _obstacle_at_link_clearance(
                geometry,
                joint_angles,
                link_index,
                fraction,
                clearance,
                side=side,
            )
            for link_index, fraction, clearance, side in specification["obstacles"]
        )
        covariance = _correlated_covariance(
            specification["stddev_degrees"], specification["correlations"]
        )
        result = calibrate_scene(
            specification["name"],
            geometry,
            joint_angles,
            obstacles,
            covariance,
            trials=trials,
            seed=specification["seed"],
            guard_sigma_multiplier=1.10,
        ).to_dict()
        result.update(
            {
                "joint_angles_rad": joint_angles,
                "joint_noise_stddev_degrees": specification["stddev_degrees"],
                "joint_noise_correlations": specification["correlations"],
                "nominal_minimum_clearances_m": [
                    geometry.minimum_clearance(joint_angles, obstacle).clearance_m
                    for obstacle in obstacles
                ],
            }
        )
        results.append(result)

    event_results = [result for result in results if result["collisions"] > 0]
    return {
        "protocol": "held-out scenes not used to choose the 1.10x sigma guard",
        "method": "planar capsule links; first-order normal projection; union bound across links and obstacles",
        "trials_per_scenario": trials,
        "total_trials": trials * len(results),
        "guard_sigma_multiplier": 1.10,
        "summary": {
            "scenarios": len(results),
            "event_scenarios": len(event_results),
            "guarded_covers_wilson": sum(
                bool(result["guarded_covers_wilson"]) for result in results
            ),
            "nominal_covers_wilson": sum(
                result["predicted_risk"] >= result["wilson_upper_95"]
                for result in results
            ),
            "nominal_mean_absolute_error": sum(
                abs(result["predicted_risk"] - result["observed_rate"])
                for result in results
            ) / len(results),
            "guarded_mean_excess_over_observed": sum(
                result["guarded_risk"] - result["observed_rate"]
                for result in results
            ) / len(results),
        },
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=25_000)
    parser.add_argument(
        "--output", type=Path, default=Path("artifacts/heldout_validation_results.json")
    )
    arguments = parser.parse_args()
    payload = run_suite(arguments.trials)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
