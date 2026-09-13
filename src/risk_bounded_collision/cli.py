"""Command-line demonstration of the offline and online pipeline."""

from __future__ import annotations

import argparse
import json
from math import pi
from pathlib import Path

from .kinematics import PlanarArm3DOF
from .models import Matrix3
from .primitives import generate_primitives
from .risk import RiskCertifier, adaptive_risk_threshold
from .selector import PrimitiveSelector
from .storage import save_library


def _diagonal_covariance(stddev_rad: float) -> Matrix3:
    variance = stddev_rad**2
    return ((variance, 0.0, 0.0), (0.0, variance, 0.0), (0.0, 0.0, variance))


def run_demo(output: Path, primitive_count: int) -> dict[str, object]:
    start = (0.0, 0.0, 0.0)
    goal = (0.30, -0.20, 0.15)
    covariance = _diagonal_covariance(pi / 180.0)
    bins = tuple(0.05 * index for index in range(1, 21))

    primitives = generate_primitives(start, count=primitive_count)
    library = RiskCertifier(PlanarArm3DOF()).build_library(primitives, bins, covariance)
    save_library(library, output)

    obstacle_distance = 0.25
    threshold = adaptive_risk_threshold(obstacle_distance, covariance)
    selection = PrimitiveSelector(library).select(
        obstacle_distance_m=obstacle_distance,
        threshold=threshold,
        goal_joint_angles=goal,
    )
    return {
        "certificate_path": str(output),
        "primitive_count": len(primitives),
        "distance_bin_count": len(bins),
        "threshold": threshold,
        "selection": {
            "primitive_id": selection.primitive_id,
            "risk_bound": selection.risk_bound,
            "distance_bin_m": selection.distance_bin_m,
            "fallback": selection.fallback,
            "reason": selection.reason,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/certificate_library.json"))
    parser.add_argument("--primitive-count", type=int, default=64)
    arguments = parser.parse_args()
    print(json.dumps(run_demo(arguments.output, arguments.primitive_count), indent=2))


if __name__ == "__main__":
    main()
