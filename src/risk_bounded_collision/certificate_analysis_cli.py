"""Quantify direction-independent certificate conservatism on random scenes."""

from __future__ import annotations

import argparse
import json
import random
from math import pi
from pathlib import Path

from .geometry import CapsuleArmGeometry
from .heldout_cli import _obstacle_at_link_clearance
from .kinematics import PlanarArm3DOF
from .models import Matrix3
from .primitives import quintic_primitive
from .risk import DirectionalRiskCertifier, RiskCertifier


RISK_LEVELS = (0.001, 0.01, 0.05)


def _covariance(stddev_degrees: tuple[float, float, float], rho: float) -> Matrix3:
    stddevs = tuple(value * pi / 180.0 for value in stddev_degrees)
    return tuple(
        tuple(
            stddevs[i] * stddevs[j] * (1.0 if i == j else rho)
            for j in range(3)
        )
        for i in range(3)
    )  # type: ignore[return-value]


def run_analysis(scene_count: int = 10_000, seed: int = 7301) -> dict[str, object]:
    if scene_count <= 0:
        raise ValueError("scene_count must be positive")
    arm = PlanarArm3DOF()
    geometry = CapsuleArmGeometry(arm)
    offline = RiskCertifier(arm, sigma_multiplier=1.10)
    directional = DirectionalRiskCertifier(geometry, sigma_multiplier=1.10)
    rng = random.Random(seed)
    rows = []
    while len(rows) < scene_count:
        angles = tuple(rng.uniform(-1.2, 1.2) for _ in range(3))
        link_index = rng.randrange(3)
        fraction = rng.uniform(0.1, 0.9)
        side = -1.0 if rng.random() < 0.5 else 1.0
        declared_clearance = rng.uniform(0.002, 0.10)
        obstacle = _obstacle_at_link_clearance(
            geometry,
            angles,
            link_index,
            fraction,
            declared_clearance,
            side=side,
        )
        minimum_clearance = geometry.minimum_clearance(angles, obstacle).clearance_m
        if minimum_clearance <= 0.0:
            continue
        stddevs = tuple(rng.uniform(0.5, 1.5) for _ in range(3))
        rho = rng.uniform(-0.35, 0.45)
        covariance = _covariance(stddevs, rho)
        primitive = quintic_primitive("stationary", angles, angles, sample_count=2)
        offline_risk = offline.primitive_risk(primitive, minimum_clearance, covariance)
        directional_risk = directional.configuration_risk(angles, obstacle, covariance)
        rows.append((offline_risk, directional_risk))

    summaries = {}
    for delta in RISK_LEVELS:
        directional_safe = sum(directional_risk <= delta for _, directional_risk in rows)
        offline_accepted = sum(offline_risk <= delta for offline_risk, _ in rows)
        over_rejected = sum(
            directional_risk <= delta < offline_risk
            for offline_risk, directional_risk in rows
        )
        false_accepted = sum(
            offline_risk <= delta < directional_risk
            for offline_risk, directional_risk in rows
        )
        summaries[f"{delta:g}"] = {
            "directional_safe_scenes": directional_safe,
            "offline_accepted_scenes": offline_accepted,
            "offline_coverage": offline_accepted / scene_count,
            "over_rejected_safe_scenes": over_rejected,
            "over_rejection_rate_among_directional_safe": (
                over_rejected / directional_safe if directional_safe else 0.0
            ),
            "false_acceptances_against_directional_model": false_accepted,
        }
    return {
        "protocol": "seeded random nominal configurations; stationary primitives; no Monte Carlo",
        "interpretation": "measures conservatism relative to the direction-aware first-order model, not physical collision probability",
        "scene_count": scene_count,
        "seed": seed,
        "sigma_multiplier": 1.10,
        "risk_levels": RISK_LEVELS,
        "summary": summaries,
        "conservatism_gap": {
            "mean_offline_minus_directional": sum(
                offline_risk - directional_risk
                for offline_risk, directional_risk in rows
            ) / scene_count,
            "maximum_offline_minus_directional": max(
                offline_risk - directional_risk
                for offline_risk, directional_risk in rows
            ),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", type=int, default=10_000)
    parser.add_argument("--seed", type=int, default=7301)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/certificate_conservatism_results.json"),
    )
    arguments = parser.parse_args()
    payload = run_analysis(arguments.scenes, arguments.seed)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
