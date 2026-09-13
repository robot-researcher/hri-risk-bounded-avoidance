"""Validate swept-clearance certificates against dense continuous-path checks."""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

from .geometry import CapsuleArmGeometry
from .heldout_cli import _obstacle_at_link_clearance
from .kinematics import PlanarArm3DOF
from .primitives import quintic_primitive
from .swept import certify_swept_clearance


def _dense_link_minima(
    geometry,
    primitive,
    obstacles,
    dense_steps_per_interval,
):
    minima = [float("inf")] * 3
    collision = False
    for start, end in zip(primitive.positions, primitive.positions[1:]):
        for step in range(dense_steps_per_interval + 1):
            fraction = step / dense_steps_per_interval
            angles = tuple(
                start[joint] + fraction * (end[joint] - start[joint])
                for joint in range(3)
            )
            for obstacle in obstacles:
                queries = geometry.link_clearances(angles, obstacle)
                for link_index, query in enumerate(queries):
                    minima[link_index] = min(
                        minima[link_index], query.clearance_m
                    )
                    collision |= query.clearance_m <= 0.0
    return tuple(minima), collision


def run_suite(
    scene_count: int = 2_500,
    *,
    dense_steps_per_interval: int = 100,
    subdivision_factor: int = 4,
    seed: int = 8101,
) -> dict[str, object]:
    if scene_count <= 0:
        raise ValueError("scene_count must be positive")
    if dense_steps_per_interval < 2:
        raise ValueError("dense_steps_per_interval must be at least two")
    if subdivision_factor < 1:
        raise ValueError("subdivision_factor must be at least one")

    rng = random.Random(seed)
    geometry = CapsuleArmGeometry(PlanarArm3DOF())
    bound_violations = 0
    false_safe = 0
    certified_free = 0
    dense_free = 0
    conservative_rejections = 0
    hidden_collisions = 0
    hidden_detected = 0
    slacks = []
    by_sample_count = {}

    for scene_index in range(scene_count):
        start = tuple(rng.uniform(-1.2, 1.2) for _ in range(3))
        target = tuple(rng.uniform(-1.2, 1.2) for _ in range(3))
        sample_count = rng.choice((2, 3, 5, 11))
        primitive = quintic_primitive(
            f"sweep-{scene_index:05d}",
            start,
            target,
            sample_count=sample_count,
        )

        obstacle_count = 1 if rng.random() < 0.75 else 2
        obstacles = []
        for _ in range(obstacle_count):
            blend = rng.random()
            reference = tuple(
                start[joint] + blend * (target[joint] - start[joint])
                for joint in range(3)
            )
            obstacles.append(
                _obstacle_at_link_clearance(
                    geometry,
                    reference,
                    rng.randrange(3),
                    rng.uniform(0.1, 0.9),
                    rng.uniform(-0.015, 0.080),
                    side=-1.0 if rng.random() < 0.5 else 1.0,
                    obstacle_radius_m=rng.uniform(0.010, 0.030),
                )
            )
        obstacles_tuple = tuple(obstacles)

        certificate = certify_swept_clearance(
            geometry,
            primitive,
            obstacles_tuple,
            subdivision_factor=subdivision_factor,
        )
        dense_minima, dense_collision = _dense_link_minima(
            geometry,
            primitive,
            obstacles_tuple,
            dense_steps_per_interval,
        )
        endpoint_collision = any(
            geometry.collides(angles, obstacle)
            for angles in primitive.positions
            for obstacle in obstacles_tuple
        )

        violation = any(
            dense + 1e-12 < lower
            for dense, lower in zip(
                dense_minima, certificate.link_lower_bounds_m
            )
        )
        bound_violations += violation
        certified_free += certificate.certified_collision_free
        dense_free += not dense_collision
        false_safe += certificate.certified_collision_free and dense_collision
        conservative_rejections += (
            not certificate.certified_collision_free and not dense_collision
        )
        hidden = not endpoint_collision and dense_collision
        hidden_collisions += hidden
        hidden_detected += hidden and not certificate.certified_collision_free
        slacks.append(
            min(
                dense - lower
                for dense, lower in zip(
                    dense_minima, certificate.link_lower_bounds_m
                )
            )
        )

        bucket = by_sample_count.setdefault(
            str(sample_count),
            {"scenes": 0, "certified_free": 0, "dense_free": 0},
        )
        bucket["scenes"] += 1
        bucket["certified_free"] += certificate.certified_collision_free
        bucket["dense_free"] += not dense_collision

    return {
        "protocol": (
            "seeded random quintic joint-line primitives; fixed circular "
            "obstacles; dense interpolation used only as an empirical oracle"
        ),
        "scene_count": scene_count,
        "seed": seed,
        "dense_steps_per_interval": dense_steps_per_interval,
        "subdivision_factor": subdivision_factor,
        "summary": {
            "bound_violations": bound_violations,
            "false_safe_certificates": false_safe,
            "certified_collision_free": certified_free,
            "dense_collision_free": dense_free,
            "conservative_rejections": conservative_rejections,
            "hidden_between_sample_collisions": hidden_collisions,
            "hidden_collisions_detected": hidden_detected,
            "mean_dense_minus_bound_clearance_m": sum(slacks) / len(slacks),
            "minimum_dense_minus_bound_clearance_m": min(slacks),
            "maximum_dense_minus_bound_clearance_m": max(slacks),
            "by_sample_count": by_sample_count,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", type=int, default=2_500)
    parser.add_argument("--dense-steps", type=int, default=100)
    parser.add_argument("--seed", type=int, default=8101)
    parser.add_argument("--subdivisions", type=int, default=4)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/swept_validation_results.json"),
    )
    arguments = parser.parse_args()
    payload = run_suite(
        arguments.scenes,
        dense_steps_per_interval=arguments.dense_steps,
        seed=arguments.seed,
        subdivision_factor=arguments.subdivisions,
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
