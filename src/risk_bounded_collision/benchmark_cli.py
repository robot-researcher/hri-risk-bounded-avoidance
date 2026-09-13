"""Measure reference-pipeline latency on the current development machine."""

from __future__ import annotations

import argparse
import json
import platform
from math import pi
from pathlib import Path
from statistics import median
from time import perf_counter_ns

from .kinematics import PlanarArm3DOF
from .models import Matrix3
from .primitives import generate_primitives
from .risk import RiskCertifier, adaptive_risk_threshold
from .selector import PrimitiveSelector
from .storage import save_library


def _covariance() -> Matrix3:
    variance = (pi / 180.0) ** 2
    return ((variance, 0.0, 0.0), (0.0, variance, 0.0), (0.0, 0.0, variance))


def _percentile(sorted_values: list[float], fraction: float) -> float:
    index = min(len(sorted_values) - 1, int(fraction * (len(sorted_values) - 1)))
    return sorted_values[index]


def run_benchmark(iterations: int, artifact_path: Path) -> dict[str, object]:
    if iterations <= 0:
        raise ValueError("iterations must be positive")
    covariance = _covariance()
    primitives = generate_primitives((0.0, 0.0, 0.0), count=64)
    bins = tuple(0.05 * index for index in range(1, 21))

    build_start = perf_counter_ns()
    library = RiskCertifier(PlanarArm3DOF()).build_library(primitives, bins, covariance)
    build_ms = (perf_counter_ns() - build_start) / 1_000_000.0
    save_library(library, artifact_path)
    selector = PrimitiveSelector(library)

    timings_us = []
    fallback_count = 0
    for index in range(iterations):
        distance = 0.05 + 0.95 * ((index % 101) / 100.0)
        start = perf_counter_ns()
        threshold = adaptive_risk_threshold(distance, covariance)
        result = selector.select(
            obstacle_distance_m=distance,
            threshold=threshold,
            goal_joint_angles=(0.30, -0.20, 0.15),
        )
        timings_us.append((perf_counter_ns() - start) / 1_000.0)
        fallback_count += result.fallback

    timings_us.sort()
    return {
        "environment": platform.platform(),
        "primitive_count": len(primitives),
        "distance_bin_count": len(bins),
        "certificate_entries": len(primitives) * len(bins),
        "offline_build_ms": build_ms,
        "online_iterations": iterations,
        "online_latency_us": {
            "median": median(timings_us),
            "p95": _percentile(timings_us, 0.95),
            "p99": _percentile(timings_us, 0.99),
            "maximum": max(timings_us),
        },
        "fallback_count": fallback_count,
        "artifact_bytes": artifact_path.stat().st_size,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=20_000)
    parser.add_argument("--output", type=Path, default=Path("artifacts/benchmark_results.json"))
    arguments = parser.parse_args()
    certificate_path = Path("artifacts/benchmark_certificate_library.json")
    payload = run_benchmark(arguments.iterations, certificate_path)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
