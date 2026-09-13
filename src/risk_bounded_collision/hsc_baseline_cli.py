"""Benchmark the reduced HSC collision-query kernel over declared seeds."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from random import Random
from statistics import mean, median
from time import perf_counter_ns

from .geometry import CapsuleArmGeometry, CircleObstacle
from .hsc_baseline import ReducedHSCKernel
from .kinematics import PlanarArm3DOF


def _samples(seed: int, count: int) -> tuple[tuple[float, float, float], ...]:
    rng = Random(seed)
    return tuple(
        tuple(rng.uniform(-1.2, 1.2) for _ in range(3)) for _ in range(count)
    )


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(fraction * len(ordered)))
    return ordered[index]


def run_trial(seed: int, training_queries: int, test_queries: int) -> dict[str, object]:
    geometry = CapsuleArmGeometry(PlanarArm3DOF())
    obstacles = (
        CircleObstacle((0.42, 0.12), 0.075),
        CircleObstacle((0.58, -0.10), 0.060),
    )
    kernel = ReducedHSCKernel(geometry, seed=seed)

    for joint_angles in _samples(10_000 + seed, training_queries):
        kernel.learn_exact(joint_angles, obstacles)

    test_samples = _samples(20_000 + seed, test_queries)
    exact_start = perf_counter_ns()
    truth = tuple(kernel.exact_collision(q, obstacles) for q in test_samples)
    exact_elapsed = perf_counter_ns() - exact_start

    decisions = []
    query_latencies_us = []
    false_free_before = 0
    collision_certificate_false_positives = 0
    for joint_angles, actual_collision in zip(test_samples, truth):
        start = perf_counter_ns()
        decision = kernel.classify(joint_angles, obstacles)
        query_latencies_us.append((perf_counter_ns() - start) / 1000.0)
        decisions.append(decision)
        if (
            decision.source == "configuration_free_certificate"
            and actual_collision
        ):
            false_free_before += 1
            kernel.rectify_false_free(
                joint_angles, decision.free_certificate_index  # type: ignore[arg-type]
            )
        if (
            decision.source == "workspace_collision_certificate"
            and not actual_collision
        ):
            collision_certificate_false_positives += 1

    replay = tuple(kernel.classify(q, obstacles) for q in test_samples)
    false_free_after = sum(
        decision.source == "configuration_free_certificate" and actual_collision
        for decision, actual_collision in zip(replay, truth)
    )
    exact_misses = sum(decision.exact_check for decision in decisions)
    collision_hits = sum(
        decision.source == "workspace_collision_certificate"
        for decision in decisions
    )
    free_hits = sum(
        decision.source == "configuration_free_certificate"
        for decision in decisions
    )
    proxy_hits = collision_hits + free_hits
    return {
        "seed": seed,
        "training_queries": training_queries,
        "test_queries": test_queries,
        "collision_prevalence": sum(truth) / test_queries,
        "free_certificates": len(kernel.free_certificates),
        "collision_certificates": len(kernel.collision_certificates),
        "proxy_hits": proxy_hits,
        "proxy_hit_rate": proxy_hits / test_queries,
        "exact_checks_required": exact_misses,
        "exact_check_reduction": 1.0 - exact_misses / test_queries,
        "workspace_collision_hits": collision_hits,
        "configuration_free_hits": free_hits,
        "false_free_before_rectification": false_free_before,
        "false_free_rate_before_rectification": false_free_before / test_queries,
        "false_free_after_rectification": false_free_after,
        "false_free_rate_after_rectification": false_free_after / test_queries,
        "collision_certificate_false_positives": collision_certificate_false_positives,
        "exact_geometry_latency_us_per_query": exact_elapsed / test_queries / 1000.0,
        "hsc_query_latency_us": {
            "median": median(query_latencies_us),
            "p95": _percentile(query_latencies_us, 0.95),
            "p99": _percentile(query_latencies_us, 0.99),
            "maximum": max(query_latencies_us),
        },
    }


def run_suite(
    runs: int = 20, training_queries: int = 1_000, test_queries: int = 5_000
) -> dict[str, object]:
    if runs <= 0 or training_queries <= 0 or test_queries <= 0:
        raise ValueError("runs and query counts must be positive")
    results = [
        run_trial(seed, training_queries, test_queries)
        for seed in range(1, runs + 1)
    ]
    return {
        "scope": "reduced HSC collision-query kernel; not full BiHSC/BiHSC* planner",
        "source_paper": "Shi, Chen, and Li, IEEE RA-L 2023",
        "author_source_commit": "36e389d6af993a41212efe03862967df8e7dd5e3",
        "runs": runs,
        "training_queries_per_run": training_queries,
        "test_queries_per_run": test_queries,
        "total_test_queries": runs * test_queries,
        "summary": {
            "mean_collision_prevalence": mean(
                result["collision_prevalence"] for result in results
            ),
            "mean_proxy_hit_rate": mean(
                result["proxy_hit_rate"] for result in results
            ),
            "mean_exact_check_reduction": mean(
                result["exact_check_reduction"] for result in results
            ),
            "total_false_free_before_rectification": sum(
                result["false_free_before_rectification"] for result in results
            ),
            "total_false_free_after_rectification": sum(
                result["false_free_after_rectification"] for result in results
            ),
            "total_collision_certificate_false_positives": sum(
                result["collision_certificate_false_positives"]
                for result in results
            ),
            "mean_exact_geometry_latency_us_per_query": mean(
                result["exact_geometry_latency_us_per_query"] for result in results
            ),
            "mean_hsc_median_latency_us": mean(
                result["hsc_query_latency_us"]["median"] for result in results
            ),
            "mean_hsc_p99_latency_us": mean(
                result["hsc_query_latency_us"]["p99"] for result in results
            ),
        },
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--training-queries", type=int, default=1_000)
    parser.add_argument("--test-queries", type=int, default=5_000)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/hsc_reduced_baseline_results.json"),
    )
    arguments = parser.parse_args()
    payload = run_suite(
        arguments.runs, arguments.training_queries, arguments.test_queries
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
