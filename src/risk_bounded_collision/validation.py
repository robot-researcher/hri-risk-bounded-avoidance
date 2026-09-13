"""Seeded Monte Carlo calibration for the first-order collision-risk model."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import sqrt
import random

from .geometry import CapsuleArmGeometry, CircleObstacle
from .models import Matrix3, Vector3
from .risk import DirectionalRiskCertifier


@dataclass(frozen=True)
class CalibrationResult:
    scenario: str
    trials: int
    seed: int
    predicted_risk: float
    guard_sigma_multiplier: float
    guarded_risk: float
    collisions: int
    observed_rate: float
    wilson_upper_95: float
    guarded_covers_wilson: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class SceneCalibrationResult:
    """Calibration result for a scene containing one or more obstacles."""

    scenario: str
    trials: int
    seed: int
    obstacle_count: int
    predicted_risk: float
    guard_sigma_multiplier: float
    guarded_risk: float
    collisions: int
    observed_rate: float
    wilson_upper_95: float
    guarded_covers_wilson: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def wilson_upper_bound(successes: int, trials: int, z: float = 1.6448536269514722) -> float:
    """One-sided Wilson upper confidence bound (95% by default)."""
    if trials <= 0 or not 0 <= successes <= trials:
        raise ValueError("require 0 <= successes <= trials and trials > 0")
    proportion = successes / trials
    denominator = 1.0 + z * z / trials
    center = proportion + z * z / (2.0 * trials)
    spread = z * sqrt(proportion * (1.0 - proportion) / trials + z * z / (4.0 * trials * trials))
    return min(1.0, (center + spread) / denominator)


def _cholesky_3x3(covariance: Matrix3) -> Matrix3:
    lower = [[0.0, 0.0, 0.0] for _ in range(3)]
    for row in range(3):
        for column in range(row + 1):
            residual = covariance[row][column] - sum(
                lower[row][k] * lower[column][k] for k in range(column)
            )
            if row == column:
                if residual < -1e-15:
                    raise ValueError("covariance must be positive semidefinite")
                lower[row][column] = sqrt(max(0.0, residual))
            elif lower[column][column] > 0.0:
                lower[row][column] = residual / lower[column][column]
            elif abs(residual) > 1e-15:
                raise ValueError("invalid singular covariance")
    return tuple(tuple(row) for row in lower)  # type: ignore[return-value]


def calibrate_configuration(
    scenario: str,
    geometry: CapsuleArmGeometry,
    joint_angles: Vector3,
    obstacle: CircleObstacle,
    joint_covariance: Matrix3,
    *,
    trials: int = 10_000,
    seed: int = 1,
    guard_sigma_multiplier: float = 1.10,
) -> CalibrationResult:
    if trials <= 0:
        raise ValueError("trials must be positive")
    predicted = DirectionalRiskCertifier(geometry).configuration_risk(
        joint_angles, obstacle, joint_covariance
    )
    guarded = DirectionalRiskCertifier(
        geometry, sigma_multiplier=guard_sigma_multiplier
    ).configuration_risk(joint_angles, obstacle, joint_covariance)
    lower = _cholesky_3x3(joint_covariance)
    rng = random.Random(seed)
    collisions = 0
    for _ in range(trials):
        standard = (rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0))
        noise = tuple(
            sum(lower[row][column] * standard[column] for column in range(3))
            for row in range(3)
        )
        perturbed = tuple(joint_angles[index] + noise[index] for index in range(3))
        collisions += geometry.collides(perturbed, obstacle)  # bool is an int
    observed = collisions / trials
    return CalibrationResult(
        scenario,
        trials,
        seed,
        predicted,
        guard_sigma_multiplier,
        guarded,
        collisions,
        observed,
        wilson_upper_bound(collisions, trials),
        guarded >= wilson_upper_bound(collisions, trials),
    )


def calibrate_scene(
    scenario: str,
    geometry: CapsuleArmGeometry,
    joint_angles: Vector3,
    obstacles: tuple[CircleObstacle, ...],
    joint_covariance: Matrix3,
    *,
    trials: int = 10_000,
    seed: int = 1,
    guard_sigma_multiplier: float = 1.10,
) -> SceneCalibrationResult:
    """Calibrate a union-bound estimate against a multi-obstacle scene.

    The same perturbed configuration is checked against every obstacle in a
    trial.  This preserves correlation between collision events and avoids the
    invalid independence assumption that would result from sampling each
    obstacle separately.
    """
    if trials <= 0:
        raise ValueError("trials must be positive")
    if not obstacles:
        raise ValueError("at least one obstacle is required")

    nominal_certifier = DirectionalRiskCertifier(geometry)
    guarded_certifier = DirectionalRiskCertifier(
        geometry, sigma_multiplier=guard_sigma_multiplier
    )
    predicted = min(
        1.0,
        sum(
            nominal_certifier.configuration_risk(
                joint_angles, obstacle, joint_covariance
            )
            for obstacle in obstacles
        ),
    )
    guarded = min(
        1.0,
        sum(
            guarded_certifier.configuration_risk(
                joint_angles, obstacle, joint_covariance
            )
            for obstacle in obstacles
        ),
    )

    lower = _cholesky_3x3(joint_covariance)
    rng = random.Random(seed)
    collisions = 0
    for _ in range(trials):
        standard = (rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0))
        noise = tuple(
            sum(lower[row][column] * standard[column] for column in range(3))
            for row in range(3)
        )
        perturbed = tuple(joint_angles[index] + noise[index] for index in range(3))
        collisions += any(
            geometry.collides(perturbed, obstacle) for obstacle in obstacles
        )

    observed = collisions / trials
    upper = wilson_upper_bound(collisions, trials)
    return SceneCalibrationResult(
        scenario=scenario,
        trials=trials,
        seed=seed,
        obstacle_count=len(obstacles),
        predicted_risk=predicted,
        guard_sigma_multiplier=guard_sigma_multiplier,
        guarded_risk=guarded,
        collisions=collisions,
        observed_rate=observed,
        wilson_upper_95=upper,
        guarded_covers_wilson=guarded >= upper,
    )
