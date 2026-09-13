"""Quintic joint-space motion primitive generation."""

from dataclasses import dataclass
import random
from typing import List, Tuple

from .models import MotionPrimitive, Vector3


@dataclass(frozen=True)
class PrimitiveLimitResult:
    valid: bool
    maximum_velocity_rad_s: Vector3
    maximum_acceleration_rad_s2: Vector3
    velocity_limits_rad_s: Vector3
    acceleration_limits_rad_s2: Vector3


def validate_sampled_dynamics(
    primitive: MotionPrimitive,
    *,
    velocity_limits_rad_s: Vector3,
    acceleration_limits_rad_s2: Vector3,
) -> PrimitiveLimitResult:
    """Check sampled finite-difference velocity and acceleration limits."""
    if any(limit <= 0.0 for limit in velocity_limits_rad_s):
        raise ValueError("velocity limits must be positive")
    if any(limit <= 0.0 for limit in acceleration_limits_rad_s2):
        raise ValueError("acceleration limits must be positive")

    step_s = primitive.duration_s / (len(primitive.positions) - 1)
    velocities = tuple(
        tuple(
            (current[joint] - previous[joint]) / step_s for joint in range(3)
        )
        for previous, current in zip(primitive.positions, primitive.positions[1:])
    )
    accelerations = tuple(
        tuple(
            (current[joint] - previous[joint]) / step_s for joint in range(3)
        )
        for previous, current in zip(velocities, velocities[1:])
    )
    maximum_velocity = tuple(
        max(abs(velocity[joint]) for velocity in velocities) for joint in range(3)
    )
    maximum_acceleration = tuple(
        max((abs(value[joint]) for value in accelerations), default=0.0)
        for joint in range(3)
    )
    valid = all(
        maximum_velocity[joint] <= velocity_limits_rad_s[joint]
        and maximum_acceleration[joint] <= acceleration_limits_rad_s2[joint]
        for joint in range(3)
    )
    return PrimitiveLimitResult(
        valid=valid,
        maximum_velocity_rad_s=maximum_velocity,  # type: ignore[arg-type]
        maximum_acceleration_rad_s2=maximum_acceleration,  # type: ignore[arg-type]
        velocity_limits_rad_s=velocity_limits_rad_s,
        acceleration_limits_rad_s2=acceleration_limits_rad_s2,
    )


def _blend(s: float) -> float:
    """Minimum-jerk quintic blend with zero endpoint velocity/acceleration."""
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def quintic_primitive(
    primitive_id: str,
    start: Vector3,
    target: Vector3,
    *,
    duration_s: float = 0.5,
    sample_count: int = 11,
) -> MotionPrimitive:
    if sample_count < 2:
        raise ValueError("sample_count must be at least two")
    positions = []
    for index in range(sample_count):
        s = index / (sample_count - 1)
        weight = _blend(s)
        positions.append(tuple(a + (b - a) * weight for a, b in zip(start, target)))
    return MotionPrimitive(primitive_id, duration_s, tuple(positions))


def generate_primitives(
    start: Vector3,
    *,
    count: int = 64,
    max_offset_rad: float = 0.45,
    joint_limits_rad: Tuple[Vector3, Vector3] = (
        (-1.5707963268, -1.5707963268, -1.5707963268),
        (1.5707963268, 1.5707963268, 1.5707963268),
    ),
    seed: int = 7,
    duration_s: float = 0.5,
    sample_count: int = 11,
) -> Tuple[MotionPrimitive, ...]:
    """Generate deterministic Latin-hypercube targets around the current state."""
    if count <= 0:
        raise ValueError("count must be positive")
    if max_offset_rad <= 0.0:
        raise ValueError("max_offset_rad must be positive")

    rng = random.Random(seed)
    strata_by_joint: List[List[float]] = []
    for _ in range(3):
        strata = [(index + rng.random()) / count for index in range(count)]
        rng.shuffle(strata)
        strata_by_joint.append(strata)

    lower, upper = joint_limits_rad
    primitives = []
    for primitive_index in range(count):
        offsets = tuple(
            (2.0 * strata_by_joint[joint][primitive_index] - 1.0) * max_offset_rad
            for joint in range(3)
        )
        target = tuple(
            min(upper[joint], max(lower[joint], start[joint] + offsets[joint]))
            for joint in range(3)
        )
        primitives.append(
            quintic_primitive(
                f"primitive-{primitive_index:04d}",
                start,
                target,  # type: ignore[arg-type]
                duration_s=duration_s,
                sample_count=sample_count,
            )
        )
    return tuple(primitives)
