"""Conservative continuous swept-clearance bounds for planar capsule links."""

from dataclasses import dataclass
from typing import Tuple

from .geometry import CapsuleArmGeometry, CircleObstacle
from .kinematics import PlanarArm3DOF
from .models import MotionPrimitive, Vector3


@dataclass(frozen=True)
class SweptClearanceCertificate:
    """Lower bounds on clearance over linearly interpolated joint intervals."""

    link_lower_bounds_m: Vector3
    maximum_half_sweep_margins_m: Vector3
    minimum_lower_bound_m: float
    certified_collision_free: bool


def subdivide_primitive(
    primitive: MotionPrimitive, factor: int
) -> MotionPrimitive:
    """Insert joint-line points without changing the primitive's path image."""
    if factor < 1:
        raise ValueError("factor must be at least one")
    if factor == 1:
        return primitive

    positions = []
    for start, end in zip(primitive.positions, primitive.positions[1:]):
        for step in range(factor):
            fraction = step / factor
            positions.append(
                tuple(
                    start[joint]
                    + fraction * (end[joint] - start[joint])
                    for joint in range(3)
                )
            )
    positions.append(primitive.terminal)
    return MotionPrimitive(
        primitive.primitive_id,
        primitive.duration_s,
        tuple(positions),
    )

def link_reach_coefficients(arm: PlanarArm3DOF, link_index: int) -> Vector3:
    """Maximum point displacement per radian for every upstream joint."""
    if link_index not in (0, 1, 2):
        raise ValueError("link_index must be 0, 1, or 2")
    return tuple(
        sum(arm.link_lengths_m[joint : link_index + 1])
        if joint <= link_index
        else 0.0
        for joint in range(3)
    )  # type: ignore[return-value]


def link_interval_motion_bound(
    arm: PlanarArm3DOF,
    start: Vector3,
    end: Vector3,
    link_index: int,
) -> float:
    """Bound the Hausdorff displacement of a link over a joint-line segment."""
    coefficients = link_reach_coefficients(arm, link_index)
    return sum(
        coefficients[joint] * abs(end[joint] - start[joint])
        for joint in range(3)
    )


def primitive_link_sweep_margins(
    arm: PlanarArm3DOF, primitive: MotionPrimitive
) -> Vector3:
    """Return worst half-interval motion erosion for each link.

    Any configuration on an interpolated interval is at most half of the full
    interval motion bound from its nearer endpoint. Quintic primitives follow
    the same joint-space line segment, so the bound covers their continuous
    geometric path even though their time parameterization is nonlinear.
    """
    return tuple(
        max(
            0.5 * link_interval_motion_bound(arm, start, end, link_index)
            for start, end in zip(primitive.positions, primitive.positions[1:])
        )
        for link_index in range(3)
    )  # type: ignore[return-value]


def certify_swept_clearance(
    geometry: CapsuleArmGeometry,
    primitive: MotionPrimitive,
    obstacles: Tuple[CircleObstacle, ...],
    *,
    subdivision_factor: int = 1,
) -> SweptClearanceCertificate:
    """Certify continuous nominal clearance to fixed circular obstacles."""
    if not obstacles:
        raise ValueError("at least one obstacle is required")
    primitive = subdivide_primitive(primitive, subdivision_factor)

    link_bounds = []
    margins = []
    for link_index in range(3):
        interval_lowers = []
        interval_margins = []
        for start, end in zip(primitive.positions, primitive.positions[1:]):
            start_clearance = min(
                geometry.link_clearances(start, obstacle)[link_index].clearance_m
                for obstacle in obstacles
            )
            end_clearance = min(
                geometry.link_clearances(end, obstacle)[link_index].clearance_m
                for obstacle in obstacles
            )
            margin = 0.5 * link_interval_motion_bound(
                geometry.arm, start, end, link_index
            )
            interval_margins.append(margin)
            interval_lowers.append(min(start_clearance, end_clearance) - margin)
        link_bounds.append(min(interval_lowers))
        margins.append(max(interval_margins))

    minimum = min(link_bounds)
    return SweptClearanceCertificate(
        link_lower_bounds_m=tuple(link_bounds),  # type: ignore[arg-type]
        maximum_half_sweep_margins_m=tuple(margins),  # type: ignore[arg-type]
        minimum_lower_bound_m=minimum,
        certified_collision_free=minimum > 0.0,
    )
