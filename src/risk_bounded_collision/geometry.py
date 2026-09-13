"""Complete-link capsule geometry for planar collision queries."""

from dataclasses import dataclass
from math import hypot
from typing import Tuple

from .kinematics import PlanarArm3DOF
from .models import Vector3

Point2 = Tuple[float, float]


@dataclass(frozen=True)
class CircleObstacle:
    center_m: Point2
    radius_m: float

    def __post_init__(self) -> None:
        if self.radius_m <= 0.0:
            raise ValueError("obstacle radius must be positive")


@dataclass(frozen=True)
class ClearanceQuery:
    link_index: int
    segment_fraction: float
    closest_point_m: Point2
    normal: Point2
    center_distance_m: float
    clearance_m: float


@dataclass(frozen=True)
class CapsuleArmGeometry:
    """Represent each planar link as a line segment expanded by a radius."""

    arm: PlanarArm3DOF
    link_radii_m: Vector3 = (0.018, 0.016, 0.014)

    def __post_init__(self) -> None:
        if any(radius <= 0.0 for radius in self.link_radii_m):
            raise ValueError("link radii must be positive")

    @staticmethod
    def _closest_point(start: Point2, end: Point2, point: Point2) -> Tuple[Point2, float]:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        squared_length = dx * dx + dy * dy
        if squared_length == 0.0:
            return start, 0.0
        fraction = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / squared_length
        fraction = max(0.0, min(1.0, fraction))
        return (start[0] + fraction * dx, start[1] + fraction * dy), fraction

    def link_clearances(
        self, joint_angles: Vector3, obstacle: CircleObstacle
    ) -> Tuple[ClearanceQuery, ...]:
        points = self.arm.joint_positions(joint_angles)
        queries = []
        for link_index in range(3):
            closest, fraction = self._closest_point(
                points[link_index], points[link_index + 1], obstacle.center_m
            )
            offset_x = closest[0] - obstacle.center_m[0]
            offset_y = closest[1] - obstacle.center_m[1]
            center_distance = hypot(offset_x, offset_y)
            if center_distance > 1e-15:
                normal = (offset_x / center_distance, offset_y / center_distance)
            else:
                segment_x = points[link_index + 1][0] - points[link_index][0]
                segment_y = points[link_index + 1][1] - points[link_index][1]
                segment_length = hypot(segment_x, segment_y)
                normal = (-segment_y / segment_length, segment_x / segment_length)
            clearance = center_distance - obstacle.radius_m - self.link_radii_m[link_index]
            queries.append(
                ClearanceQuery(
                    link_index,
                    fraction,
                    closest,
                    normal,
                    center_distance,
                    clearance,
                )
            )
        return tuple(queries)

    def minimum_clearance(self, joint_angles: Vector3, obstacle: CircleObstacle) -> ClearanceQuery:
        return min(self.link_clearances(joint_angles, obstacle), key=lambda query: query.clearance_m)

    def collides(self, joint_angles: Vector3, obstacle: CircleObstacle) -> bool:
        return self.minimum_clearance(joint_angles, obstacle).clearance_m <= 0.0
