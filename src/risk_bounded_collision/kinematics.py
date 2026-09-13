"""Planar 3-DOF kinematics used by the first research prototype.

The ROS implementation will eventually replace this model with URDF-derived
kinematics. Keeping the interface small makes that substitution explicit.
"""

from dataclasses import dataclass
from math import cos, sin
from typing import Tuple

from .models import Matrix3, Vector3


@dataclass(frozen=True)
class PlanarArm3DOF:
    link_lengths_m: Vector3 = (0.30, 0.25, 0.15)

    def __post_init__(self) -> None:
        if any(length <= 0.0 for length in self.link_lengths_m):
            raise ValueError("link lengths must be positive")

    def joint_positions(self, joint_angles: Vector3) -> Tuple[Tuple[float, float], ...]:
        """Return the base and the endpoint of each link."""
        x = 0.0
        y = 0.0
        angle = 0.0
        points = [(x, y)]
        for length, joint_angle in zip(self.link_lengths_m, joint_angles):
            angle += joint_angle
            x += length * cos(angle)
            y += length * sin(angle)
            points.append((x, y))
        return tuple(points)

    def endpoint_jacobian(
        self, joint_angles: Vector3, link_index: int
    ) -> Tuple[Vector3, Vector3]:
        """Return the 2x3 translational Jacobian for a link endpoint."""
        if link_index not in (0, 1, 2):
            raise ValueError("link_index must be 0, 1, or 2")

        cumulative_angles = []
        running_angle = 0.0
        for angle in joint_angles:
            running_angle += angle
            cumulative_angles.append(running_angle)

        dx = [0.0, 0.0, 0.0]
        dy = [0.0, 0.0, 0.0]
        for joint_index in range(link_index + 1):
            for segment_index in range(joint_index, link_index + 1):
                length = self.link_lengths_m[segment_index]
                angle = cumulative_angles[segment_index]
                dx[joint_index] -= length * sin(angle)
                dy[joint_index] += length * cos(angle)
        return (tuple(dx), tuple(dy))  # type: ignore[return-value]

    def endpoint_covariance(
        self, joint_angles: Vector3, link_index: int, joint_covariance: Matrix3
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Propagate joint covariance to a link endpoint using J Sigma J^T."""
        jacobian = self.endpoint_jacobian(joint_angles, link_index)
        output = [[0.0, 0.0], [0.0, 0.0]]
        for row in range(2):
            for column in range(2):
                output[row][column] = sum(
                    jacobian[row][i] * joint_covariance[i][j] * jacobian[column][j]
                    for i in range(3)
                    for j in range(3)
                )
        return ((output[0][0], output[0][1]), (output[1][0], output[1][1]))

    def point_jacobian(
        self, joint_angles: Vector3, link_index: int, fraction: float
    ) -> Tuple[Vector3, Vector3]:
        """Jacobian of a point interpolated along a link centerline.

        ``fraction=0`` is the link start and ``fraction=1`` is its endpoint.
        For a closest point on a segment, the envelope theorem allows this
        fixed-fraction Jacobian to be used in the local clearance gradient.
        """
        if not 0.0 <= fraction <= 1.0:
            raise ValueError("fraction must be in [0, 1]")
        end = self.endpoint_jacobian(joint_angles, link_index)
        if link_index == 0:
            start = (
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
        else:
            start = self.endpoint_jacobian(joint_angles, link_index - 1)
        return (
            tuple((1.0 - fraction) * start[0][i] + fraction * end[0][i] for i in range(3)),
            tuple((1.0 - fraction) * start[1][i] + fraction * end[1][i] for i in range(3)),
        )  # type: ignore[return-value]
