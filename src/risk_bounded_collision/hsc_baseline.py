"""Reduced Hybrid Safety Certificate collision-query reproduction.

This module reproduces the certificate-query kernel from Shi, Chen, and Li,
not the complete BiHSC tree planner. Collision-free configurations create
random configuration-space balls. Colliding configurations create sufficient
workspace certificates from an interior link point and circular obstacle.
False collision-free certificates are contracted by the paper's factor 0.9.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import dist, hypot
import random

from .geometry import CapsuleArmGeometry, CircleObstacle
from .models import Vector3


@dataclass
class FreeCertificate:
    center: Vector3
    radius: float

    def contains(self, joint_angles: Vector3) -> bool:
        return dist(self.center, joint_angles) <= self.radius


@dataclass(frozen=True)
class CollisionCertificate:
    link_index: int
    segment_fraction: float
    obstacle: CircleObstacle
    combined_radius_m: float

    def contains(
        self, geometry: CapsuleArmGeometry, joint_angles: Vector3
    ) -> bool:
        points = geometry.arm.joint_positions(joint_angles)
        start = points[self.link_index]
        end = points[self.link_index + 1]
        point = (
            start[0] + self.segment_fraction * (end[0] - start[0]),
            start[1] + self.segment_fraction * (end[1] - start[1]),
        )
        return hypot(
            point[0] - self.obstacle.center_m[0],
            point[1] - self.obstacle.center_m[1],
        ) <= self.combined_radius_m


@dataclass(frozen=True)
class HSCDecision:
    collision: bool
    source: str
    exact_check: bool
    free_certificate_index: int | None = None


class ReducedHSCKernel:
    """Bounded certificate store for a reduced HSC query experiment."""

    def __init__(
        self,
        geometry: CapsuleArmGeometry,
        *,
        max_free_certificates: int = 128,
        max_collision_certificates: int = 128,
        free_radius_range: tuple[float, float] = (0.04, 0.18),
        contraction: float = 0.9,
        seed: int = 1,
    ) -> None:
        if max_free_certificates <= 0 or max_collision_certificates <= 0:
            raise ValueError("certificate capacities must be positive")
        if not 0.0 < free_radius_range[0] <= free_radius_range[1]:
            raise ValueError("invalid free-certificate radius range")
        if not 0.0 < contraction < 1.0:
            raise ValueError("contraction must be in (0, 1)")
        self.geometry = geometry
        self.max_free_certificates = max_free_certificates
        self.max_collision_certificates = max_collision_certificates
        self.free_radius_range = free_radius_range
        self.contraction = contraction
        self.free_certificates: list[FreeCertificate] = []
        self.collision_certificates: list[CollisionCertificate] = []
        self._rng = random.Random(seed)

    def exact_collision(
        self, joint_angles: Vector3, obstacles: tuple[CircleObstacle, ...]
    ) -> bool:
        return any(
            self.geometry.collides(joint_angles, obstacle) for obstacle in obstacles
        )

    def learn_exact(
        self, joint_angles: Vector3, obstacles: tuple[CircleObstacle, ...]
    ) -> bool:
        """Perform an exact query and add one corresponding certificate."""
        if not obstacles:
            raise ValueError("at least one obstacle is required")
        queries = [
            (self.geometry.minimum_clearance(joint_angles, obstacle), obstacle)
            for obstacle in obstacles
        ]
        query, obstacle = min(queries, key=lambda item: item[0].clearance_m)
        collision = query.clearance_m <= 0.0
        if collision and len(self.collision_certificates) < self.max_collision_certificates:
            self.collision_certificates.append(
                CollisionCertificate(
                    link_index=query.link_index,
                    segment_fraction=query.segment_fraction,
                    obstacle=obstacle,
                    combined_radius_m=(
                        self.geometry.link_radii_m[query.link_index]
                        + obstacle.radius_m
                    ),
                )
            )
        elif not collision and len(self.free_certificates) < self.max_free_certificates:
            self.free_certificates.append(
                FreeCertificate(
                    center=joint_angles,
                    radius=self._rng.uniform(*self.free_radius_range),
                )
            )
        return collision

    def classify(
        self,
        joint_angles: Vector3,
        obstacles: tuple[CircleObstacle, ...],
        *,
        learn_on_miss: bool = False,
    ) -> HSCDecision:
        for certificate in self.collision_certificates:
            if certificate.contains(self.geometry, joint_angles):
                return HSCDecision(True, "workspace_collision_certificate", False)
        for index, certificate in enumerate(self.free_certificates):
            if certificate.contains(joint_angles):
                return HSCDecision(
                    False, "configuration_free_certificate", False, index
                )
        collision = self.exact_collision(joint_angles, obstacles)
        if learn_on_miss:
            self.learn_exact(joint_angles, obstacles)
        return HSCDecision(collision, "exact_geometry", True)

    def rectify_false_free(
        self, joint_angles: Vector3, free_certificate_index: int
    ) -> float:
        """Apply alpha <- 0.9 ||q_f-q|| after a false-free certificate."""
        certificate = self.free_certificates[free_certificate_index]
        certificate.radius = min(
            certificate.radius,
            self.contraction * dist(certificate.center, joint_angles),
        )
        return certificate.radius
