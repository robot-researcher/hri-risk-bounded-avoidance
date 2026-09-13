import unittest

from risk_bounded_collision.geometry import CapsuleArmGeometry, CircleObstacle
from risk_bounded_collision.kinematics import PlanarArm3DOF
from risk_bounded_collision.models import MotionPrimitive
from risk_bounded_collision.primitives import quintic_primitive
from risk_bounded_collision.swept import (
    certify_swept_clearance,
    link_interval_motion_bound,
    link_reach_coefficients,
    subdivide_primitive,
)


class SweptClearanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.arm = PlanarArm3DOF((0.30, 0.25, 0.15))
        self.geometry = CapsuleArmGeometry(self.arm, (0.01, 0.01, 0.01))

    def test_link_reach_coefficients_cover_all_upstream_joints(self) -> None:
        self.assertEqual(link_reach_coefficients(self.arm, 2), (0.70, 0.40, 0.15))
        self.assertEqual(link_reach_coefficients(self.arm, 0), (0.30, 0.0, 0.0))

    def test_interval_motion_bound_is_zero_for_hold(self) -> None:
        angles = (0.2, -0.1, 0.3)
        self.assertEqual(link_interval_motion_bound(self.arm, angles, angles, 2), 0.0)

    def test_subdivision_preserves_path_and_tightens_bound(self) -> None:
        primitive = MotionPrimitive(
            "coarse",
            0.5,
            ((-0.4, 0.1, 0.0), (0.5, -0.2, 0.3)),
        )
        obstacle = CircleObstacle((0.45, 0.25), 0.02)
        refined = subdivide_primitive(primitive, 4)
        self.assertEqual(refined.start, primitive.start)
        self.assertEqual(refined.terminal, primitive.terminal)
        self.assertEqual(len(refined.positions), 5)

        coarse = certify_swept_clearance(
            self.geometry, primitive, (obstacle,)
        )
        fine = certify_swept_clearance(
            self.geometry,
            primitive,
            (obstacle,),
            subdivision_factor=4,
        )
        self.assertGreaterEqual(
            fine.minimum_lower_bound_m,
            coarse.minimum_lower_bound_m - 1e-12,
        )

        with self.assertRaises(ValueError):
            subdivide_primitive(primitive, 0)

    def test_certificate_detects_collision_hidden_between_endpoints(self) -> None:
        primitive = MotionPrimitive(
            "hidden",
            0.5,
            ((-0.4, 0.0, 0.0), (0.4, 0.0, 0.0)),
        )
        obstacle = CircleObstacle((0.50, 0.0), 0.02)
        self.assertFalse(self.geometry.collides(primitive.start, obstacle))
        self.assertFalse(self.geometry.collides(primitive.terminal, obstacle))
        self.assertTrue(self.geometry.collides((0.0, 0.0, 0.0), obstacle))

        certificate = certify_swept_clearance(
            self.geometry, primitive, (obstacle,)
        )
        self.assertFalse(certificate.certified_collision_free)
        self.assertLessEqual(certificate.minimum_lower_bound_m, 0.0)

    def test_dense_clearance_never_falls_below_certificate(self) -> None:
        primitive = quintic_primitive(
            "moving",
            (-0.5, 0.3, -0.2),
            (0.6, -0.4, 0.35),
            sample_count=5,
        )
        obstacles = (
            CircleObstacle((0.20, 0.42), 0.03),
            CircleObstacle((0.58, -0.16), 0.025),
        )
        certificate = certify_swept_clearance(
            self.geometry, primitive, obstacles
        )

        dense_minima = [float("inf")] * 3
        for start, end in zip(primitive.positions, primitive.positions[1:]):
            for step in range(201):
                fraction = step / 200.0
                angles = tuple(
                    start[joint] + fraction * (end[joint] - start[joint])
                    for joint in range(3)
                )
                for link_index in range(3):
                    clearance = min(
                        self.geometry.link_clearances(angles, obstacle)[
                            link_index
                        ].clearance_m
                        for obstacle in obstacles
                    )
                    dense_minima[link_index] = min(
                        dense_minima[link_index], clearance
                    )

        for dense, lower in zip(
            dense_minima, certificate.link_lower_bounds_m
        ):
            self.assertGreaterEqual(dense + 1e-12, lower)

    def test_certificate_requires_an_obstacle(self) -> None:
        primitive = quintic_primitive(
            "hold", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
        )
        with self.assertRaises(ValueError):
            certify_swept_clearance(self.geometry, primitive, ())


if __name__ == "__main__":
    unittest.main()
