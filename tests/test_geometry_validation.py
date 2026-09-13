import unittest

from risk_bounded_collision.geometry import CapsuleArmGeometry, CircleObstacle
from risk_bounded_collision.kinematics import PlanarArm3DOF
from risk_bounded_collision.risk import DirectionalRiskCertifier
from risk_bounded_collision.validation import (
    calibrate_configuration,
    calibrate_scene,
    wilson_upper_bound,
)


class GeometryAndValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.arm = PlanarArm3DOF((0.30, 0.25, 0.15))
        self.geometry = CapsuleArmGeometry(self.arm, (0.01, 0.01, 0.01))
        variance = 0.01**2
        self.covariance = (
            (variance, 0.0, 0.0),
            (0.0, variance, 0.0),
            (0.0, 0.0, variance),
        )

    def test_capsule_clearance_uses_link_interior(self) -> None:
        obstacle = CircleObstacle((0.50, 0.10), 0.02)
        query = self.geometry.minimum_clearance((0.0, 0.0, 0.0), obstacle)
        self.assertEqual(query.link_index, 1)
        self.assertAlmostEqual(query.clearance_m, 0.07)
        self.assertGreater(query.segment_fraction, 0.0)
        self.assertLess(query.segment_fraction, 1.0)

    def test_capsule_collision(self) -> None:
        obstacle = CircleObstacle((0.50, 0.02), 0.02)
        self.assertTrue(self.geometry.collides((0.0, 0.0, 0.0), obstacle))

    def test_point_jacobian_interpolates_link(self) -> None:
        jacobian = self.arm.point_jacobian((0.0, 0.0, 0.0), 1, 0.5)
        self.assertAlmostEqual(jacobian[1][0], 0.425)
        self.assertAlmostEqual(jacobian[1][1], 0.125)
        self.assertAlmostEqual(jacobian[1][2], 0.0)

    def test_point_jacobian_matches_finite_difference(self) -> None:
        joint_angles = (0.25, -0.35, 0.20)
        link_index = 2
        fraction = 0.4
        analytical = self.arm.point_jacobian(joint_angles, link_index, fraction)
        step = 1e-6

        def point_at(angles):
            points = self.arm.joint_positions(angles)
            start = points[link_index]
            end = points[link_index + 1]
            return (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )

        for joint in range(3):
            plus = list(joint_angles)
            minus = list(joint_angles)
            plus[joint] += step
            minus[joint] -= step
            plus_point = point_at(tuple(plus))
            minus_point = point_at(tuple(minus))
            numerical = (
                (plus_point[0] - minus_point[0]) / (2.0 * step),
                (plus_point[1] - minus_point[1]) / (2.0 * step),
            )
            self.assertAlmostEqual(analytical[0][joint], numerical[0], places=7)
            self.assertAlmostEqual(analytical[1][joint], numerical[1], places=7)

    def test_clearance_gradient_matches_finite_difference(self) -> None:
        joint_angles = (0.0, 0.0, 0.0)
        obstacle = CircleObstacle((0.50, 0.10), 0.02)
        query = self.geometry.minimum_clearance(joint_angles, obstacle)
        jacobian = self.arm.point_jacobian(
            joint_angles, query.link_index, query.segment_fraction
        )
        analytical = tuple(
            query.normal[0] * jacobian[0][joint]
            + query.normal[1] * jacobian[1][joint]
            for joint in range(3)
        )
        step = 1e-6
        for joint in range(3):
            plus = list(joint_angles)
            minus = list(joint_angles)
            plus[joint] += step
            minus[joint] -= step
            plus_clearance = self.geometry.minimum_clearance(tuple(plus), obstacle).clearance_m
            minus_clearance = self.geometry.minimum_clearance(tuple(minus), obstacle).clearance_m
            numerical = (plus_clearance - minus_clearance) / (2.0 * step)
            self.assertAlmostEqual(analytical[joint], numerical, places=7)

    def test_directional_risk_decreases_with_clearance(self) -> None:
        certifier = DirectionalRiskCertifier(self.geometry)
        near = CircleObstacle((0.50, 0.04), 0.02)
        far = CircleObstacle((0.50, 0.10), 0.02)
        near_risk = certifier.configuration_risk((0.0, 0.0, 0.0), near, self.covariance)
        far_risk = certifier.configuration_risk((0.0, 0.0, 0.0), far, self.covariance)
        self.assertGreater(near_risk, far_risk)

    def test_sigma_guard_increases_estimated_risk(self) -> None:
        obstacle = CircleObstacle((0.50, 0.05), 0.02)
        nominal = DirectionalRiskCertifier(self.geometry).configuration_risk(
            (0.0, 0.0, 0.0), obstacle, self.covariance
        )
        guarded = DirectionalRiskCertifier(
            self.geometry, sigma_multiplier=1.10
        ).configuration_risk((0.0, 0.0, 0.0), obstacle, self.covariance)
        self.assertGreater(guarded, nominal)

    def test_wilson_upper_bound_for_zero_events(self) -> None:
        upper = wilson_upper_bound(0, 10_000)
        self.assertGreater(upper, 0.0)
        self.assertLess(upper, 0.001)

    def test_seeded_calibration_is_reproducible(self) -> None:
        obstacle = CircleObstacle((0.50, 0.08), 0.02)
        first = calibrate_configuration(
            "test", self.geometry, (0.0, 0.0, 0.0), obstacle, self.covariance,
            trials=200, seed=19,
        )
        second = calibrate_configuration(
            "test", self.geometry, (0.0, 0.0, 0.0), obstacle, self.covariance,
            trials=200, seed=19,
        )
        self.assertEqual(first, second)

    def test_scene_calibration_supports_correlated_noise_and_multiple_obstacles(self) -> None:
        variance = 0.01**2
        covariance = (
            (variance, 0.4 * variance, 0.0),
            (0.4 * variance, variance, 0.2 * variance),
            (0.0, 0.2 * variance, variance),
        )
        obstacles = (
            CircleObstacle((0.40, 0.06), 0.02),
            CircleObstacle((0.60, -0.07), 0.02),
        )
        result = calibrate_scene(
            "multi",
            self.geometry,
            (0.0, 0.0, 0.0),
            obstacles,
            covariance,
            trials=200,
            seed=23,
        )
        self.assertEqual(result.obstacle_count, 2)
        self.assertEqual(result.trials, 200)
        self.assertGreaterEqual(result.guarded_risk, result.predicted_risk)
        self.assertGreaterEqual(result.wilson_upper_95, result.observed_rate)

    def test_scene_calibration_requires_an_obstacle(self) -> None:
        with self.assertRaises(ValueError):
            calibrate_scene(
                "empty", self.geometry, (0.0, 0.0, 0.0), (), self.covariance
            )


if __name__ == "__main__":
    unittest.main()
