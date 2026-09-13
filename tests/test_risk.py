import unittest

from risk_bounded_collision.kinematics import PlanarArm3DOF
from risk_bounded_collision.primitives import quintic_primitive
from risk_bounded_collision.risk import (
    RiskCertifier,
    RollingCovarianceEstimator,
    adaptive_risk_threshold,
    validate_joint_covariance,
)


class RiskTests(unittest.TestCase):
    def setUp(self) -> None:
        variance = 0.02**2
        self.covariance = (
            (variance, 0.0, 0.0),
            (0.0, variance, 0.0),
            (0.0, 0.0, variance),
        )

    def test_risk_decreases_with_clearance(self) -> None:
        primitive = quintic_primitive("p", (0.0, 0.0, 0.0), (0.1, -0.1, 0.05))
        certifier = RiskCertifier(PlanarArm3DOF())
        close_risk = certifier.primitive_risk(primitive, 0.005, self.covariance)
        far_risk = certifier.primitive_risk(primitive, 0.05, self.covariance)
        self.assertGreater(close_risk, far_risk)

    def test_threshold_is_monotonic(self) -> None:
        low_noise = self.covariance
        high_noise = tuple(
            tuple(value * 4.0 for value in row) for row in self.covariance
        )
        near = adaptive_risk_threshold(0.10, low_noise)
        far = adaptive_risk_threshold(0.50, low_noise)
        noisy = adaptive_risk_threshold(0.50, high_noise)  # type: ignore[arg-type]
        self.assertGreater(far, near)
        self.assertLess(noisy, far)

    def test_covariance_estimator_detects_variation(self) -> None:
        estimator = RollingCovarianceEstimator(window_size=3)
        estimator.add((0.0, 1.0, 2.0))
        estimator.add((0.1, 1.0, 1.9))
        estimator.add((-0.1, 1.0, 2.1))
        covariance = estimator.covariance()
        self.assertAlmostEqual(covariance[0][0], 0.01)
        self.assertEqual(covariance[1][1], estimator.variance_floor)
        self.assertLess(covariance[0][2], 0.0)

    def test_complete_link_certificate_union_bounds_link_risks(self) -> None:
        from math import erf, sqrt

        primitive = quintic_primitive("p", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        arm = PlanarArm3DOF()
        certifier = RiskCertifier(arm, sigma_multiplier=1.0)
        clearance = 0.02
        link_risks = []
        for link_index in range(3):
            covariance = arm.endpoint_covariance(
                primitive.start, link_index, self.covariance
            )
            a, b = covariance[0]
            _, d = covariance[1]
            eigenvalue = 0.5 * (a + d + sqrt((a - d) ** 2 + 4.0 * b * b))
            sigma = sqrt(max(certifier.variance_floor, eigenvalue))
            link_risks.append(0.5 * (1.0 + erf((-clearance / sigma) / sqrt(2.0))))
        self.assertAlmostEqual(
            certifier.primitive_risk(primitive, clearance, self.covariance),
            min(1.0, sum(link_risks)),
        )

    def test_invalid_covariance_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            validate_joint_covariance(
                ((1.0, 2.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
            )
        with self.assertRaises(ValueError):
            validate_joint_covariance(
                ((1.0, 2.0, 0.0), (2.0, 1.0, 0.0), (0.0, 0.0, 1.0))
            )


    def test_library_uses_continuous_swept_bound(self) -> None:
        primitive = quintic_primitive(
            "stationary", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), sample_count=2
        )
        certifier = RiskCertifier(PlanarArm3DOF(), sigma_multiplier=1.0)
        clearance = 0.03
        marginal = certifier.primitive_risk(primitive, clearance, self.covariance)
        continuous = certifier.continuous_swept_risk(
            primitive, clearance, self.covariance
        )
        library = certifier.build_library(
            (primitive,), (clearance,), self.covariance
        )
        self.assertGreaterEqual(continuous, marginal)
        self.assertAlmostEqual(library.risks[0][0], continuous)

    def test_subdivision_tightens_continuous_risk(self) -> None:
        primitive = quintic_primitive(
            "coarse",
            (-0.4, 0.2, -0.1),
            (0.5, -0.3, 0.4),
            sample_count=2,
        )
        coarse = RiskCertifier(
            PlanarArm3DOF(), sigma_multiplier=1.0, sweep_subdivisions=1
        ).continuous_swept_risk(primitive, 0.20, self.covariance)
        fine = RiskCertifier(
            PlanarArm3DOF(), sigma_multiplier=1.0, sweep_subdivisions=8
        ).continuous_swept_risk(primitive, 0.20, self.covariance)
        self.assertLessEqual(fine, coarse)

        with self.assertRaises(ValueError):
            RiskCertifier(PlanarArm3DOF(), sweep_subdivisions=0)

if __name__ == "__main__":
    unittest.main()
