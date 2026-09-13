"""Offline risk certification and online uncertainty/threshold estimation."""

from collections import deque
from math import erf, exp, isfinite, sqrt
from typing import Deque, Tuple

from .kinematics import PlanarArm3DOF
from .geometry import CapsuleArmGeometry, CircleObstacle
from .models import CertificateLibrary, Matrix3, MotionPrimitive, Vector3
from .swept import link_reach_coefficients, primitive_link_sweep_margins, subdivide_primitive


def _normal_cdf(value: float) -> float:
    return 0.5 * (1.0 + erf(value / sqrt(2.0)))


def _largest_eigenvalue_2x2(matrix: Tuple[Tuple[float, float], Tuple[float, float]]) -> float:
    a, b = matrix[0]
    _, d = matrix[1]
    discriminant = max(0.0, (a - d) ** 2 + 4.0 * b * b)
    return max(0.0, 0.5 * (a + d + sqrt(discriminant)))


def covariance_trace(covariance: Matrix3) -> float:
    return covariance[0][0] + covariance[1][1] + covariance[2][2]

def validate_joint_covariance(covariance: Matrix3, *, tolerance: float = 1e-12) -> None:
    """Reject non-finite, asymmetric, or non-positive-semidefinite covariance."""
    if tolerance < 0.0:
        raise ValueError("tolerance must be non-negative")
    if any(not isfinite(value) for row in covariance for value in row):
        raise ValueError("covariance entries must be finite")
    if any(
        abs(covariance[i][j] - covariance[j][i]) > tolerance
        for i in range(3)
        for j in range(i)
    ):
        raise ValueError("covariance must be symmetric")
    if any(covariance[i][i] < -tolerance for i in range(3)):
        raise ValueError("covariance must be positive semidefinite")
    for i, j in ((0, 1), (0, 2), (1, 2)):
        minor = covariance[i][i] * covariance[j][j] - covariance[i][j] ** 2
        if minor < -tolerance:
            raise ValueError("covariance must be positive semidefinite")
    a, b, c = covariance[0]
    _, d, e = covariance[1]
    _, _, f = covariance[2]
    determinant = (
        a * (d * f - e * e)
        - b * (b * f - c * e)
        + c * (b * e - c * d)
    )
    if determinant < -tolerance:
        raise ValueError("covariance must be positive semidefinite")



def adaptive_risk_threshold(
    obstacle_distance_m: float,
    joint_covariance: Matrix3,
    *,
    delta_max: float = 0.05,
    alpha: float = 0.05,
    beta: float = 25.0,
    minimum_distance_m: float = 1e-4,
) -> float:
    """Compute delta(t); larger clearance permits more risk, noise permits less."""
    if obstacle_distance_m < 0.0:
        raise ValueError("obstacle distance cannot be negative")
    if not 0.0 < delta_max <= 1.0:
        raise ValueError("delta_max must be in (0, 1]")
    distance = max(obstacle_distance_m, minimum_distance_m)
    return delta_max * exp(-alpha / distance) * exp(-beta * covariance_trace(joint_covariance))


class RiskCertifier:
    """Build continuous direction-independent complete-link certificates.

    Clearance is eroded by a nonlinear between-sample motion bound. Weighted
    Gaussian joint-error tails then union-bound collision over the swept path.
    Certification-only subdivision tightens the bound without changing motion.
    """

    def __init__(
        self,
        arm: PlanarArm3DOF,
        variance_floor: float = 1e-12,
        sigma_multiplier: float = 1.10,
        sweep_subdivisions: int = 4,
    ) -> None:
        if variance_floor <= 0.0:
            raise ValueError("variance_floor must be positive")
        if sigma_multiplier < 1.0:
            raise ValueError("sigma_multiplier must be at least one")
        if sweep_subdivisions < 1:
            raise ValueError("sweep_subdivisions must be at least one")
        self.arm = arm
        self.variance_floor = variance_floor
        self.sigma_multiplier = sigma_multiplier

        self.sweep_subdivisions = sweep_subdivisions

    def primitive_risk(
        self,
        primitive: MotionPrimitive,
        clearance_m: float,
        joint_covariance: Matrix3,
    ) -> float:
        validate_joint_covariance(joint_covariance)
        if clearance_m <= 0.0:
            return 1.0
        worst_risk = 0.0
        for joint_angles in primitive.positions:
            configuration_risk = 0.0
            for link_index in range(3):
                cartesian_covariance = self.arm.endpoint_covariance(
                    joint_angles, link_index, joint_covariance
                )
                sigma = self.sigma_multiplier * sqrt(max(self.variance_floor, _largest_eigenvalue_2x2(cartesian_covariance)))
                configuration_risk += _normal_cdf(-clearance_m / sigma)
            worst_risk = max(worst_risk, min(1.0, configuration_risk))
        return min(1.0, max(0.0, worst_risk))

    def sampled_trajectory_risk(
        self,
        primitive: MotionPrimitive,
        clearance_m: float,
        joint_covariance: Matrix3,
    ) -> float:
        """Union-bound collision across links and sampled trajectory states."""
        validate_joint_covariance(joint_covariance)
        if clearance_m <= 0.0:
            return 1.0
        trajectory_risk = 0.0
        for joint_angles in primitive.positions:
            configuration_risk = 0.0
            for link_index in range(3):
                cartesian_covariance = self.arm.endpoint_covariance(
                    joint_angles, link_index, joint_covariance
                )
                sigma = self.sigma_multiplier * sqrt(
                    max(self.variance_floor, _largest_eigenvalue_2x2(cartesian_covariance))
                )
                configuration_risk += _normal_cdf(-clearance_m / sigma)
            trajectory_risk += min(1.0, configuration_risk)
        return min(1.0, trajectory_risk)

    def continuous_swept_risk(
        self,
        primitive: MotionPrimitive,
        clearance_m: float,
        joint_covariance: Matrix3,
    ) -> float:
        """Bound collision anywhere on the continuously interpolated primitive.

        Joint error is modeled as one Gaussian offset that is quasi-static over
        the short primitive. The bound does not cover continuously resampled
        white noise, moving obstacles, or tracking dynamics. Its geometric
        displacement calculation is nonlinear rather than a Jacobian
        linearization.
        """
        validate_joint_covariance(joint_covariance)
        if clearance_m <= 0.0:
            return 1.0

        certification_primitive = subdivide_primitive(
            primitive, self.sweep_subdivisions
        )
        margins = primitive_link_sweep_margins(self.arm, certification_primitive)
        total_risk = 0.0
        for link_index in range(3):
            continuous_clearance = clearance_m - margins[link_index]
            if continuous_clearance <= 0.0:
                return 1.0

            coefficients = link_reach_coefficients(self.arm, link_index)
            weighted_sigmas = tuple(
                coefficients[joint]
                * self.sigma_multiplier
                * sqrt(max(0.0, joint_covariance[joint][joint]))
                for joint in range(3)
            )
            active = sum(value > 0.0 for value in weighted_sigmas)
            scale = sum(weighted_sigmas)
            if scale == 0.0:
                continue

            # Proportional allocation gives every active joint the same
            # standardized threshold. Boole's inequality permits correlation.
            standardized_clearance = continuous_clearance / scale
            link_risk = 2.0 * active * _normal_cdf(-standardized_clearance)
            total_risk += min(1.0, link_risk)

        return min(1.0, total_risk)

    def build_library(
        self,
        primitives: Tuple[MotionPrimitive, ...],
        distance_bins_m: Tuple[float, ...],
        joint_covariance: Matrix3,
    ) -> CertificateLibrary:
        risks = tuple(
            tuple(
                self.continuous_swept_risk(primitive, clearance, joint_covariance)
                for clearance in distance_bins_m
            )
            for primitive in primitives
        )
        return CertificateLibrary(distance_bins_m, primitives, risks)


class RollingCovarianceEstimator:
    """Estimate a full 3x3 sample covariance from a bounded encoder window."""

    def __init__(self, window_size: int = 50, variance_floor: float = 1e-12) -> None:
        if window_size < 2:
            raise ValueError("window_size must be at least two")
        self._samples: Deque[Vector3] = deque(maxlen=window_size)
        self.variance_floor = variance_floor

    def add(self, sample: Vector3) -> None:
        self._samples.append(sample)

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def covariance(self) -> Matrix3:
        if len(self._samples) < 2:
            floor = self.variance_floor
            return ((floor, 0.0, 0.0), (0.0, floor, 0.0), (0.0, 0.0, floor))
        means = tuple(sum(sample[i] for sample in self._samples) / len(self._samples) for i in range(3))
        denominator = len(self._samples) - 1
        rows = []
        for i in range(3):
            row = []
            for j in range(3):
                value = sum(
                    (sample[i] - means[i]) * (sample[j] - means[j])
                    for sample in self._samples
                ) / denominator
                if i == j:
                    value = max(self.variance_floor, value)
                row.append(value)
            rows.append(tuple(row))
        return tuple(rows)  # type: ignore[return-value]


class DirectionalRiskCertifier:
    """Geometry-aware first-order risk estimate for circular obstacles.

    The per-configuration estimate uses the union bound across all three link
    capsules. A primitive's value is the worst sampled configuration. This is
    the quantity calibrated by the Monte Carlo experiment module.
    """

    def __init__(
        self,
        geometry: CapsuleArmGeometry,
        variance_floor: float = 1e-12,
        sigma_multiplier: float = 1.0,
    ) -> None:
        if variance_floor <= 0.0:
            raise ValueError("variance_floor must be positive")
        if sigma_multiplier < 1.0:
            raise ValueError("sigma_multiplier must be at least one")
        self.geometry = geometry
        self.variance_floor = variance_floor
        self.sigma_multiplier = sigma_multiplier

    @staticmethod
    def _clearance_variance(
        gradient: Vector3, joint_covariance: Matrix3
    ) -> float:
        return max(
            0.0,
            sum(
                gradient[i] * joint_covariance[i][j] * gradient[j]
                for i in range(3)
                for j in range(3)
            ),
        )

    def configuration_risk(
        self,
        joint_angles: Vector3,
        obstacle: CircleObstacle,
        joint_covariance: Matrix3,
    ) -> float:
        validate_joint_covariance(joint_covariance)
        probability_sum = 0.0
        for query in self.geometry.link_clearances(joint_angles, obstacle):
            if query.clearance_m <= 0.0:
                return 1.0
            jacobian = self.geometry.arm.point_jacobian(
                joint_angles, query.link_index, query.segment_fraction
            )
            gradient = tuple(
                query.normal[0] * jacobian[0][joint]
                + query.normal[1] * jacobian[1][joint]
                for joint in range(3)
            )
            variance = max(
                self.variance_floor,
                self._clearance_variance(gradient, joint_covariance),
            )
            sigma = self.sigma_multiplier * sqrt(variance)
            probability_sum += _normal_cdf(-query.clearance_m / sigma)
        return min(1.0, probability_sum)

    def primitive_risk(
        self,
        primitive: MotionPrimitive,
        obstacle: CircleObstacle,
        joint_covariance: Matrix3,
    ) -> float:
        return max(
            self.configuration_risk(joint_angles, obstacle, joint_covariance)
            for joint_angles in primitive.positions
        )
