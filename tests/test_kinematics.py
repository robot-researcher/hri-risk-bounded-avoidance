import math
import unittest

from risk_bounded_collision.kinematics import PlanarArm3DOF


class PlanarArmTests(unittest.TestCase):
    def test_straight_arm_endpoint(self) -> None:
        arm = PlanarArm3DOF((0.30, 0.25, 0.15))
        points = arm.joint_positions((0.0, 0.0, 0.0))
        self.assertAlmostEqual(points[-1][0], 0.70)
        self.assertAlmostEqual(points[-1][1], 0.0)

    def test_first_joint_rotates_full_arm(self) -> None:
        arm = PlanarArm3DOF((0.30, 0.25, 0.15))
        points = arm.joint_positions((math.pi / 2.0, 0.0, 0.0))
        self.assertAlmostEqual(points[-1][0], 0.0, places=12)
        self.assertAlmostEqual(points[-1][1], 0.70)

    def test_endpoint_jacobian_at_zero(self) -> None:
        jacobian = PlanarArm3DOF().endpoint_jacobian((0.0, 0.0, 0.0), 2)
        self.assertEqual(jacobian[0], (0.0, 0.0, 0.0))
        for actual, expected in zip(jacobian[1], (0.70, 0.40, 0.15)):
            self.assertAlmostEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
