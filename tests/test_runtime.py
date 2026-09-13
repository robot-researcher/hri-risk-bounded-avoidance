import unittest
from math import pi

from risk_bounded_collision.kinematics import PlanarArm3DOF
from risk_bounded_collision.primitives import generate_primitives
from risk_bounded_collision.risk import RiskCertifier
from risk_bounded_collision.runtime import RuntimeObservation, SafetyRuntime, SimulatedRobotAdapter
from risk_bounded_collision.selector import PrimitiveSelector


def covariance(stddev_degrees):
    variance = (stddev_degrees * pi / 180.0) ** 2
    return ((variance, 0.0, 0.0), (0.0, variance, 0.0), (0.0, 0.0, variance))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        primitives = generate_primitives((0.0, 0.0, 0.0), count=16)
        bins = tuple(0.05 * index for index in range(1, 11))
        library = RiskCertifier(PlanarArm3DOF()).build_library(primitives, bins, covariance(1.0))
        self.adapter = SimulatedRobotAdapter()
        self.runtime = SafetyRuntime(PrimitiveSelector(library), self.adapter)

    def test_stale_perception_holds(self):
        decision = self.runtime.step(
            RuntimeObservation((0.0, 0.0, 0.0), covariance(1.0), 0.30, 1.0),
            (0.2, -0.1, 0.1),
        )
        self.assertTrue(decision.selection.fallback)
        self.assertEqual("HOLD", decision.command)
        self.assertIn("stale", decision.selection.reason)

    def test_clear_workspace_selects_motion(self):
        decision = self.runtime.step(
            RuntimeObservation((0.0, 0.0, 0.0), covariance(1.0), 0.30, 0.01),
            (0.2, -0.1, 0.1),
        )
        self.assertFalse(decision.selection.fallback)
        self.assertTrue(decision.command.startswith("EXECUTE"))

    def test_wrong_primitive_start_holds(self):
        decision = self.runtime.step(
            RuntimeObservation((0.2, 0.0, 0.0), covariance(1.0), 0.30, 0.01),
            (0.2, -0.1, 0.1),
        )
        self.assertTrue(decision.selection.fallback)
        self.assertIn("start tolerance", decision.selection.reason)


if __name__ == "__main__":
    unittest.main()
