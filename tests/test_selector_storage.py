import tempfile
import unittest
from pathlib import Path

from risk_bounded_collision.models import CertificateLibrary
from risk_bounded_collision.primitives import quintic_primitive
from risk_bounded_collision.selector import PrimitiveSelector
from risk_bounded_collision.storage import load_library, save_library


class SelectorAndStorageTests(unittest.TestCase):
    def setUp(self) -> None:
        self.primitives = (
            quintic_primitive("toward-goal", (0.0, 0.0, 0.0), (0.2, 0.0, 0.0)),
            quintic_primitive("away", (0.0, 0.0, 0.0), (-0.2, 0.0, 0.0)),
        )
        self.library = CertificateLibrary(
            (0.10, 0.20),
            self.primitives,
            ((0.03, 0.01), (0.02, 0.005)),
        )

    def test_selector_uses_conservative_bin_and_goal_cost(self) -> None:
        result = PrimitiveSelector(self.library).select(
            obstacle_distance_m=0.25,
            threshold=0.02,
            goal_joint_angles=(0.25, 0.0, 0.0),
        )
        self.assertFalse(result.fallback)
        self.assertEqual(result.distance_bin_m, 0.20)
        self.assertEqual(result.primitive_id, "toward-goal")

    def test_selector_holds_when_nothing_is_certified(self) -> None:
        result = PrimitiveSelector(self.library).select(
            obstacle_distance_m=0.10,
            threshold=0.001,
            goal_joint_angles=(0.25, 0.0, 0.0),
        )
        self.assertTrue(result.fallback)
        self.assertIsNone(result.primitive_id)

    def test_json_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "library.json"
            save_library(self.library, path)
            restored = load_library(path)
        self.assertEqual(restored, self.library)


if __name__ == "__main__":
    unittest.main()
