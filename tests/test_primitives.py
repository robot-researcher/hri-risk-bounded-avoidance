import unittest

from risk_bounded_collision.primitives import (
    generate_primitives,
    quintic_primitive,
    validate_sampled_dynamics,
)


class PrimitiveTests(unittest.TestCase):
    def test_quintic_hits_endpoints(self) -> None:
        primitive = quintic_primitive("p", (0.0, 0.0, 0.0), (0.2, -0.1, 0.3))
        self.assertEqual(primitive.start, (0.0, 0.0, 0.0))
        self.assertEqual(primitive.terminal, (0.2, -0.1, 0.3))

    def test_generation_is_reproducible_and_respects_limits(self) -> None:
        first = generate_primitives((0.0, 0.0, 0.0), count=12, seed=11)
        second = generate_primitives((0.0, 0.0, 0.0), count=12, seed=11)
        self.assertEqual(first, second)
        self.assertEqual(len(first), 12)
        for primitive in first:
            self.assertTrue(all(-1.5708 <= angle <= 1.5708 for angle in primitive.terminal))

    def test_sampled_dynamics_accepts_slow_primitive(self) -> None:
        primitive = quintic_primitive(
            "slow", (0.0, 0.0, 0.0), (0.1, -0.1, 0.05), duration_s=1.0
        )
        result = validate_sampled_dynamics(
            primitive,
            velocity_limits_rad_s=(1.0, 1.0, 1.0),
            acceleration_limits_rad_s2=(2.0, 2.0, 2.0),
        )
        self.assertTrue(result.valid)

    def test_sampled_dynamics_rejects_aggressive_primitive(self) -> None:
        primitive = quintic_primitive(
            "fast", (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), duration_s=0.1
        )
        result = validate_sampled_dynamics(
            primitive,
            velocity_limits_rad_s=(1.0, 1.0, 1.0),
            acceleration_limits_rad_s2=(5.0, 5.0, 5.0),
        )
        self.assertFalse(result.valid)
        self.assertGreater(result.maximum_velocity_rad_s[0], 1.0)

    def test_sampled_dynamics_rejects_nonpositive_limits(self) -> None:
        primitive = quintic_primitive(
            "invalid-limit", (0.0, 0.0, 0.0), (0.1, 0.0, 0.0)
        )
        with self.assertRaises(ValueError):
            validate_sampled_dynamics(
                primitive,
                velocity_limits_rad_s=(0.0, 1.0, 1.0),
                acceleration_limits_rad_s2=(1.0, 1.0, 1.0),
            )


if __name__ == "__main__":
    unittest.main()
