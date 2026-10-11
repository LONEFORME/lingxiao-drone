import unittest

from Mission_GPT import mission


class HorizontalVelocityProviderTest(unittest.TestCase):
    def _mission(self, provider=None):
        return mission([0] * 14, [0] * 11, horizontal_velocity_provider=provider)

    def test_no_provider_preserves_waypoint_velocity(self):
        task = self._mission()
        result = task._select_horizontal_velocity(
            12, -7, (0.0, 0.0, 1.5), (0.0, 0.0, 0.0), 3, (0.0, 0.0, 1.5)
        )
        self.assertEqual(result, (12, -7))
        self.assertEqual(task._horizontal_control_source, "t265_waypoint")

    def test_active_provider_is_the_only_selected_source(self):
        task = self._mission(lambda context: {
            "active": True,
            "vx_cms": 5.4,
            "vy_cms": -2.6,
            "source": "vision_xy",
            "reason": "full_track",
        })
        result = task._select_horizontal_velocity(
            12, -7, (0.0, 0.0, 1.5), (0.0, 0.0, 0.0), 3, (0.0, 0.0, 1.5)
        )
        self.assertEqual(result, (5, -3))
        self.assertEqual(task._horizontal_control_source, "vision_xy")

    def test_provider_exception_zeros_current_tick_and_detaches(self):
        def failed(_context):
            raise RuntimeError("boom")

        task = self._mission(failed)
        result = task._select_horizontal_velocity(
            12, -7, (0.0, 0.0, 1.5), (0.0, 0.0, 0.0), 3, (0.0, 0.0, 1.5)
        )
        self.assertEqual(result, (0, 0))
        self.assertIsNone(task.horizontal_velocity_provider)
        self.assertTrue(task._horizontal_provider_fault_latched)

    def test_nonfinite_or_over_limit_provider_is_rejected(self):
        for vx, vy in ((float("nan"), 0.0), (41.0, 0.0)):
            task = self._mission(lambda _context, x=vx, y=vy: {
                "active": True, "vx_cms": x, "vy_cms": y,
            })
            self.assertEqual(task._select_horizontal_velocity(
                1, 2, (0.0, 0.0, 1.5), (0.0, 0.0, 0.0), 3,
                (0.0, 0.0, 1.5),
            ), (0, 0))

    def test_zero_dwell_final_waypoint_advances_to_descend(self):
        task = self._mission()
        task.targets = [[0.0, 0.0, 1.5], [0.0, 0.0, 0.15]]
        task.target_index = 1
        self.assertEqual(task._waypoint_hold_s(), 0.0)
        task._advance_waypoint(
            "precision_arrival", [0.0, 0.0, 0.15], task.targets[1], 0.0
        )
        self.assertEqual(task.target_index, 2)
        task.navigate([0.0, 0.0, 0.15], 0.0)
        self.assertEqual(task.state, "DESCEND")


if __name__ == "__main__":
    unittest.main()
