import math
import unittest

from .static_square_servo import StaticSquareServo, StaticSquareServoConfig
from .vision.platform_observation import FeatureFlag, PlatformObservation


class FakeReader:
    def __init__(self):
        self.observation = None
        self.running = True

    def latest(self, now, max_age_s):
        if self.observation is None or self.observation.age_s(now) > max_age_s:
            return None
        return self.observation

    def is_running(self):
        return self.running


def context(now, x=0.0, y=0.0, z=1.5, confidence=3):
    return {
        "now_monotonic": now,
        "position_m": (x, y, z),
        "velocity_m_s": (0.0, 0.0, 0.0),
        "t265_confidence": confidence,
    }


def blue_config(**kwargs):
    return StaticSquareServoConfig(vision_target_source="blue_square", **kwargs)


class StaticSquareServoSafetyTest(unittest.TestCase):
    def test_static_servo_forces_visual_velocity_feedforward_off(self):
        servo = StaticSquareServo(
            FakeReader(),
            StaticSquareServoConfig(target_velocity_feedforward_gain=0.6),
        )
        self.assertEqual(servo.controller.config.target_velocity_feedforward_gain, 0.0)

    def test_confirmed_left_target_commands_positive_x_then_stale_zero(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader, blue_config())
        servo.arm(0.0, (0.0, 0.0))
        for seq, now in enumerate((0.03, 0.06, 0.09), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 220, 240, 80, 0, 0, 90,
                int(FeatureFlag.SURROGATE_SQUARE), now,
            )
            servo(context(now))
        self.assertGreater(servo.snapshot().command_m_s[0], 0.0)
        self.assertAlmostEqual(servo.snapshot().command_m_s[1], 0.0, places=6)
        decision = servo(context(0.30))
        self.assertEqual((decision["vx_cms"], decision["vy_cms"]), (0, 0))
        self.assertFalse(decision["active"])
        self.assertEqual(servo.snapshot().mode, "LOST")

    def test_center_observation_brakes_without_visual_velocity_feedforward(self):
        reader = FakeReader()
        servo = StaticSquareServo(
            reader,
            StaticSquareServoConfig(
                confirm_frames=3,
                full_max_speed_m_s=0.08,
                max_accel_m_s2=0.10,
                max_jerk_m_s3=0.8,
            ),
        )
        servo.arm(0.0, (0.0, 0.0))

        # 先让静态目标保持在画面左侧，使速度整形器达到限速。
        for seq in range(1, 21):
            now = seq * 0.1
            reader.observation = PlatformObservation(
                1, seq, seq, True, 120, 240, 80, 0, 0, 90,
                int(FeatureFlag.APRILTAG_VALID), now,
            )
            servo(context(now))
            self.assertLessEqual(
                math.hypot(*servo.snapshot().command_m_s), 0.08 + 1e-9
            )
        self.assertGreater(servo.snapshot().command_m_s[0], 0.07)

        # 目标到达画面中心后只应制动，不能保留旧的“目标运动速度”。
        braking_speeds = []
        for seq in range(21, 36):
            now = seq * 0.1
            reader.observation = PlatformObservation(
                1, seq, seq, True, 320, 240, 80, 0, 0, 90,
                int(FeatureFlag.APRILTAG_VALID), now,
            )
            servo(context(now))
            braking_speeds.append(servo.snapshot().command_m_s[0])
        self.assertGreaterEqual(min(braking_speeds), -1e-8)
        self.assertAlmostEqual(braking_speeds[-1], 0.0, places=6)

    def test_t265_velocity_damping_reduces_command_in_same_direction(self):
        def command_for_velocity(vx):
            reader = FakeReader()
            servo = StaticSquareServo(
                reader,
                StaticSquareServoConfig(
                    confirm_frames=1,
                    kp=0.35,
                    kd=0.50,
                    max_accel_m_s2=10.0,
                    max_jerk_m_s3=100.0,
                ),
            )
            servo.arm(0.0, (0.0, 0.0))
            reader.observation = PlatformObservation(
                1, 1, 1, True, 290, 240, 80, 0, 0, 90,
                int(FeatureFlag.APRILTAG_VALID), 0.1,
            )
            ctx = context(0.1)
            ctx["velocity_m_s"] = (vx, 0.0, 0.0)
            servo(ctx)
            return servo.snapshot().command_m_s[0]

        stationary_command = command_for_velocity(0.0)
        moving_toward_target_command = command_for_velocity(0.05)
        self.assertGreater(stationary_command, 0.0)
        self.assertLess(moving_toward_target_command, stationary_command)

    def test_center_deadband_keeps_damping_until_t265_velocity_is_stable(self):
        reader = FakeReader()
        servo = StaticSquareServo(
            reader,
            StaticSquareServoConfig(
                confirm_frames=1,
                kd=0.50,
                velocity_deadband_m_s=0.02,
                max_accel_m_s2=10.0,
                max_jerk_m_s3=100.0,
            ),
        )
        servo.arm(0.0, (0.0, 0.0))
        reader.observation = PlatformObservation(
            1, 1, 1, True, 320, 240, 80, 0, 0, 90,
            int(FeatureFlag.APRILTAG_VALID), 0.1,
        )
        moving = context(0.1)
        moving["velocity_m_s"] = (0.05, 0.0, 0.0)
        servo(moving)
        self.assertLess(servo.snapshot().command_m_s[0], 0.0)
        self.assertNotEqual(servo.snapshot().mode, "CENTERED")

        reader.observation = PlatformObservation(
            1, 2, 2, True, 320, 240, 80, 0, 0, 90,
            int(FeatureFlag.APRILTAG_VALID), 0.2,
        )
        servo(context(0.2))
        self.assertEqual(servo.snapshot().mode, "CENTERED")

    def test_platform_velocity_feedforward_tracks_positive_y(self):
        reader = FakeReader()
        servo = StaticSquareServo(
            reader,
            StaticSquareServoConfig(
                confirm_frames=1,
                platform_velocity_m_s=(0.0, 0.10),
                full_max_speed_m_s=0.15,
                max_accel_m_s2=10.0,
                max_jerk_m_s3=100.0,
            ),
        )
        servo.arm(0.0, (0.0, 0.0))
        reader.observation = PlatformObservation(
            1, 1, 1, True, 320, 240, 80, 0, 0, 90,
            int(FeatureFlag.APRILTAG_VALID), 0.1,
        )
        matched = context(0.1)
        matched["velocity_m_s"] = (0.0, 0.10, 0.0)
        servo(matched)
        command_x, command_y = servo.snapshot().command_m_s
        self.assertAlmostEqual(command_x, 0.0, places=9)
        self.assertAlmostEqual(command_y, 0.10, places=9)
        self.assertEqual(servo.snapshot().mode, "CENTERED")

        reader.observation = PlatformObservation(
            1, 2, 2, True, 320, 240, 80, 0, 0, 90,
            int(FeatureFlag.APRILTAG_VALID), 0.2,
        )
        stationary = context(0.2)
        servo(stationary)
        self.assertGreater(servo.snapshot().command_m_s[1], 0.10)
        self.assertLessEqual(servo.snapshot().command_m_s[1], 0.15)

    def test_reacquires_new_target_after_stale_tracker_reset(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader)
        servo.arm(0.0, (0.0, 0.0))
        for seq, now in enumerate((0.03, 0.06, 0.09), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 220, 240, 70, 0, 0, 90,
                int(FeatureFlag.APRILTAG_VALID), now,
            )
            servo(context(now))
        self.assertEqual(servo.snapshot().mode, "FULL_TRACK")

        lost = servo(context(0.31))
        self.assertFalse(lost["active"])
        for seq, now in enumerate((0.34, 0.37, 0.40), 4):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 420, 240, 70, 0, 0, 90,
                int(FeatureFlag.APRILTAG_VALID), now,
            )
            decision = servo(context(now))
        self.assertTrue(decision["active"])
        self.assertEqual(servo.snapshot().mode, "FULL_TRACK")

    def test_partial_target_uses_coarse_speed_limit(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader, blue_config())
        servo.arm(1.0, (0.0, 0.0))
        reader.observation = PlatformObservation(
            1, 1, 1, True, 100, 240, 0, 0, 0, 70,
            int(FeatureFlag.SURROGATE_SQUARE | FeatureFlag.PARTIAL), 1.03,
        )
        decision = servo(context(1.03))
        self.assertEqual(servo.snapshot().mode, "PARTIAL_COARSE")
        self.assertGreaterEqual(decision["vx_cms"], 0)
        self.assertLessEqual((decision["vx_cms"] ** 2 + decision["vy_cms"] ** 2) ** 0.5, 5.0)

    def test_partial_vertical_direction_matches_flight_calibration(self):
        for cy, expected_sign in ((140, -1), (340, 1)):
            reader = FakeReader()
            servo = StaticSquareServo(reader, blue_config())
            servo.arm(1.0, (0.0, 0.0))
            reader.observation = PlatformObservation(
                1, 1, 1, True, 320, cy, 0, 0, 0, 70,
                int(FeatureFlag.SURROGATE_SQUARE | FeatureFlag.PARTIAL), 1.03,
            )
            servo(context(1.03))
            self.assertGreater(expected_sign * servo.snapshot().command_m_s[1], 0.0)
            self.assertAlmostEqual(servo.snapshot().command_m_s[0], 0.0, places=9)

    def test_full_target_above_commands_negative_y_and_right_commands_negative_x(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader, blue_config())
        servo.arm(2.0, (0.0, 0.0))
        for seq, now in enumerate((2.03, 2.06, 2.09), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 420, 140, 90, 0, 0, 90,
                int(FeatureFlag.SURROGATE_SQUARE), now,
            )
            servo(context(now))
        command_x, command_y = servo.snapshot().command_m_s
        self.assertLess(command_x, 0.0)
        self.assertLess(command_y, 0.0)

    def test_hard_geofence_fault_is_latched(self):
        servo = StaticSquareServo(FakeReader())
        servo.arm(1.0, (0.0, 0.0))
        decision = servo(context(1.03, x=0.61))
        self.assertTrue(decision["fault"])
        self.assertEqual(decision["reason"], "hard_geofence")
        with self.assertRaises(RuntimeError):
            servo.arm(2.0, (0.0, 0.0))

    def test_soft_geofence_removes_outward_component(self):
        servo = StaticSquareServo(FakeReader())
        servo.arm(1.0, (0.0, 0.0))
        command = servo._apply_soft_geofence((0.08, 0.02), (0.55, 0.0, 1.5))
        self.assertAlmostEqual(command[0], 0.0, places=9)
        self.assertAlmostEqual(command[1], 0.02, places=9)

    def test_timeout_and_height_or_t265_faults(self):
        servo = StaticSquareServo(FakeReader(), blue_config(max_duration_s=0.1))
        servo.arm(1.0, (0.0, 0.0))
        decision = servo(context(1.11))
        self.assertFalse(decision["fault"])
        self.assertEqual(servo.snapshot().mode, "EXITING")

        for kwargs, reason in (
            ({"z": 1.7}, "height_out_of_range"),
            ({"confidence": 0}, "t265_confidence_zero"),
        ):
            guarded = StaticSquareServo(FakeReader())
            guarded.arm(2.0, (0.0, 0.0))
            fault = guarded(context(2.03, **kwargs))
            self.assertTrue(fault["fault"])
            self.assertEqual(fault["reason"], reason)

    def test_reader_thread_stop_faults_same_tick(self):
        reader = FakeReader()
        reader.running = False
        servo = StaticSquareServo(reader)
        servo.arm(1.0, (0.0, 0.0))
        decision = servo(context(1.03))
        self.assertTrue(decision["fault"])
        self.assertEqual(decision["reason"], "cybercam_reader_stopped")

    def test_apriltag_requires_three_frames_and_rejects_surrogate(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader)
        servo.arm(3.0, (0.0, 0.0))
        for seq, now in enumerate((3.03, 3.06), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 220, 240, 70, 0, 0, 90,
                int(FeatureFlag.APRILTAG_VALID), now,
            )
            servo(context(now))
        self.assertEqual(servo.snapshot().command_cm_s, (0, 0))
        reader.observation = PlatformObservation(
            1, 3, 3, True, 220, 240, 70, 0, 0, 90,
            int(FeatureFlag.APRILTAG_VALID), 3.09,
        )
        servo(context(3.09))
        self.assertGreater(servo.snapshot().command_m_s[0], 0.0)
        stale = servo(context(3.30))
        self.assertEqual((stale["vx_cms"], stale["vy_cms"]), (0, 0))
        self.assertEqual(servo.snapshot().mode, "LOST")

        rejected = StaticSquareServo(FakeReader())
        rejected.arm(4.0, (0.0, 0.0))
        for seq, now in enumerate((4.03, 4.06, 4.09), 1):
            rejected.reader.observation = PlatformObservation(
                1, seq, seq, True, 220, 240, 70, 0, 0, 90,
                int(FeatureFlag.SURROGATE_SQUARE), now,
            )
            rejected(context(now))
        self.assertEqual(rejected.snapshot().command_cm_s, (0, 0))
        self.assertEqual(rejected.snapshot().mode, "LOST")

    def test_mixed_or_partial_apriltag_flags_are_rejected(self):
        for flags in (
            FeatureFlag.APRILTAG_VALID | FeatureFlag.SURROGATE_SQUARE,
            FeatureFlag.APRILTAG_VALID | FeatureFlag.PARTIAL,
        ):
            reader = FakeReader()
            servo = StaticSquareServo(reader)
            servo.arm(5.0, (0.0, 0.0))
            reader.observation = PlatformObservation(
                1, 1, 1, True, 220, 240, 70, 0, 0, 90, int(flags), 5.03,
            )
            servo(context(5.03))
            self.assertEqual(servo.snapshot().command_cm_s, (0, 0))
            self.assertFalse(servo.observation_usable_for_preflight(
                reader.observation, 5.03
            ))

    def test_temporal_apriltag_is_accepted_with_coarse_speed_limit(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader)
        servo.arm(6.0, (0.0, 0.0))
        for seq, now in enumerate((6.03, 6.06, 6.09), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 100, 100, 80, 0, 0, 80,
                int(FeatureFlag.APRILTAG_VALID | FeatureFlag.TEMPORAL_TRACKED),
                now,
            )
            decision = servo(context(now))
        self.assertTrue(decision["active"])
        self.assertEqual(servo.snapshot().mode, "TEMPORAL_TRACK")
        self.assertLessEqual(
            math.hypot(*servo.snapshot().command_m_s),
            servo.config.partial_max_speed_m_s + 1e-9,
        )

    def test_color_fusion_is_accepted_and_uses_coarse_speed_limit(self):
        reader = FakeReader()
        servo = StaticSquareServo(reader)
        servo.arm(7.0, (0.0, 0.0))
        for seq, now in enumerate((7.03, 7.06, 7.09), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 100, 100, 250, 70, 0, 80,
                int(FeatureFlag.APRILTAG_VALID | FeatureFlag.COLOR_SHAPE_TRACKED),
                now,
            )
            decision = servo(context(now))
        self.assertTrue(decision["active"])
        self.assertEqual(servo.snapshot().mode, "COLOR_TRACK")
        self.assertLessEqual(
            math.hypot(*servo.snapshot().command_m_s),
            servo.config.partial_max_speed_m_s + 1e-9,
        )

    def test_centered_completion_can_be_deferred_during_descent(self):
        reader = FakeReader()
        servo = StaticSquareServo(
            reader,
            StaticSquareServoConfig(confirm_frames=1, centered_hold_s=0.1),
        )
        servo.arm(8.0, (0.0, 0.0))
        servo.set_completion_enabled(False)
        for seq, now in enumerate((8.03, 8.16, 8.30), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 320, 240, 80, 0, 0, 90,
                int(FeatureFlag.APRILTAG_VALID), now,
            )
            servo(context(now))
        self.assertTrue(servo.active)
        self.assertEqual(servo.snapshot().mode, "CENTERED")

        servo.set_completion_enabled(True)
        reader.observation = PlatformObservation(
            1, 4, 4, True, 320, 240, 80, 0, 0, 90,
            int(FeatureFlag.APRILTAG_VALID), 8.33,
        )
        servo(context(8.33))
        servo(context(8.36))
        self.assertEqual(servo.snapshot().mode, "EXITING")


if __name__ == "__main__":
    unittest.main()
