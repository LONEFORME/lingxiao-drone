import unittest

import cv2
import numpy as np

from CyberCamera.boards.cybercam_d.detector import (
    AprilTagDetector,
    BlueSquareDetector,
    FeatureFlag,
    PlatformDetection,
    PlatformDetector,
)
from CyberCamera.boards.cybercam_d.protocol import encode
from CyberCamera.boards.cybercam_d.camera_backend import WalnutPiCSICapture
from drone_control.competition_2026_d.control.formation_controller import (
    FormationConfig,
    FormationController,
)
from drone_control.competition_2026_d.vision.camera_model import CameraIntrinsics, DownwardCameraModel
from drone_control.competition_2026_d.vision.cybercam_protocol import ObservationGate, parse_line
from drone_control.competition_2026_d.vision.platform_tracker import PlatformTracker
from drone_control.competition_2026_d.vision.platform_observation import PlatformObservation
from drone_control.competition_2026_d.vision.platform_tracker import PlatformEstimate
from drone_control.competition_2026_d.static_square_servo import (
    StaticSquareServo,
    StaticSquareServoConfig,
)


def april_tag_canvas(markers, shape=(300, 400)):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    image = np.full(shape, 255, np.uint8)
    for tag_id, x, y, size in markers:
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, size)
        image[y:y + size, x:x + size] = marker
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


class VisionCoreTest(unittest.TestCase):
    def test_external_csi_adapter_uses_walnutpi_sensor_contract(self):
        calls = []

        class FakeSensor:
            def __init__(self, width, height):
                calls.append((width, height))
                self.released = False
                self.hmirror = 0

            def isOpened(self):
                return True

            def set_hmirror(self, value):
                self.hmirror = value

            def set_vflip(self, _value):
                pass

            def read(self):
                return True, np.zeros((240, 320, 3), np.uint8)

            def release(self):
                self.released = True

        capture = WalnutPiCSICapture(320, 240, hmirror=True, sensor_factory=FakeSensor)
        self.assertEqual(calls, [(320, 240)])
        self.assertTrue(capture.isOpened())
        ok, frame = capture.read()
        self.assertTrue(ok)
        self.assertEqual(frame.shape, (240, 320, 3))
        capture.release()
        self.assertTrue(capture._sensor.released)

    def test_vs1_roundtrip_and_stream_resync(self):
        line = encode(7, 10, 1000, True, 321, 242, 150, 90, 125, 88, 7)
        obs = parse_line(line, received_monotonic=2.0)
        self.assertEqual((obs.stream_id, obs.seq, obs.angle_cdeg), (7, 10, 125))
        gate = ObservationGate(confirm_frames=3)
        self.assertIsNotNone(gate.accept(obs))
        self.assertIsNone(gate.accept(obs))
        for seq in (0, 1):
            new = parse_line(encode(8, seq, 2000 + seq, True, 10, 20, 0, 80, 0, 70, 14))
            self.assertIsNone(gate.accept(new))
        accepted = gate.accept(parse_line(encode(8, 2, 2002, True, 10, 20, 0, 80, 0, 70, 14)))
        self.assertIsNotNone(accepted)
        self.assertEqual(gate.active_stream, 8)

        tag_line = encode(
            9, 1, 3000, True, 320, 240, 46, 0, 9000, 80,
            int(FeatureFlag.APRILTAG_VALID),
        )
        tag_obs = parse_line(tag_line, received_monotonic=3.0)
        self.assertEqual(tag_obs.flags, int(FeatureFlag.APRILTAG_VALID))

    def test_concentric_target_detection(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        center = (350, 210)
        cv2.circle(image, center, 100, (0, 0, 0), 10)
        cv2.circle(image, center, 60, (0, 0, 0), 8)
        cv2.line(image, (center[0] - 35, center[1]), (center[0] + 35, center[1]), (0, 0, 0), 7)
        cv2.line(image, (center[0], center[1] - 35), (center[0], center[1] + 35), (0, 0, 0), 7)
        result = PlatformDetector().detect(image)
        self.assertTrue(result.found)
        self.assertLess(abs(result.cx - center[0]), 6)
        self.assertLess(abs(result.cy - center[1]), 6)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.INNER_VALID)

    def test_apriltag_id_center_geometry_and_color_cast(self):
        image = april_tag_canvas([(0, 140, 90, 100)])
        image[:, :, 0] = (image[:, :, 0] * 0.45).astype(np.uint8)
        image[:, :, 1] = (image[:, :, 1] * 0.75).astype(np.uint8)
        result = AprilTagDetector("tag36h11", 0).detect(image)
        self.assertTrue(result.found)
        self.assertLess(abs(result.cx - 190), 3)
        self.assertLess(abs(result.cy - 140), 3)
        self.assertGreater(result.outer_px, 90)
        self.assertGreaterEqual(result.quality, 55)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.APRILTAG_VALID)
        self.assertEqual(len(result.debug_polygon), 4)

    def test_apriltag_wrong_or_mixed_ids_are_ambiguous(self):
        detector = AprilTagDetector("tag36h11", 0)
        wrong = detector.detect(april_tag_canvas([(1, 140, 90, 100)]))
        self.assertFalse(wrong.found)
        self.assertTrue(FeatureFlag(wrong.flags) & FeatureFlag.AMBIGUOUS)
        mixed = detector.detect(april_tag_canvas([
            (0, 40, 100, 80), (1, 270, 100, 80),
        ]))
        self.assertFalse(mixed.found)
        self.assertTrue(FeatureFlag(mixed.flags) & FeatureFlag.AMBIGUOUS)

    def test_apriltag_too_small_is_rejected(self):
        image = april_tag_canvas([(0, 191, 141, 17)])
        self.assertFalse(AprilTagDetector("tag36h11", 0).detect(image).found)

    def test_apriltag_rotated_marker_remains_valid(self):
        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36H11
        )
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 100)
        image = np.full((300, 400), 255, np.uint8)
        image[90:190, 140:240] = cv2.rotate(marker, cv2.ROTATE_90_CLOCKWISE)
        result = AprilTagDetector("tag36h11", 0).detect(image)
        self.assertTrue(result.found)
        self.assertGreaterEqual(abs(result.angle_cdeg), 8900)

    def test_large_inner_circle_and_cross_support_partial_near_mode(self):
        detector = PlatformDetector()
        far = np.full((240, 320, 3), 255, np.uint8)
        cv2.circle(far, (160, 120), 90, (0, 0, 0), 8)
        cv2.circle(far, (160, 120), 54, (0, 0, 0), 7)
        cv2.line(far, (125, 120), (195, 120), (0, 0, 0), 7)
        cv2.line(far, (160, 85), (160, 155), (0, 0, 0), 7)
        self.assertTrue(detector.detect(far).found)
        image = np.full((240, 320, 3), 255, np.uint8)
        center = (160, 120)
        cv2.circle(image, center, 55, (0, 0, 0), 8)
        cv2.line(image, (120, 120), (200, 120), (0, 0, 0), 9)
        cv2.line(image, (160, 80), (160, 160), (0, 0, 0), 9)
        result = detector.detect(image)
        self.assertTrue(result.found)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.CROSS_VALID)

    def test_cross_only_cannot_acquire_without_prior_circle_track(self):
        image = np.full((240, 320, 3), 255, np.uint8)
        cv2.line(image, (100, 120), (220, 120), (0, 0, 0), 9)
        cv2.line(image, (160, 60), (160, 180), (0, 0, 0), 9)
        self.assertFalse(PlatformDetector().detect(image).found)

    def test_formal_detector_releases_stale_center_after_losses(self):
        detector = PlatformDetector(source_confirm_frames=1)
        first = PlatformDetection(True, 60, 60, 100, 60, 0, 90, 3)
        far = PlatformDetection(True, 250, 170, 100, 60, 0, 90, 3)
        self.assertTrue(detector._apply_temporal(first, "full", 320, 240).found)
        for _ in range(5):
            self.assertFalse(detector._apply_temporal(far, "full", 320, 240).found)
        self.assertTrue(detector._apply_temporal(far, "full", 320, 240).found)

    def test_blue_square_detection_and_surrogate_flag(self):
        image = np.full((240, 320, 3), (95, 120, 145), np.uint8)
        center = (190, 105)
        box = cv2.boxPoints((center, (82, 78), 7.0)).astype(np.int32)
        cv2.fillConvexPoly(image, box, (255, 120, 10))
        result = BlueSquareDetector().detect(image)
        self.assertTrue(result.found)
        self.assertLess(abs(result.cx - center[0]), 3)
        self.assertLess(abs(result.cy - center[1]), 3)
        self.assertGreater(result.outer_px, 70)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.SURROGATE_SQUARE)
        self.assertFalse(FeatureFlag(result.flags) & FeatureFlag.OUTER_VALID)
        self.assertEqual(len(result.debug_polygon), 4)

    def test_blue_line_and_rectangle_are_rejected(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.line(image, (10, 20), (300, 210), (255, 80, 0), 5)
        cv2.rectangle(image, (30, 170), (230, 205), (255, 80, 0), -1)
        self.assertFalse(BlueSquareDetector().detect(image).found)

    def test_partial_blue_square_at_edge_is_coarse_only(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (105, 185), (215, 239), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertTrue(result.found)
        flags = FeatureFlag(result.flags)
        self.assertTrue(flags & FeatureFlag.PARTIAL)
        self.assertTrue(flags & FeatureFlag.SURROGATE_SQUARE)
        self.assertEqual(result.outer_px, 0)

    def test_two_valid_blue_targets_are_ambiguous(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (30, 70), (100, 140), (255, 90, 0), -1)
        cv2.rectangle(image, (210, 70), (280, 140), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertFalse(result.found)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.AMBIGUOUS)

    def test_surrogate_observation_requires_explicit_permission(self):
        obs = PlatformObservation(
            1, 2, 3, True, 160, 120, 70, 0, 0, 90,
            int(FeatureFlag.SURROGATE_SQUARE), 10.0,
        )
        self.assertFalse(obs.usable(10.05, 0.15, 55))
        self.assertTrue(obs.usable(10.05, 0.15, 55, allow_surrogate=True))

    def test_static_profile_disables_target_velocity_and_uses_deadband(self):
        controller = FormationController(FormationConfig(
            kp=0.75,
            kd=0.22,
            max_speed_m_s=0.10,
            max_accel_m_s2=0.18,
            max_jerk_m_s3=0.60,
            position_deadband_m=0.05,
            target_velocity_feedforward_gain=0.0,
        ))
        controller.reset(0.0)
        estimate = PlatformEstimate(0.03, 0.0, 1.0, 0.0, 0.1, 0.02, False)
        command = controller.command(estimate, (0.0, 0.0), (0.0, 0.0), 0.1)
        self.assertTrue(command.valid)
        self.assertAlmostEqual(command.vx_m_s, 0.0)
        self.assertAlmostEqual(command.vy_m_s, 0.0)

    def test_camera_signs_tracker_and_controller_limits(self):
        model = DownwardCameraModel(CameraIntrinsics(500, 500, 320, 240))
        forward, right = model.body_relative_xy(370, 290, 1.0)
        self.assertGreater(forward, 0)
        self.assertGreater(right, 0)
        tracker = PlatformTracker()
        tracker.update(0.0, 0.0, 0.0)
        estimate = tracker.update(0.1, 0.0, 0.1)
        self.assertIsNotNone(estimate)
        controller = FormationController()
        controller.reset(0.1)
        command = controller.command(estimate, (0, 0), (0, 0), 0.13)
        self.assertTrue(command.valid)
        self.assertLessEqual((command.vx_m_s ** 2 + command.vy_m_s ** 2) ** 0.5, 0.40)

    def test_static_servo_left_is_positive_x_and_stale_is_zero(self):
        class FakeReader:
            def __init__(self):
                self.observation = None

            def latest(self, now, max_age_s):
                if self.observation is None or self.observation.age_s(now) > max_age_s:
                    return None
                return self.observation

            def is_running(self):
                return True

        reader = FakeReader()
        servo = StaticSquareServo(
            reader, StaticSquareServoConfig(vision_target_source="blue_square")
        )
        servo.arm(0.0, (0.0, 0.0))
        for seq, now in enumerate((0.03, 0.06, 0.09), 1):
            reader.observation = PlatformObservation(
                1, seq, seq, True, 220, 240, 80, 0, 0, 90,
                int(FeatureFlag.SURROGATE_SQUARE), now,
            )
            servo({
                "now_monotonic": now,
                "position_m": (0.0, 0.0, 1.5),
                "velocity_m_s": (0.0, 0.0, 0.0),
                "t265_confidence": 3,
            })
        self.assertGreater(servo.snapshot().command_m_s[0], 0.0)
        self.assertAlmostEqual(servo.snapshot().command_m_s[1], 0.0, places=6)
        decision = servo({
            "now_monotonic": 0.30,
            "position_m": (0.0, 0.0, 1.5),
            "velocity_m_s": (0.0, 0.0, 0.0),
            "t265_confidence": 3,
        })
        self.assertEqual((decision["vx_cms"], decision["vy_cms"]), (0, 0))
        self.assertEqual(servo.snapshot().mode, "LOST")

    def test_static_servo_hard_geofence_and_t265_fault_latch(self):
        class EmptyReader:
            def latest(self, now, max_age_s):
                return None

            def is_running(self):
                return True

        servo = StaticSquareServo(EmptyReader(), StaticSquareServoConfig())
        servo.arm(1.0, (0.0, 0.0))
        decision = servo({
            "now_monotonic": 1.03,
            "position_m": (0.61, 0.0, 1.5),
            "velocity_m_s": (0.0, 0.0, 0.0),
            "t265_confidence": 3,
        })
        self.assertTrue(decision["fault"])
        self.assertEqual(decision["reason"], "hard_geofence")
        self.assertTrue(servo.faulted)
        with self.assertRaises(RuntimeError):
            servo.arm(2.0, (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
