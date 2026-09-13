import unittest

import cv2
import numpy as np

try:
    from .detector import (
        AprilTagBlueFusionDetector, AprilTagDetector, BlueSquareDetector,
        FeatureFlag, RingCrossDetector,
    )
except ImportError:
    from detector import (
        AprilTagBlueFusionDetector, AprilTagDetector, BlueSquareDetector,
        FeatureFlag, RingCrossDetector,
    )


def april_frame(tag_id=0, x=260, y=180, side=120):
    image = np.full((480, 640, 3), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, side)
    image[y:y + side, x:x + side] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


def fusion_frame(tag_id=0, blue_center=(320, 240), blue_side=250, tag_side=120):
    image = np.full((480, 640, 3), 220, np.uint8)
    cx, cy = blue_center
    half = blue_side // 2
    cv2.rectangle(
        image, (cx - half, cy - half), (cx + half, cy + half),
        (255, 90, 0), -1,
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, tag_side)
    x = cx - tag_side // 2
    y = cy - tag_side // 2
    image[y:y + tag_side, x:x + tag_side] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


def ring_cross_frame(center=(320, 240), side=300, visible_corners=(0, 1, 2, 3)):
    image = np.full((480, 640, 3), 225, np.uint8)
    cx, cy = center
    half = side // 2
    cv2.circle(image, (cx, cy), int(side * 0.44), (15, 15, 15), 12)
    cv2.circle(image, (cx, cy), int(side * 0.27), (15, 15, 15), 12)
    cv2.line(image, (cx - half, cy), (cx + half, cy), (15, 15, 15), 12)
    cv2.line(image, (cx, cy - half), (cx, cy + half), (15, 15, 15), 12)
    corners = (
        (cx - half, cy - half), (cx + half, cy - half),
        (cx + half, cy + half), (cx - half, cy + half),
    )
    red_side = max(22, int(round(side / 6.2)))
    red_half = red_side // 2
    for index in visible_corners:
        x, y = corners[index]
        cv2.rectangle(
            image, (x - red_half, y - red_half),
            (x + red_half, y + red_half), (0, 0, 230), -1,
        )
    return image


class AprilTagFlowSafetyTest(unittest.TestCase):
    def test_direct_decode_then_optical_flow_translation(self):
        detector = AprilTagDetector(redetect_interval=100)
        direct = detector.detect(april_frame())
        self.assertTrue(direct.found)
        self.assertFalse(FeatureFlag(direct.flags) & FeatureFlag.TEMPORAL_TRACKED)

        tracked = detector.detect(april_frame(x=268, y=186))
        self.assertTrue(tracked.found)
        self.assertTrue(FeatureFlag(tracked.flags) & FeatureFlag.APRILTAG_VALID)
        self.assertTrue(FeatureFlag(tracked.flags) & FeatureFlag.TEMPORAL_TRACKED)
        self.assertAlmostEqual(tracked.cx - direct.cx, 8, delta=2)
        self.assertAlmostEqual(tracked.cy - direct.cy, 6, delta=2)

    def test_wrong_id_clears_existing_track(self):
        detector = AprilTagDetector(redetect_interval=1)
        self.assertTrue(detector.detect(april_frame(tag_id=0)).found)
        wrong = detector.detect(april_frame(tag_id=1))
        self.assertFalse(wrong.found)
        self.assertTrue(FeatureFlag(wrong.flags) & FeatureFlag.AMBIGUOUS)
        self.assertIsNone(detector._track_points)


class BlueSquareSafetyTest(unittest.TestCase):
    def test_complete_square(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (110, 70), (210, 170), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertTrue(result.found)
        self.assertFalse(FeatureFlag(result.flags) & FeatureFlag.PARTIAL)

    def test_edge_square_is_partial(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (105, 185), (215, 239), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertTrue(result.found)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.PARTIAL)
        self.assertEqual(result.outer_px, 0)

    def test_two_candidates_are_ambiguous(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.rectangle(image, (30, 70), (100, 140), (255, 90, 0), -1)
        cv2.rectangle(image, (210, 70), (280, 140), (255, 90, 0), -1)
        result = BlueSquareDetector().detect(image)
        self.assertFalse(result.found)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.AMBIGUOUS)

    def test_blue_line_is_rejected(self):
        image = np.full((240, 320, 3), 220, np.uint8)
        cv2.line(image, (0, 120), (319, 120), (255, 90, 0), 5)
        self.assertFalse(BlueSquareDetector().detect(image).found)


class AprilTagBlueFusionSafetyTest(unittest.TestCase):
    def test_direct_tag_locks_identity_and_blue_drives_center(self):
        detector = AprilTagBlueFusionDetector(redetect_interval=1)
        result = detector.detect(fusion_frame())
        flags = FeatureFlag(result.flags)
        self.assertTrue(result.found)
        self.assertTrue(flags & FeatureFlag.APRILTAG_VALID)
        self.assertTrue(flags & FeatureFlag.COLOR_SHAPE_TRACKED)
        self.assertAlmostEqual(result.cx, 320, delta=3)
        self.assertAlmostEqual(result.cy, 240, delta=3)

    def test_blue_cannot_initialize_identity_by_itself(self):
        detector = AprilTagBlueFusionDetector(redetect_interval=1)
        image = np.full((480, 640, 3), 220, np.uint8)
        cv2.rectangle(image, (195, 115), (445, 365), (255, 90, 0), -1)
        result = detector.detect(image)
        self.assertFalse(result.found)
        self.assertFalse(detector._identity_locked)


class RingCrossSafetyTest(unittest.TestCase):
    def test_four_red_corners_and_cross_locate_center(self):
        result = RingCrossDetector().detect(ring_cross_frame(center=(350, 220)))
        self.assertTrue(result.found)
        self.assertAlmostEqual(result.cx, 350, delta=8)
        self.assertAlmostEqual(result.cy, 220, delta=8)
        flags = FeatureFlag(result.flags)
        self.assertTrue(flags & FeatureFlag.COLOR_SHAPE_TRACKED)
        self.assertTrue(flags & FeatureFlag.CROSS_VALID)

    def test_two_same_edge_corners_still_locate_center(self):
        result = RingCrossDetector().detect(
            ring_cross_frame(center=(410, 240), visible_corners=(0, 3))
        )
        self.assertTrue(result.found)
        self.assertAlmostEqual(result.cx, 410, delta=10)
        self.assertAlmostEqual(result.cy, 240, delta=10)
        self.assertTrue(FeatureFlag(result.flags) & FeatureFlag.PARTIAL)

    def test_small_red_distractor_does_not_bias_three_corner_center(self):
        image = ring_cross_frame(
            center=(410, 225), visible_corners=(0, 1, 3)
        )
        cv2.rectangle(image, (70, 400), (88, 418), (0, 0, 230), -1)
        result = RingCrossDetector().detect(image)
        self.assertTrue(result.found)
        self.assertAlmostEqual(result.cx, 410, delta=10)
        self.assertAlmostEqual(result.cy, 225, delta=10)

    def test_dark_mirror_side_does_not_steal_two_corner_center(self):
        image = ring_cross_frame(
            center=(520, 235), visible_corners=(0, 3)
        )
        cv2.rectangle(image, (120, 60), (360, 410), (20, 20, 20), -1)
        result = RingCrossDetector().detect(image)
        self.assertTrue(result.found)
        self.assertGreater(result.cx, 430)
        self.assertAlmostEqual(result.cy, 235, delta=12)

    def test_estimated_center_outside_frame_is_rejected(self):
        image = ring_cross_frame(
            center=(320, -20), visible_corners=(2, 3)
        )
        self.assertFalse(RingCrossDetector().detect(image).found)

    def test_map_cross_without_red_anchor_is_rejected(self):
        image = np.full((480, 640, 3), 225, np.uint8)
        cv2.line(image, (50, 240), (590, 240), (10, 10, 10), 14)
        cv2.line(image, (320, 30), (320, 450), (10, 10, 10), 14)
        self.assertFalse(RingCrossDetector().detect(image).found)

    def test_red_square_without_ring_cross_is_rejected(self):
        image = np.full((480, 640, 3), 225, np.uint8)
        cv2.rectangle(image, (80, 80), (135, 135), (0, 0, 230), -1)
        self.assertFalse(RingCrossDetector().detect(image).found)

    def test_wrong_tag_clears_identity_lock(self):
        detector = AprilTagBlueFusionDetector(redetect_interval=1)
        self.assertTrue(detector.detect(fusion_frame()).found)
        wrong = detector.detect(fusion_frame(tag_id=1))
        self.assertFalse(wrong.found)
        self.assertTrue(FeatureFlag(wrong.flags) & FeatureFlag.AMBIGUOUS)
        self.assertFalse(detector._identity_locked)


if __name__ == "__main__":
    unittest.main()
