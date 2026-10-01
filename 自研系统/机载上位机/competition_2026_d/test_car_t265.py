import math

import numpy as np

from .car_t265 import (
    CarT265Config,
    CarT265Pose,
    SENSOR_TO_CAR,
)


def _matrix_to_quaternion_xyzw(matrix):
    matrix = np.asarray(matrix, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    return np.array([x, y, z, w], dtype=float)


def _raw_pose_from_car_pose(position_car, yaw_rad):
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    rotation_car = np.array(
        [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    rotation_sensor = SENSOR_TO_CAR.T @ rotation_car @ SENSOR_TO_CAR
    translation_sensor = SENSOR_TO_CAR.T @ np.asarray(
        position_car,
        dtype=float,
    )
    return translation_sensor, _matrix_to_quaternion_xyzw(rotation_sensor)


def _ready_pose(**overrides):
    options = dict(
        calibration_duration_s=0.0,
        calibration_min_samples=1,
        position_filter_alpha=1.0,
        velocity_filter_alpha=1.0,
        max_position_jump_m=10.0,
    )
    options.update(overrides)
    pose = CarT265Pose(CarT265Config(**options))
    pose.begin_calibration()
    assert pose.update_pose(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        confidence=3,
        timestamp=1.1,
    )
    return pose


def test_forward_and_left_axes_match_project_convention():
    pose = _ready_pose()

    assert pose.update_pose(
        (0.0, 0.0, -0.50),
        (0.0, 0.0, 0.0, 1.0),
        confidence=3,
        timestamp=1.2,
    )
    np.testing.assert_allclose(
        pose.get_center_position(),
        (0.50, 0.0, 0.0),
        atol=1e-9,
    )

    assert pose.update_pose(
        (-0.40, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        confidence=3,
        timestamp=1.3,
    )
    np.testing.assert_allclose(
        pose.get_center_position(),
        (0.0, 0.40, 0.0),
        atol=1e-9,
    )


def test_left_turn_is_positive_heading():
    pose = _ready_pose()
    translation, quaternion = _raw_pose_from_car_pose(
        (0.0, 0.0, 0.0),
        math.radians(90.0),
    )

    assert pose.update_pose(
        translation,
        quaternion,
        confidence=3,
        timestamp=1.2,
    )
    assert math.isclose(
        pose.get_heading_rad(),
        math.pi / 2.0,
        abs_tol=1e-9,
    )
    assert pose.get_heading_cdeg() == 9000


def test_fixed_center_rotation_is_removed_before_filtering():
    pose = _ready_pose(position_filter_alpha=0.15)
    lever = pose.config.lever_arm_body_m

    for index, angle_deg in enumerate((15.0, 45.0, 90.0, 135.0), start=1):
        angle_rad = math.radians(angle_deg)
        cosine = math.cos(angle_rad)
        sine = math.sin(angle_rad)
        rotation_car = np.array(
            [
                [cosine, -sine, 0.0],
                [sine, cosine, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        sensor_position = (rotation_car - np.eye(3)) @ lever
        translation, quaternion = _raw_pose_from_car_pose(
            sensor_position,
            angle_rad,
        )
        assert pose.update_pose(
            translation,
            quaternion,
            confidence=3,
            timestamp=1.1 + index * 0.1,
        )
        np.testing.assert_allclose(
            pose.get_raw_center_position(),
            (0.0, 0.0, 0.0),
            atol=1e-9,
        )
        np.testing.assert_allclose(
            pose.get_center_position(),
            (0.0, 0.0, 0.0),
            atol=1e-9,
        )


def test_calibration_uses_only_valid_confidence_samples_and_averages_origin():
    pose = CarT265Pose(
        CarT265Config(
            calibration_duration_s=0.2,
            calibration_min_samples=3,
            position_filter_alpha=1.0,
            velocity_filter_alpha=1.0,
            max_position_jump_m=1.0,
        )
    )
    pose.begin_calibration()

    assert not pose.update_pose(
        (5.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        confidence=1,
        timestamp=10.05,
    )
    for timestamp, native_z in (
        (10.10, -0.99),
        (10.20, -1.00),
        (10.30, -1.01),
    ):
        accepted = pose.update_pose(
            (0.0, 0.0, native_z),
            (0.0, 0.0, 0.0, 1.0),
            confidence=3,
            timestamp=timestamp,
        )

    assert accepted
    assert pose.is_ready()
    assert pose.get_status(now=10.30).rejected_low_confidence == 1
    np.testing.assert_allclose(
        pose.get_center_position(),
        (0.0, 0.0, 0.0),
        atol=1e-9,
    )

    assert pose.update_pose(
        (0.0, 0.0, -1.50),
        (0.0, 0.0, 0.0, 1.0),
        confidence=3,
        timestamp=10.40,
    )
    np.testing.assert_allclose(
        pose.get_center_position(),
        (0.50, 0.0, 0.0),
        atol=1e-9,
    )


def test_low_confidence_jump_and_stale_pose_are_rejected():
    pose = _ready_pose(
        max_position_jump_m=0.20,
        max_pose_age_s=0.25,
    )
    assert pose.update_pose(
        (0.0, 0.0, -0.10),
        (0.0, 0.0, 0.0, 1.0),
        confidence=3,
        timestamp=1.2,
    )
    valid_position = pose.get_center_position()

    assert not pose.update_pose(
        (0.0, 0.0, -0.15),
        (0.0, 0.0, 0.0, 1.0),
        confidence=1,
        timestamp=1.3,
    )
    assert not pose.update_pose(
        (0.0, 0.0, -1.00),
        (0.0, 0.0, 0.0, 1.0),
        confidence=3,
        timestamp=1.4,
    )
    np.testing.assert_allclose(
        pose.get_center_position(),
        valid_position,
        atol=1e-9,
    )
    assert pose.is_healthy(now=1.44)
    assert not pose.is_healthy(now=1.46)

    status = pose.get_status(now=1.46)
    assert status.rejected_low_confidence == 1
    assert status.rejected_jump == 1
    assert status.last_error == "position_jump"


def test_center_velocity_uses_compensated_position():
    pose = _ready_pose()
    assert pose.update_pose(
        (0.0, 0.0, -0.10),
        (0.0, 0.0, 0.0, 1.0),
        confidence=3,
        timestamp=1.2,
    )
    np.testing.assert_allclose(
        pose.get_center_velocity(),
        (1.0, 0.0, 0.0),
        atol=1e-9,
    )
