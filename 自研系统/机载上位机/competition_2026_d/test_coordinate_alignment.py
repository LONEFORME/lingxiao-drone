import math

import numpy as np

from .coordinate_alignment import (
    OFFICIAL_FIELD_POINTS_M,
    fit_labeled_measurements,
    fit_rigid_transform_2d,
)


def test_rigid_transform_recovers_rotation_and_translation():
    source = np.array(
        [
            [0.0, 0.0],
            [1.5, 0.0],
            [1.5, 1.5],
            [0.0, 1.5],
        ]
    )
    angle = math.radians(90.0)
    rotation = np.array(
        [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ]
    )
    translation = np.array([1.5, 2.0])
    target = (rotation @ source.T).T + translation

    result = fit_rigid_transform_2d(source, target)

    assert math.isclose(result.rotation_deg, 90.0, abs_tol=1e-9)
    assert math.isclose(result.translation_x_m, 1.5, abs_tol=1e-9)
    assert math.isclose(result.translation_y_m, 2.0, abs_tol=1e-9)
    assert math.isclose(result.scale_diagnostic, 1.0, abs_tol=1e-9)
    assert result.rms_error_m < 1e-9
    assert not result.reflection_suspected


def test_reflected_axis_is_reported_instead_of_hidden_in_transform():
    source = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [0.0, 2.0],
            [1.5, 1.0],
        ]
    )
    target = source.copy()
    target[:, 0] *= -1.0
    target += np.array([3.0, 1.0])

    result = fit_rigid_transform_2d(source, target)

    assert result.reflection_suspected
    assert result.mirrored_fit_rms_m < 1e-9
    assert result.rms_error_m > 0.20


def test_labeled_fit_uses_official_field_points_and_reports_residuals():
    h_x, h_y = OFFICIAL_FIELD_POINTS_M["H"]
    measurements = {
        label: (xy[0] - h_x, xy[1] - h_y)
        for label, xy in OFFICIAL_FIELD_POINTS_M.items()
    }

    transform, residuals = fit_labeled_measurements(measurements)

    assert math.isclose(transform.rotation_deg, 0.0, abs_tol=1e-9)
    assert math.isclose(transform.translation_x_m, h_x, abs_tol=1e-9)
    assert math.isclose(transform.translation_y_m, h_y, abs_tol=1e-9)
    assert transform.max_error_m < 1e-9
    assert set(residuals) == {"A", "B", "C", "D", "H"}
