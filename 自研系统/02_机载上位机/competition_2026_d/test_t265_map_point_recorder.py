import math

import pytest

from .t265_map_point_recorder import (
    MapPointRecorder,
    PoseSample,
    measurement_rejection_reasons,
    summarize_samples,
)


def samples(x=1.0, y=2.0, confidence=3):
    return [
        PoseSample(i * 0.05, x + dx, y - dx, 0.2, x + dx, y - dx, 0.2,
                   math.radians(2.0), 0.0, 0.0, confidence, 0.01)
        for i, dx in enumerate((-0.01, 0.0, 0.01))
    ]


def test_summary_uses_median_and_reports_confidence():
    result = summarize_samples(samples(), confidence_min=2)
    assert result["x_abs_m"] == pytest.approx(1.0)
    assert result["y_abs_m"] == pytest.approx(2.0)
    assert result["confidence_3_ratio"] == pytest.approx(1.0)
    assert result["std_x_cm"] > 0.0


def test_first_point_must_be_h_and_return_reports_closure():
    recorder = MapPointRecorder("H")
    with pytest.raises(ValueError):
        recorder.add_summary("A", summarize_samples(samples(), 2))
    start = recorder.add_summary("H", summarize_samples(samples(), 2))
    assert start["x_m"] == pytest.approx(0.0)
    point = recorder.add_summary("A", summarize_samples(samples(2.0, 4.0), 2))
    assert point["x_m"] == pytest.approx(1.0)
    assert point["y_m"] == pytest.approx(2.0)
    returned = recorder.add_summary("H", summarize_samples(samples(1.03, 1.96), 2))
    assert returned["closure_xy_m"] == pytest.approx(0.05)


def test_summary_rejects_low_confidence_only_samples():
    with pytest.raises(RuntimeError):
        summarize_samples(samples(confidence=1), confidence_min=2)


def test_unstable_measurement_is_rejected_before_csv_recording():
    unstable = [
        PoseSample(i * 0.05, float(i), float(i * 5), 0.2, float(i),
                   float(i * 5), 0.2, 0.0, 1.0, 5.0, 2, 0.01)
        for i in range(5)
    ]
    summary = summarize_samples(unstable, 2)
    reasons = measurement_rejection_reasons(summary, 2.0, 0.05, 0.10)
    assert any("标准差" in reason for reason in reasons)
    assert any("速度" in reason for reason in reasons)
    assert any("跨度" in reason for reason in reasons)


def test_step_jump_is_rejected_without_advancing_last_point():
    recorder = MapPointRecorder("H")
    recorder.add_summary("H", summarize_samples(samples(), 2))
    with pytest.raises(ValueError, match="跳变"):
        recorder.add_summary("C_IN", summarize_samples(samples(20.0, 30.0), 2))
    accepted = recorder.add_summary("A", summarize_samples(samples(2.0, 3.0), 2))
    assert accepted["step_from_previous_m"] == pytest.approx(math.sqrt(2.0))


def test_stale_pose_frame_is_rejected_even_with_good_confidence():
    stale = [
        PoseSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 3, 0.30)
    ]
    summary = summarize_samples(stale, 2)
    reasons = measurement_rejection_reasons(summary, 2.0, 0.05, 0.10)
    assert any("Pose帧年龄" in reason for reason in reasons)
