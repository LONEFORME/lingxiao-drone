import math

import pytest

from Lcode.heading_hold import HeadingHoldConfig, HeadingHoldController, wrap_degrees


def _controller(**overrides):
    controller = HeadingHoldController(HeadingHoldConfig(enabled=True, **overrides))
    controller.arm(0.0, now=0.0)
    return controller


def test_default_config_enables_heading_hold():
    assert HeadingHoldConfig.from_env({}).enabled is True
    assert HeadingHoldConfig.from_env({"DRONE_HEADING_HOLD": "0"}).enabled is False


def test_wrap_degrees_uses_shortest_path_at_boundary():
    assert wrap_degrees(358.0) == -2.0
    assert wrap_degrees(-358.0) == 2.0
    assert wrap_degrees(180.0) == -180.0


def test_command_sign_deadband_and_limit():
    controller = _controller(fault_error_deg=90.0)
    assert controller.update(math.radians(5.0), 3, 0.1).command_dps == -1
    controller.reset_for_new_mission()
    controller.arm(0.0, 0.2)
    assert controller.update(math.radians(-5.0), 3, 0.3).command_dps == 1
    assert controller.update(math.radians(-1.5), 3, 0.4).command_dps == 0


def test_low_confidence_outputs_zero_and_preserves_target():
    controller = _controller(fault_error_deg=10.0)
    degraded = controller.update(math.radians(3.0), 1, 0.1)
    recovered = controller.update(math.radians(3.0), 3, 0.2)
    assert degraded.command_dps == 0
    assert degraded.degraded_reason == "low_confidence"
    assert degraded.target_deg == 0.0
    assert recovered.command_dps == -1


def test_hard_error_fault_latches_until_new_mission():
    controller = _controller(fault_error_deg=8.0)
    first = controller.update(math.radians(9.0), 3, 0.1)
    still_faulted = controller.update(0.0, 3, 0.2)
    assert first.command_dps == 0
    assert "exceeds_limit" in first.fault_reason
    assert still_faulted.fault_reason == first.fault_reason
    controller.reset_for_new_mission()
    controller.arm(0.0, 0.3)
    assert controller.update(math.radians(3.0), 3, 0.4).command_dps == -1


def test_submax_growth_does_not_latch_runaway_fault():
    controller = _controller(
        max_rate_dps=2,
        fault_error_deg=20.0,
        runaway_window_s=1.0,
        runaway_growth_deg=3.0,
    )
    controller.update(math.radians(2.0), 3, 0.0)
    status = controller.update(math.radians(5.2), 3, 1.0)
    assert status.command_dps == -1
    assert status.fault_reason is None


def test_runaway_growth_latches_after_max_command_for_full_window():
    controller = _controller(
        kp=0.5,
        max_rate_dps=2,
        fault_error_deg=20.0,
        runaway_window_s=1.0,
        runaway_growth_deg=3.0,
    )
    first_max = controller.update(math.radians(4.0), 3, 1.0)
    before_window = controller.update(math.radians(7.2), 3, 1.9)
    status = controller.update(math.radians(7.2), 3, 2.0)
    assert first_max.command_dps == -2
    assert before_window.command_dps == -2
    assert before_window.fault_reason is None
    assert status.command_dps == 0
    assert "grew" in status.fault_reason


def test_arm_only_latches_target_once():
    controller = HeadingHoldController(HeadingHoldConfig())
    controller.arm(math.radians(10.0), 0.0)
    controller.arm(math.radians(20.0), 0.1)
    assert controller.target_deg == pytest.approx(10.0)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"kp": 0.0},
        {"kp": 0.6},
        {"deadband_deg": 0.1},
        {"max_rate_dps": 0},
        {"max_rate_dps": 4},
        {"fault_error_deg": 1.0},
    ],
)
def test_config_rejects_unsafe_values(kwargs):
    with pytest.raises(ValueError):
        HeadingHoldConfig(**kwargs)
