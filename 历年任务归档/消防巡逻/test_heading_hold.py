import math

import pytest

from Lcode.heading_hold import (
    HeadingHoldConfig,
    HeadingHoldController,
    wrap_degrees,
)


def _controller(**overrides):
    config = HeadingHoldConfig(enabled=True, **overrides)
    controller = HeadingHoldController(config)
    controller.arm(0.0, now=0.0)
    return controller


def test_wrap_degrees_uses_shortest_path_at_boundary():
    assert wrap_degrees(358.0) == -2.0
    assert wrap_degrees(-358.0) == 2.0
    assert wrap_degrees(180.0) == -180.0


def test_positive_current_yaw_produces_negative_command():
    status = _controller().update(math.radians(5.0), confidence=3, now=0.1)
    assert status.error_deg == pytest.approx(-5.0)
    assert status.command_dps == -1


def test_negative_current_yaw_produces_positive_command():
    status = _controller().update(math.radians(-5.0), confidence=3, now=0.1)
    assert status.error_deg == pytest.approx(5.0)
    assert status.command_dps == 1


def test_deadband_outputs_zero():
    status = _controller().update(math.radians(1.5), confidence=3, now=0.1)
    assert status.command_dps == 0


def test_minimum_integer_command_outside_deadband_is_symmetric():
    controller = _controller(kp=0.1)
    assert controller.update(math.radians(2.0), 3, 0.1).command_dps == -1
    controller.reset_for_new_mission()
    controller.arm(0.0, 0.2)
    assert controller.update(math.radians(-2.0), 3, 0.3).command_dps == 1


def test_command_is_limited():
    status = _controller(kp=0.5, max_rate_dps=2, fault_error_deg=90.0).update(
        math.radians(-20.0), confidence=3, now=0.1
    )
    assert status.command_dps == 2


def test_disabled_controller_does_not_arm_or_command():
    controller = HeadingHoldController(HeadingHoldConfig(enabled=False))
    arm_status = controller.arm(math.radians(10), now=0.0)
    status = controller.update(math.radians(20), confidence=3, now=0.1)
    assert arm_status.armed is False
    assert status.command_dps == 0
    assert status.degraded_reason == "disabled"


def test_unarmed_controller_outputs_zero():
    controller = HeadingHoldController(HeadingHoldConfig(enabled=True))
    status = controller.update(math.radians(3), confidence=3, now=0.1)
    assert status.command_dps == 0
    assert status.degraded_reason == "not_armed"


def test_low_confidence_temporarily_degrades_but_preserves_target():
    controller = _controller(fault_error_deg=10.0)
    degraded = controller.update(math.radians(3), confidence=1, now=0.1)
    recovered = controller.update(math.radians(3), confidence=3, now=0.2)
    assert degraded.command_dps == 0
    assert degraded.degraded_reason == "low_confidence"
    assert degraded.target_deg == 0.0
    assert recovered.command_dps == -1
    assert recovered.fault_reason is None


def test_large_error_after_confidence_recovery_latches_fault():
    controller = _controller(fault_error_deg=8.0)
    controller.update(math.radians(2), confidence=1, now=0.1)
    status = controller.update(math.radians(9), confidence=3, now=0.2)
    assert status.command_dps == 0
    assert "exceeds_limit" in status.fault_reason


def test_arm_only_latches_target_once():
    controller = HeadingHoldController(HeadingHoldConfig(enabled=True))
    controller.arm(math.radians(10), now=0.0)
    controller.arm(math.radians(20), now=0.1)
    assert controller.target_deg == pytest.approx(10.0)


def test_hard_error_fault_is_latched_until_new_mission():
    controller = _controller(fault_error_deg=8.0)
    first = controller.update(math.radians(9), confidence=3, now=0.1)
    still_faulted = controller.update(math.radians(0), confidence=3, now=0.2)
    assert first.command_dps == 0
    assert still_faulted.command_dps == 0
    assert still_faulted.fault_reason == first.fault_reason

    controller.reset_for_new_mission()
    controller.arm(0.0, now=0.3)
    recovered = controller.update(math.radians(3), confidence=3, now=0.4)
    assert recovered.command_dps == -1
    assert recovered.fault_reason is None


def test_runaway_growth_with_same_command_direction_latches_fault():
    controller = _controller(
        fault_error_deg=20.0,
        runaway_window_s=1.0,
        runaway_growth_deg=3.0,
    )
    controller.update(math.radians(2), confidence=3, now=0.0)
    status = controller.update(math.radians(5.2), confidence=3, now=1.0)
    assert status.command_dps == 0
    assert "grew" in status.fault_reason


def test_runaway_window_resets_when_command_changes_direction():
    controller = _controller(
        fault_error_deg=20.0,
        runaway_window_s=1.0,
        runaway_growth_deg=3.0,
    )
    controller.update(math.radians(2), confidence=3, now=0.0)
    controller.update(math.radians(-2), confidence=3, now=0.8)
    status = controller.update(math.radians(-4), confidence=3, now=1.1)
    assert status.fault_reason is None
    assert status.command_dps == 1


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


def test_from_env_defaults_enabled_and_parses_overrides():
    assert HeadingHoldConfig.from_env({}).enabled is True
    assert HeadingHoldConfig.from_env({"DRONE_HEADING_HOLD": "0"}).enabled is False
    config = HeadingHoldConfig.from_env(
        {
            "DRONE_HEADING_HOLD": "1",
            "DRONE_HEADING_HOLD_KP": "0.3",
            "DRONE_HEADING_HOLD_DEADBAND_DEG": "2.0",
            "DRONE_HEADING_HOLD_MAX_DPS": "2",
        }
    )
    assert config == HeadingHoldConfig(
        enabled=True,
        kp=0.3,
        deadband_deg=2.0,
        max_rate_dps=2,
    )
