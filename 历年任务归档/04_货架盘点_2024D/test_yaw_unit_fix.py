"""旧 yaw_pid 退役后的任务级航向保持回归测试。"""

import math

import pytest

from Lcode.global_variable import sp_side
from Lcode.heading_hold import HeadingHoldConfig, HeadingHoldController
from Mission_GPT import mission


class FakeRealsenseYaw:
    def __init__(self, confidence=3, vel=(0.0, 0.0, 0.0)):
        self._confidence = confidence
        self._vel = vel

    def get_tracking_confidence(self):
        return self._confidence

    def get_velocity(self):
        return list(self._vel)


def _make_mission():
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    m.heading_source = "t265"
    m.t265_ok = True
    m.realsense = FakeRealsenseYaw()
    m.heading_hold = HeadingHoldController(
        HeadingHoldConfig(enabled=True, fault_error_deg=20.0)
    )
    m.heading_hold.arm(0.0, now=0.0)
    return m


def test_old_yaw_pid_is_removed():
    assert not hasattr(_make_mission(), "yaw_pid")


def test_t265_navigate_preserves_controller_command_sign():
    m = _make_mission()
    target = m.targets[0]
    m.navigate([target[0] + 0.5, target[1], target[2]], math.radians(5.0))
    assert m._heading_status.error_deg == pytest.approx(-5.0)
    assert m.se_fc[6] - sp_side == -1


def test_small_error_inside_deadband_sends_zero():
    m = _make_mission()
    target = m.targets[0]
    m.navigate([target[0] + 0.5, target[1], target[2]], math.radians(0.5))
    assert m.se_fc[6] - sp_side == 0
