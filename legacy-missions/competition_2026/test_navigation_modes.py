import io
import json
import math
import time

import pytest

import Mission_GPT as mg
from Lcode.global_variable import sp_side
from Lcode.heading_hold import HeadingHoldConfig, HeadingHoldController
from Lcode.navigation_profile import NavigationProfileConfig
from Mission_GPT import mission


class FakeRealsense:
    def __init__(self, confidence=3, vel=(0.0, 0.0, 0.0), yaw=0.0):
        self.confidence = confidence
        self.vel = vel
        self.yaw = yaw

    def get_tracking_confidence(self):
        return self.confidence

    def get_velocity(self):
        return list(self.vel)

    def get_orientation(self):
        return [0.0, 0.0, self.yaw]


def _make_mission(profile="cruise"):
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    m.targets = [
        [0.0, 0.0, 1.0],
        [2.0, 0.0, 1.0],
        [4.0, 0.0, 1.0],
    ]
    m.navigation_profile = NavigationProfileConfig(profile=profile)
    m.t265_ok = True
    m.realsense = FakeRealsense()
    return m


def test_cruise_head_waypoint_does_not_ignore_height():
    m = _make_mission()
    for _ in range(3):
        m.navigate([0.0, 0.0, 0.3], 0.0)
    assert m.target_index == 0
    assert m._arrival_window[-1] is False


def test_middle_cruise_waypoint_advances_after_three_cycles_without_speed_or_z_gate():
    m = _make_mission()
    m.target_index = 1
    m.realsense.vel = (0.5, 0.0, 0.0)
    for _ in range(2):
        m.navigate([2.14, 0.0, 0.3], 0.0)
    assert m.target_index == 1
    m.navigate([2.14, 0.0, 0.3], 0.0)
    assert m.target_index == 2


def test_middle_cruise_waypoint_can_require_height_before_advancing():
    m = _make_mission()
    m.navigation_profile = NavigationProfileConfig(
        profile="cruise", cruise_confirm_cycles=2, cruise_require_z=True
    )
    m.target_index = 1
    for _ in range(3):
        m.navigate([2.14, 0.0, 0.3], 0.0)
    assert m.target_index == 1
    m.navigate([2.14, 0.0, 1.0], 0.0)
    m.navigate([2.14, 0.0, 1.0], 0.0)
    assert m.target_index == 2


def test_cruise_uses_circular_radius_and_resets_confirmation_when_leaving():
    m = _make_mission()
    m.target_index = 1
    for _ in range(3):
        m.navigate([2.11, 0.11, 1.0], 0.0)
    assert m.target_index == 1
    m.navigate([2.14, 0.0, 1.0], 0.0)
    m.navigate([2.14, 0.0, 1.0], 0.0)
    m.navigate([2.16, 0.0, 1.0], 0.0)
    m.navigate([2.14, 0.0, 1.0], 0.0)
    m.navigate([2.14, 0.0, 1.0], 0.0)
    assert m.target_index == 1
    m.navigate([2.14, 0.0, 1.0], 0.0)
    assert m.target_index == 2


def test_last_waypoint_remains_precision():
    m = _make_mission()
    m.target_index = 2
    m.realsense.vel = (0.5, 0.0, 0.0)
    for _ in range(3):
        m.navigate([4.0, 0.0, 0.3], 0.0)
    assert m.target_index == 2


def test_tracking_loss_pauses_timeout_and_clears_partial_confirmation():
    m = _make_mission()
    m.target_index = 1
    m.navigate([2.14, 0.0, 1.0], 0.0)
    m.navigate([2.14, 0.0, 1.0], 0.0)
    assert m._cruise_arrival_count == 2
    m.arrival_start_time = time.time() - 100.0
    m.realsense.confidence = 0
    m.navigate([2.14, 0.0, 1.0], 0.0)
    assert m.target_index == 1
    assert m._cruise_arrival_count == 0
    assert time.time() - m.arrival_start_time < 1.0


def test_arrival_and_timeout_same_tick_only_advance_once():
    m = _make_mission()
    m.target_index = 1
    m.navigate([2.14, 0.0, 1.0], 0.0)
    m.navigate([2.14, 0.0, 1.0], 0.0)
    m.arrival_start_time = time.time() - 100.0
    m.navigate([2.14, 0.0, 1.0], 0.0)
    assert m.target_index == 2
    assert time.time() - m.arrival_start_time < 1.0


def test_waypoint_event_keeps_completed_index_and_target_together():
    m = _make_mission()
    m.target_index = 1
    m._log_file = io.StringIO()
    for _ in range(3):
        m.navigate([2.14, 0.0, 1.0], 0.0)
    entries = [json.loads(line) for line in m._log_file.getvalue().splitlines()]
    event = [entry for entry in entries if entry.get("event") == "waypoint_advance"][-1]
    assert event["target_idx"] == 1
    assert event["target"] == [2.0, 0.0, 1.0]
    assert event["reason"] == "cruise_arrival"


def test_heading_hold_command_is_shared_by_precision_and_cruise():
    for profile in ("precision", "cruise"):
        m = _make_mission(profile)
        m.heading_hold = HeadingHoldController(
            HeadingHoldConfig(enabled=True, fault_error_deg=20.0)
        )
        m.heading_hold.arm(0.0, 0.0)
        m.target_index = 1
        m.navigate([1.0, 0.0, 1.0], math.radians(5.0))
        assert m.se_fc[6] - sp_side == -1, profile


def test_emergency_disarms_heading_hold_and_clears_yaw():
    m = _make_mission()
    m.heading_hold = HeadingHoldController(HeadingHoldConfig(enabled=True))
    m.heading_hold.arm(0.0, 0.0)
    m.se_fc[6] = 3 + sp_side
    m.emergency()
    assert m.heading_hold.armed is False
    assert m.se_fc[6] - sp_side == 0
    assert m.emergency_stop is True


def test_takeoff_latches_current_heading_before_unlock(monkeypatch):
    m = _make_mission()
    m.realsense.yaw = math.radians(12.0)
    monkeypatch.setattr(mg, "TAKEOFF_TIMEOUT_S", 0.0)
    monkeypatch.setattr(mg, "DRY_RUN", True)
    m.takeoff()
    assert m.heading_hold.armed is True
    assert m.heading_hold.target_deg == pytest.approx(12.0)
    assert m.se_fc[6] - sp_side == 0
