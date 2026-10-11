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


class ScanRealsense(FakeRealsense):
    def __init__(self, position=(0.2, -0.1, 1.2), confidence=3, yaw=0.0):
        super().__init__(confidence=confidence, vel=(0.0, 0.0, 0.0), yaw=yaw)
        self.position = position

    def get_position(self):
        return self.position


def _make_mission(profile="cruise"):
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    # Existing navigation tests exercise the legacy T265 feedback semantics;
    # dedicated tests below cover the new production-default FC source.
    m.heading_source = "t265"
    m.targets = [
        [0.0, 0.0, 1.0],
        [2.0, 0.0, 1.0],
        [4.0, 0.0, 1.0],
    ]
    m.navigation_profile = NavigationProfileConfig(profile=profile)
    m.t265_ok = True
    m.realsense = FakeRealsense()
    return m


def test_dispatch_exception_neutralizes_xy_and_enters_land(monkeypatch):
    m = _make_mission()
    m.state = "NAVIGATE"
    m.se_fc[3] = sp_side + 20
    m.se_fc[4] = sp_side - 20
    m.task_running = True
    monkeypatch.setattr(m, "navigate", lambda *_: (_ for _ in ()).throw(RuntimeError("boom")))

    try:
        m._dispatch_state_tick([0.0, 0.0, 1.0], 0.0)
    except RuntimeError as exc:
        m.on_flight_loop_exception(exc, "NAVIGATE")

    assert m.se_fc[3] == sp_side
    assert m.se_fc[4] == sp_side
    assert m.state == "LAND"
    assert m.task_running is True


def test_land_exception_requests_land_before_emergency_stop():
    m = _make_mission()
    m.state = "LAND"
    m.se_fc[2] = 1
    m.se_fc[3] = sp_side + 20
    m.se_fc[4] = sp_side - 20

    m.on_flight_loop_exception(RuntimeError("land failed"), "LAND")

    assert m.se_fc[2] == 0
    assert m.se_fc[3] == sp_side
    assert m.se_fc[4] == sp_side
    assert m.se_fc[6] == sp_side
    assert m.emergency_stop is True


def test_unrecoverable_loop_handler_bare_writes_disarm_and_stops_task():
    m = _make_mission()
    m.task_running = True
    m.se_fc[2] = 1
    m.se_fc[3] = sp_side + 20
    m.se_fc[4] = sp_side - 20

    m._flight_loop_unrecoverable()

    assert m.se_fc[2] == 0
    assert m.se_fc[3] == sp_side
    assert m.se_fc[4] == sp_side
    assert m.se_fc[6] == sp_side
    assert m.se_fc[7] == 101
    assert m.task_running is False


def test_flight_loop_exception_event_records_context():
    m = _make_mission()
    m._log_file = io.StringIO()
    m._navigation_purpose = "return"
    m.target_index = 1

    m._log_flight_loop_exception(ValueError("bad transition"), "NAVIGATE")

    event = json.loads(m._log_file.getvalue())
    assert event["event"] == "flight_loop_exception"
    assert event["state"] == "NAVIGATE"
    assert event["exception_type"] == "ValueError"
    assert event["navigation_purpose"] == "return"
    assert event["target_idx"] == 1


def test_scan_tick_uses_xy_pid_against_latched_target():
    m = _make_mission()
    m.realsense = ScanRealsense(position=(0.20, -0.10, 1.20))
    m._ramp_z_cm = 120.0
    m.begin_scan_hold((0.0, 0.0, 1.25))
    commands = []
    m.set_speed = lambda x, y, yaw, z: commands.append((x, y, yaw, z))

    # SCAN must use its latched target, not the mutable navigation targets.
    m.targets = [(9.0, 9.0, 2.0)]
    m.scan_tick([0.20, -0.10, 1.20], 0.0)

    assert m.state == "SCAN"
    assert commands
    assert commands[-1][0] < 0
    assert commands[-1][1] > 0
    assert commands[-1][3] >= 120


def test_scan_tick_preserves_z_ramp_and_heading_hold():
    m = _make_mission()
    m.realsense = ScanRealsense(position=(0.0, 0.0, 1.20), yaw=0.05)
    m._ramp_z_cm = 120.0
    m.begin_scan_hold((0.0, 0.0, 1.25))
    commands = []
    m.set_speed = lambda x, y, yaw, z: commands.append((x, y, yaw, z))

    m.scan_tick([0.0, 0.0, 1.20], 0.05)

    assert commands[-1][3] >= 120
    assert commands[-1][3] <= 125
    assert commands[-1][2] == m._heading_status.command_dps


def test_scan_tracking_loss_calls_hook_without_waypoint_advance():
    m = _make_mission()
    m.realsense = ScanRealsense(confidence=0)
    m.target_index = 2
    events = []
    m.on_scan_tracking_lost = lambda pos, yaw: events.append((pos, yaw))
    m.begin_scan_hold((0.0, 0.0, 1.25))

    m.scan_tick([0.0, 0.0, 1.25], 0.0)

    assert events == [([0.0, 0.0, 1.25], 0.0)]
    assert m.target_index == 2


def test_end_scan_hold_clears_latched_target():
    m = _make_mission()
    m.begin_scan_hold((0.0, 0.0, 1.25))

    m.end_scan_hold()

    assert m._scan_target is None


def _armed_recovery_mission(monkeypatch, now=100.0):
    clock = {"now": float(now)}
    monkeypatch.setattr(mg.time, "time", lambda: clock["now"])
    m = _make_mission(profile="precision")
    m.heading_hold.reset_for_new_mission()
    m.heading_hold.arm(0.0, clock["now"])
    m._heading_status = m.heading_hold.update(0.0, 3, clock["now"])
    m._ramp_z_cm = 125.0
    m.state = "NAVIGATE"
    commands = []
    m.set_speed = lambda x, y, yaw, z: commands.append((x, y, yaw, z))
    return m, clock, commands


def test_heading_fault_stops_xy_and_uses_bounded_recovery(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)

    control = m.position_control_tick(
        m.targets[0], [0.2, -0.1, 1.2], math.radians(-9.0)
    )

    assert control["heading_recovery_active"] is True
    assert control["heading_recovery_failed"] is False
    assert commands[-1] == (0, 0, 5, 120)
    assert m.state == "NAVIGATE"


def test_heading_recovery_without_progress_bypasses_heading_and_continues(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.2], math.radians(-9.0))
    clock["now"] += mg.HEADING_RECOVERY_PROGRESS_WINDOW_S

    control = m.position_control_tick(
        m.targets[0], [0.0, 0.0, 1.2], math.radians(-8.8)
    )

    assert control["heading_recovery_failed"] is False
    assert "no_progress" in control["heading_recovery_reason"]
    assert commands[-1] == (0, 0, 0, 120)
    assert m.state == "NAVIGATE"
    assert m._heading_bypass_active is True

    resumed = m.position_control_tick(
        m.targets[0], [0.2, -0.1, 1.2], math.radians(-8.8)
    )
    assert resumed["vx"] != 0
    assert resumed["vy"] != 0
    assert resumed["yaw_cmd"] == 0
    assert m.state == "NAVIGATE"


def test_heading_recovery_hard_error_bypasses_heading_without_landing(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)

    control = m.position_control_tick(
        m.targets[0], [0.0, 0.0, 1.2], math.radians(-21.0)
    )

    assert control["heading_recovery_failed"] is False
    assert "exceeds_recovery_limit" in control["heading_recovery_reason"]
    assert m._heading_bypass_active is True
    assert m.state == "NAVIGATE"


def test_heading_recovery_timeout_bypasses_heading_without_landing(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.2], math.radians(-9.0))
    for elapsed, error in ((1.0, -8.0), (2.0, -7.0)):
        clock["now"] = 100.0 + elapsed
        control = m.position_control_tick(
            m.targets[0], [0.0, 0.0, 1.2], math.radians(error)
        )
        assert control["heading_recovery_active"] is True
    clock["now"] = 100.0 + mg.HEADING_RECOVERY_TIMEOUT_S

    control = m.position_control_tick(
        m.targets[0], [0.0, 0.0, 1.2], math.radians(-6.0)
    )

    assert control["heading_recovery_failed"] is False
    assert control["heading_recovery_reason"] == "heading_recovery_timeout"
    assert m._heading_bypass_active is True
    assert m.state == "NAVIGATE"


def test_heading_recovery_low_confidence_never_sends_recovery_yaw(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.2], math.radians(-9.0))
    m.realsense.confidence = 1
    clock["now"] += 0.1

    control = m.position_control_tick(
        m.targets[0], [0.0, 0.0, 1.2], math.radians(-8.0)
    )

    assert control["heading_recovery_failed"] is True
    assert control["heading_recovery_reason"] == "heading_recovery_low_confidence_1"
    assert commands[-1][2] == 0
    assert m.state == "LAND"


def test_heading_recovery_uses_hysteresis_and_preserves_target(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.2], math.radians(-9.0))
    clock["now"] += 0.4
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.2], math.radians(-4.9))
    clock["now"] += 0.5
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.2], math.radians(-5.6))
    clock["now"] += 0.1
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.2], math.radians(-4.9))
    clock["now"] += mg.HEADING_RECOVERY_STABLE_S

    control = m.position_control_tick(
        m.targets[0], [0.0, 0.0, 1.2], math.radians(-4.9)
    )

    assert control["heading_recovery_completed"] is True
    assert m.heading_hold.target_deg == pytest.approx(0.0)
    assert m._heading_recovery_successes == 1
    assert commands[-1] == (0, 0, 0, 120)


def test_recovery_repeat_limit_bypasses_heading_without_landing(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)
    m._heading_recovery_successes = mg.HEADING_RECOVERY_MAX_SUCCESSES

    control = m.position_control_tick(
        m.targets[0], [0.0, 0.0, 1.2], math.radians(-9.0)
    )

    assert control["heading_recovery_failed"] is False
    assert control["heading_recovery_reason"] == "heading_recovery_repeat_limit"
    assert m._heading_bypass_active is True
    assert m.state == "NAVIGATE"


def test_scan_tick_writes_continuous_flight_log(monkeypatch):
    m, clock, commands = _armed_recovery_mission(monkeypatch)
    m.realsense = ScanRealsense(position=(0.0, 0.0, 1.2), yaw=0.0)
    m._log_file = io.StringIO()
    m._last_log_time = 0.0
    m.begin_scan_hold((0.0, 0.0, 1.25))

    m.scan_tick([0.0, 0.0, 1.2], 0.0)

    entry = json.loads(m._log_file.getvalue())
    assert entry["state"] == "SCAN"
    assert entry["yaw_cmd_sent"] == 0
    assert entry["height_setpoint_cm"] >= 120
    assert entry["heading_recovery_active"] is False


def test_replace_navigation_targets_resets_all_arrival_and_pid_state(monkeypatch):
    m = _make_mission()
    m.target_index = 3
    m.last_target_index = 3
    m._arrival_window.extend([True, True])
    m._vel_window.extend([(0.1, 0.1)])
    m.arrival_confirmed_time = 12.0
    m.arrival_start_time = 11.0
    m._cruise_arrival_count = 4
    m._active_segment_distance_m = 99.0
    m._ramp_z_cm = 123.0
    m.x_pid.pid._integral = 7.0
    m.y_pid.pid._integral = -3.0
    monkeypatch.setattr(mg.time, "time", lambda: 50.0)

    generation = m.replace_navigation_targets(
        [(0.1, 0.2, 1.4), (0.0, 0.0, 1.4)],
        [0.3, 0.4, 1.2],
        purpose="return",
    )

    assert m.targets == [(0.1, 0.2, 1.4), (0.0, 0.0, 1.4)]
    assert m.target_index == 0
    assert m.last_target_index == -1
    assert not m._arrival_window
    assert not m._vel_window
    assert m.arrival_confirmed_time is None
    assert m.arrival_start_time == 50.0
    assert m._cruise_arrival_count == 0
    assert m._active_segment_distance_m == pytest.approx(math.hypot(0.2, 0.2))
    assert m.x_pid.pid.components == (0, 0, 0)
    assert m.y_pid.pid.components == (0, 0, 0)
    assert m._navigation_purpose == "return"
    assert m._ramp_z_cm == 123.0
    assert generation == 1


def test_return_middle_waypoints_use_cruise_and_final_remains_precision():
    m = _make_mission(profile="precision")
    m.replace_navigation_targets(
        [(-2.65, 0.05, 1.4), (-2.65, 3.5, 1.4), (-2.5, 3.5, 1.4)],
        [-1.75, 0.05, 1.4],
        purpose="return",
    )

    assert m._waypoint_mode() == "cruise"
    m.target_index = 1
    assert m._waypoint_mode() == "cruise"
    m.target_index = 2
    assert m._waypoint_mode() == "precision"


def test_return_middle_waypoint_advances_at_a1_logged_position():
    m = _make_mission(profile="precision")
    m.navigation_profile = NavigationProfileConfig(
        profile="precision", cruise_confirm_cycles=3, cruise_require_z=False
    )
    m.replace_navigation_targets(
        [(-2.65, 0.0625, 1.4), (-2.65, 3.5, 1.4), (-2.5, 3.5, 1.4)],
        [-1.7609, 0.0623, 1.43],
        purpose="return",
    )
    m.realsense.vel = (0.5, 0.0, 0.0)

    for _ in range(3):
        m.navigate([-2.6369, 0.0703, 1.4], 0.0)

    assert m.target_index == 1
    assert m.state != "LAND"


@pytest.mark.parametrize(
    "position,confidence",
    [
        ([-2.49, 0.0, 1.4], 3),
        ([-2.65, 0.0, 1.19], 3),
        ([-2.64, 0.0, 1.4], 1),
    ],
)
def test_return_timeout_only_advances_when_safely_near(position, confidence):
    m = _make_mission(profile="precision")
    m.replace_navigation_targets(
        [(-2.65, 0.0, 1.4), (-2.5, 3.5, 1.4)],
        [-1.75, 0.0, 1.4],
        purpose="return",
    )
    m.realsense.confidence = confidence
    m.last_target_index = 0
    m.arrival_start_time = time.time() - 100.0

    m.navigate(position, 0.0)

    assert m.state == "LAND"
    assert m.target_index == 0


def test_return_timeout_near_advances_middle_waypoint():
    m = _make_mission(profile="precision")
    m.replace_navigation_targets(
        [(-2.65, 0.0, 1.4), (-2.5, 3.5, 1.4)],
        [-1.75, 0.0, 1.4],
        purpose="return",
    )
    m.last_target_index = 0
    m.arrival_start_time = time.time() - 100.0

    m.navigate([-2.64, 0.0, 1.4], 0.0)

    assert m.target_index == 1
    assert m.state != "LAND"


def test_return_final_timeout_lands_even_when_near():
    m = _make_mission(profile="precision")
    m.replace_navigation_targets(
        [(-2.5, 3.5, 1.4)], [-2.65, 3.5, 1.4], purpose="return"
    )
    m.last_target_index = 0
    m.arrival_start_time = time.time() - 100.0

    m.navigate([-2.49, 3.5, 1.4], 0.0)

    assert m.state == "LAND"
    assert m.target_index == 0


def test_return_waypoint_event_records_effective_mode_and_purpose():
    m = _make_mission(profile="precision")
    m.replace_navigation_targets(
        [(-2.65, 0.0, 1.4), (-2.5, 3.5, 1.4)],
        [-1.75, 0.0, 1.4],
        purpose="return",
    )
    m._log_file = io.StringIO()

    for _ in range(3):
        m.navigate([-2.64, 0.0, 1.4], 0.0)

    entries = [json.loads(line) for line in m._log_file.getvalue().splitlines()]
    event = [entry for entry in entries if entry.get("event") == "waypoint_advance"][-1]
    assert event["waypoint_mode"] == "cruise"
    assert event["navigation_purpose"] == "return"


def test_replace_navigation_targets_rejects_empty_route():
    m = _make_mission()

    with pytest.raises(ValueError, match="cannot be empty"):
        m.replace_navigation_targets([], [0.0, 0.0, 1.0])


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


def test_cruise_route_only_can_require_height_before_advancing():
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


def test_takeoff_aborts_safely_when_t265_confidence_is_lost(monkeypatch):
    m = _make_mission()
    m.realsense.confidence = 0
    monkeypatch.setattr(mg, "DRY_RUN", True)
    monkeypatch.setattr(mg, "TAKEOFF_CONFIDENCE_ABORT_S", 0.0)
    monkeypatch.setattr(mg.time, "sleep", lambda _seconds: None)

    m.takeoff()

    assert m.state == "LAND"
    assert m._takeoff_abort_reason == "t265_confidence_0"
    assert m.se_fc[2] == 0
    assert m.se_fc[5] == 0
    assert m.se_fc[3] == sp_side
    assert m.se_fc[4] == sp_side
    assert m.se_fc[6] == sp_side


def test_takeoff_never_unlocks_without_t265(monkeypatch):
    m = _make_mission()
    m.t265_ok = False
    m.realsense = None
    monkeypatch.setattr(mg, "DRY_RUN", False)

    m.takeoff()

    assert m.state == "LAND"
    assert m._takeoff_abort_reason == "t265_unavailable"
    assert m.se_fc[2] == 0


def test_takeoff_never_unlocks_when_preunlock_t265_read_fails(monkeypatch):
    class BrokenRealsense(FakeRealsense):
        def get_orientation(self):
            raise RuntimeError("camera stopped")

    m = _make_mission()
    m.realsense = BrokenRealsense(confidence=3)
    monkeypatch.setattr(mg, "DRY_RUN", False)

    m.takeoff()

    assert m.state == "LAND"
    assert m._takeoff_abort_reason == "t265_preunlock_read_error"
    assert m.se_fc[2] == 0


def test_takeoff_aborts_safely_when_liftoff_height_times_out(monkeypatch):
    m = _make_mission()
    monkeypatch.setattr(mg, "DRY_RUN", True)
    monkeypatch.setattr(mg, "TAKEOFF_TIMEOUT_S", 0.0)
    monkeypatch.setattr(mg.time, "sleep", lambda _seconds: None)

    m.takeoff()

    assert m.state == "LAND"
    assert m._takeoff_abort_reason == "liftoff_height_timeout"
    assert m.se_fc[2] == 0
    assert m.se_fc[5] == 0
    assert m.se_fc[3] == sp_side
    assert m.se_fc[4] == sp_side
    assert m.se_fc[6] == sp_side
