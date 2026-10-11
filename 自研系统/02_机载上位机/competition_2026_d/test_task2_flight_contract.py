"""任务二正式飞行适配层的持续解锁契约测试（不访问硬件）。"""

from collections import deque
from types import SimpleNamespace

from drone_control.competition_2026_d.task2_flight import (
    FC_DIRECT_LOCK_SIGN,
    Task2FlightMission,
)
from drone_control.competition_2026_d.task2_mission import Task2Phase
from drone_control.competition_2026_d.task2_mission import (
    Task2Config,
    Task2MissionDirector,
)
from drone_control.competition_2026_d.vision.platform_tracker import (
    PlatformTracker,
)


def _flight(*, task_sta=1, next_task_sign=0, unlock_sta=1, dry_run=False):
    flight = Task2FlightMission.__new__(Task2FlightMission)
    flight.se_fc = [170, 2, task_sta, 500, 500, 100, 500, next_task_sign]
    flight.re_fc = [0, 0, 0, 0, 0, unlock_sta]
    flight._ramp_z_cm = 10.0
    flight._dry_run = dry_run
    flight.state = "NAVIGATE"
    flight.last_speed = None
    flight._direct_lock_active = False
    flight._direct_lock_started_at = None
    flight._direct_lock_confirm_count = 0
    flight._direct_lock_timeout_logged = False
    flight._direct_lock_confirmed_at = None
    flight._direct_lock_reset_started_at = None
    flight._direct_lock_reset_confirm_count = 0
    flight._direct_lock_retakeoff_blocked = False
    flight._direct_lock_retakeoff_block_logged = False
    flight._direct_lock_laser_samples = deque()
    flight._direct_lock_zero_setpoint_since = None
    flight._t265_continuity_last_time = None
    flight._t265_continuity_last_position = None
    flight._t265_recovery_until = None
    flight._t265_recovery_used = False
    flight._t265_continuity_net_correction = (0.0, 0.0, 0.0)
    flight._landing_vz_applied_m_s = 0.0
    flight.serial_fc_ref = None
    flight.set_speed = lambda *args: setattr(flight, "last_speed", args)
    return flight


def _command(phase, *, keep_armed=True):
    return SimpleNamespace(phase=phase, keep_armed=keep_armed)


def test_deck_ride_keeps_original_task_active_without_mutating_command_bits():
    flight = _flight()
    before = tuple(flight.se_fc)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.DYNAMIC_LANDING)
    )
    assert tuple(flight.se_fc) == before
    assert flight.state == "NAVIGATE"


def test_retakeoff_is_allowed_only_while_flight_controller_remains_unlocked():
    flight = _flight(unlock_sta=1)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.RETAKEOFF)
    )
    assert flight.se_fc[2] == 1
    assert flight.se_fc[7] == 0
    assert flight.state == "NAVIGATE"


def test_retakeoff_refuses_second_unlock_when_flight_controller_is_locked():
    flight = _flight(unlock_sta=0)

    assert not flight._verify_continuous_arm_contract(
        _command(Task2Phase.RETAKEOFF)
    )
    assert flight.state == "HOVER_WAIT"
    assert flight.last_speed == (0, 0, 0, 10)
    assert flight.se_fc[2] == 1
    assert flight.se_fc[7] == 0


def test_unexpected_midmission_command_bit_change_stops_instead_of_rearming():
    flight = _flight(task_sta=0, next_task_sign=101, unlock_sta=0)

    assert not flight._verify_continuous_arm_contract(
        _command(Task2Phase.DYNAMIC_LANDING)
    )
    assert flight.state == "HOVER_WAIT"
    assert flight.last_speed == (0, 0, 0, 10)
    # 检查器只拒绝继续，不制造新的 task_sta 0->1 起飞边沿。
    assert flight.se_fc[2] == 0
    assert flight.se_fc[7] == 101


def test_contract_is_inactive_after_mission_completion():
    flight = _flight(task_sta=0, next_task_sign=101, unlock_sta=0)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.COMPLETE, keep_armed=False)
    )
    assert flight.state == "NAVIGATE"


def test_dry_run_keeps_task_zero_and_does_not_require_unlock_feedback():
    flight = _flight(task_sta=0, unlock_sta=0, dry_run=True)

    assert flight._verify_continuous_arm_contract(
        _command(Task2Phase.RETAKEOFF)
    )
    assert flight.state == "NAVIGATE"
    assert flight.se_fc[2] == 0
    assert flight.se_fc[7] == 0


def test_fc_one_key_land_handoff_uses_existing_flight_controller_format():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            fc_one_key_land_enabled=True,
            fc_one_key_land_height_m=0.10,
        )
    )
    flight.heading_hold = SimpleNamespace(disarm=lambda reason: None)
    command = SimpleNamespace(
        phase=Task2Phase.DYNAMIC_LANDING,
        landing_active=True,
    )

    assert not flight._should_handoff_to_fc_one_key_land(command, 0.11)
    assert flight._should_handoff_to_fc_one_key_land(command, 0.10)

    flight._begin_fc_one_key_land(0.10)
    assert flight.state == "LAND"
    assert flight.se_fc[2] == 0
    assert flight.se_fc[7] == 101
    assert flight.last_speed == (0, 0, 0, 0)


def test_fc_direct_lock_requires_sustained_zero_height_target():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            fc_one_key_land_enabled=False,
            fc_direct_lock_enabled=True,
            fc_direct_lock_height_m=0.05,
        )
    )
    flight.heading_hold = SimpleNamespace(disarm=lambda reason: None)
    command = SimpleNamespace(
        phase=Task2Phase.DYNAMIC_LANDING,
        landing_active=True,
    )

    # A low laser reading alone may still be a 5 cm hover equilibrium.
    assert not flight._should_begin_fc_direct_lock(command, 0.05, True, 0.1)

    flight._ramp_z_cm = 0.0
    flight._direct_lock_zero_setpoint_since = 0.0
    for index in range(8):
        assert not flight._should_begin_fc_direct_lock(
            command, 0.058 + (index % 2) * 0.002, True, index * 0.10
        )
    assert flight._should_begin_fc_direct_lock(command, 0.059, True, 0.80)

    flight._begin_fc_direct_lock(0.10)
    assert flight.state == "LAND"
    assert flight.se_fc[2] == 1
    assert flight.se_fc[7] == FC_DIRECT_LOCK_SIGN
    assert flight.se_fc[7] != 101
    assert flight.last_speed == (0, 0, 0, 0)


def test_fc_direct_lock_accepts_stable_sub_ten_cm_laser_fallback():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            fc_direct_lock_enabled=True,
            fc_direct_lock_height_m=0.05,
            fc_direct_lock_stable_height_m=0.10,
            fc_direct_lock_stable_hold_s=0.80,
            fc_direct_lock_stable_tolerance_m=0.01,
            fc_direct_lock_stable_min_samples=8,
        )
    )
    command = SimpleNamespace(
        phase=Task2Phase.DYNAMIC_LANDING,
        landing_active=True,
    )
    flight._ramp_z_cm = 0.0
    flight._direct_lock_zero_setpoint_since = 0.0

    for index in range(8):
        assert not flight._should_begin_fc_direct_lock(
            command,
            0.078 + (index % 2) * 0.002,
            True,
            index * 0.10,
        )
    assert flight._should_begin_fc_direct_lock(
        command, 0.079, True, 0.80
    )


def test_fc_direct_lock_requires_explicit_unlock_and_zero_pwm_feedback():
    flight = _flight(unlock_sta=0)
    flight.director = Task2MissionDirector(Task2Config())
    flight.heading_hold = SimpleNamespace(disarm=lambda reason: None)
    flight._direct_lock_active = True
    flight._direct_lock_started_at = 10**12
    flight.serial_fc_ref = SimpleNamespace(debug_data={})
    stopped = []
    flight.stop_all = lambda: stopped.append(True)

    for _ in range(10):
        flight.land()
    assert not stopped

    flight.serial_fc_ref.debug_data["motor_pwm_mask"] = 0
    for _ in range(4):
        flight.land()
    assert not stopped
    flight.land()
    assert stopped == [True]
    assert flight.director.mission_success


def test_direct_lock_holds_then_resets_task_and_enters_second_takeoff():
    flight = _flight(unlock_sta=0)
    flight.re_fc[0] = 0
    flight.director = Task2MissionDirector(
        Task2Config(
            platform_retakeoff_enabled=True,
            platform_locked_hold_s=0.0,
            platform_task_reset_hold_s=0.0,
        )
    )
    flight.director._transition(Task2Phase.DYNAMIC_LANDING, 0.0, "test")
    flight._direct_lock_active = True
    flight._direct_lock_started_at = 10**12
    flight.serial_fc_ref = SimpleNamespace(
        debug_data={"motor_pwm_mask": 0}
    )

    class T265Probe:
        start_calls = 0
        autoset_calls = 0

        def is_running(self):
            return True

        def get_tracking_confidence(self):
            return 3

        def get_pose_age_s(self):
            return 0.01

        def get_position(self):
            return (1.25, 0.75, 0.10)

        def start(self):
            self.start_calls += 1

        def autoset(self):
            self.autoset_calls += 1

    flight.realsense = T265Probe()
    resets = []
    flight.laser_contact = SimpleNamespace(reset=lambda: resets.append(True))
    flight._landing_gate_passed = True
    flight._touchdown_confirmed = True
    flight._deck_ride_complete = False
    flight._landing_aborted = False
    flight._tracker_active_prev = True
    flight._last_nav_time = 1.0

    for _ in range(5):
        flight.land()

    assert flight._direct_lock_reset_started_at is not None
    assert flight.se_fc[2] == 0
    assert flight.se_fc[7] == 0

    for _ in range(3):
        flight.land()

    assert flight.state == "TAKEOFF"
    assert not flight._direct_lock_active
    assert flight.director.phase == Task2Phase.RETAKEOFF
    assert flight.director._retakeoff_anchor == (1.25, 0.75)
    assert flight.se_fc[2] == 0
    assert flight.se_fc[7] == 0
    assert resets == [True]
    assert flight.realsense.start_calls == 0
    assert flight.realsense.autoset_calls == 0


def test_stationary_platform_estimate_uses_vision_then_freezes_last_position():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            stationary_retakeoff_test=True,
            stationary_test_point_x_m=0.0,
            stationary_test_point_y_m=0.50,
        )
    )
    flight.platform_tracker = PlatformTracker()
    flight._stationary_platform_measurement = None

    visible = SimpleNamespace(
        found=True,
        error_xy_m=(0.05, -0.02),
        quality=90,
    )
    estimate = flight._update_stationary_platform_estimate(
        gate=visible,
        world_position=(0.10, 0.40, 1.0),
        now=1.0,
    )
    assert estimate is not None
    assert abs(estimate.x_m - 0.15) < 1e-9
    assert abs(estimate.y_m - 0.38) < 1e-9

    lost_too_close = SimpleNamespace(
        found=False,
        error_xy_m=None,
        quality=0,
    )
    frozen = flight._update_stationary_platform_estimate(
        gate=lost_too_close,
        world_position=(0.15, 0.38, 0.10),
        now=1.1,
    )
    assert frozen is not None
    assert abs(flight._stationary_platform_measurement[0] - 0.15) < 1e-9
    assert abs(flight._stationary_platform_measurement[1] - 0.38) < 1e-9


def test_stationary_no_vision_landing_uses_fixed_point_as_fresh_gate():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            stationary_retakeoff_test=True,
            stationary_skip_vision=True,
        )
    )

    class LandingProbe:
        def __init__(self):
            self.input = None

        def tick(self, landing_input):
            self.input = landing_input
            return SimpleNamespace()

    flight.dynamic_landing = LandingProbe()
    estimate = SimpleNamespace(
        x_m=0.0,
        y_m=0.50,
        vx_m_s=0.0,
        vy_m_s=0.0,
        uncertainty_m=0.01,
    )
    gate = SimpleNamespace(found=False, ambiguous=False)

    flight._run_landing(
        estimate,
        world_position=(0.0, 0.50, 1.0),
        velocity=(0.0, 0.0, 0.0),
        laser_height=1.0,
        gate=gate,
        observation=None,
        contact_evidence=False,
        safety_fault=None,
        car_velocity=None,
    )

    assert flight.dynamic_landing.input.visual_usable
    assert flight.dynamic_landing.input.car_motion_fresh


def test_landing_height_integrates_negative_speed_downward():
    flight = _flight()
    flight._ramp_z_cm = 100.0

    flight._step_landing_height(-0.20, 0.10)
    assert flight._ramp_z_cm == 98.0

    flight._step_landing_height(0.12, 0.10)
    assert flight._ramp_z_cm == 99.2

    flight._ramp_z_cm = 1.0
    flight._step_landing_height(-0.30, 0.10)
    assert flight._ramp_z_cm == 0.0


def test_retakeoff_and_return_h_t265_jump_is_absorbed_once():
    for phase in (Task2Phase.RETAKEOFF, Task2Phase.RETURN_H):
        flight = _flight()
        flight.director = Task2MissionDirector(Task2Config())
        flight.director._transition(phase, 0.0, "test")

        class T265Probe:
            def __init__(self):
                self.corrections = []

            def apply_position_continuity_correction(self, delta):
                self.corrections.append(tuple(delta))

        flight.realsense = T265Probe()
        first = flight._preserve_retakeoff_t265_continuity(
            now=1.0,
            world_position=(1.90, 1.70, 1.20),
            velocity=(0.0, 0.0, 0.30),
        )
        corrected = flight._preserve_retakeoff_t265_continuity(
            now=1.03,
            world_position=(2.24, 1.70, 1.21),
            velocity=(0.0, 0.0, 0.30),
        )

        assert first == (1.90, 1.70, 1.20)
        assert corrected[:2] == (1.90, 1.70)
        assert abs(corrected[2] - 1.209) < 1e-9
        assert len(flight.realsense.corrections) == 1
        assert flight._t265_recovery_used


def test_second_independent_t265_jump_is_not_hidden():
    flight = _flight()
    flight.director = Task2MissionDirector(Task2Config())
    flight.director._transition(Task2Phase.RETAKEOFF, 0.0, "test")
    flight._t265_recovery_used = True
    flight._t265_recovery_until = None
    flight._t265_continuity_last_time = 1.0
    flight._t265_continuity_last_position = (1.0, 1.0, 1.0)
    flight.realsense = SimpleNamespace(
        apply_position_continuity_correction=lambda delta: None
    )

    uncorrected = flight._preserve_retakeoff_t265_continuity(
        now=1.03,
        world_position=(1.40, 1.0, 1.0),
        velocity=(0.0, 0.0, 0.0),
    )

    assert uncorrected == (1.40, 1.0, 1.0)


def test_landing_descent_speed_slews_but_stop_is_immediate():
    flight = _flight()
    flight.dynamic_landing = SimpleNamespace(
        config=SimpleNamespace(descend_slew_m_s2=0.40)
    )
    flight._landing_vz_applied_m_s = 0.0

    first = flight._smooth_landing_vz(-0.30, 0.05)
    assert abs(first - -0.02) < 1e-9
    flight._landing_vz_applied_m_s = first
    second = flight._smooth_landing_vz(-0.30, 0.05)
    assert abs(second - -0.04) < 1e-9

    # Contact/gate hold must stop descent without a lingering ramp.
    flight._landing_vz_applied_m_s = second
    assert flight._smooth_landing_vz(0.0, 0.05) == 0.0


def test_visual_platform_measurement_updates_once_per_camera_frame():
    flight = _flight()
    flight.platform_tracker = PlatformTracker()
    flight._last_platform_vision_seq = None
    flight._platform_measurement_source = "none"
    gate = SimpleNamespace(
        found=True,
        error_xy_m=(0.05, -0.03),
        seq=10,
        quality=90,
    )

    first = flight._update_visual_platform_estimate(
        gate=gate,
        world_position=(1.0, 2.0, 1.0),
        now=1.0,
    )
    assert first is not None
    assert abs(first.x_m - 1.05) < 1e-9
    assert abs(first.y_m - 1.97) < 1e-9
    assert flight._platform_measurement_source == "visual"

    repeated = flight._update_visual_platform_estimate(
        gate=gate,
        world_position=(1.0, 2.0, 1.0),
        now=1.05,
    )
    assert repeated is not None
    assert repeated.predicted
    assert flight._platform_measurement_source == "visual_prediction"


def test_visual_landing_xy_speed_limit_reduces_near_ground():
    flight = _flight()
    flight.director = Task2MissionDirector(
        Task2Config(
            landing_xy_speed_high_m_s=0.15,
            landing_xy_speed_mid_m_s=0.10,
            landing_xy_speed_low_m_s=0.07,
        )
    )
    flight.dynamic_landing = SimpleNamespace(
        config=SimpleNamespace(mid_height_m=0.80, low_height_m=0.35)
    )

    assert flight._landing_xy_speed_limit(1.0) == 0.15
    assert flight._landing_xy_speed_limit(0.50) == 0.10
    assert flight._landing_xy_speed_limit(0.20) == 0.07
    vx, vy = flight._limit_xy_speed(0.30, 0.40, 0.10)
    assert abs((vx * vx + vy * vy) ** 0.5 - 0.10) < 1e-9
