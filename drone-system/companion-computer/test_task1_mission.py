import math

from drone_control.competition_2026_d.control.task1_path_controller import (
    PolylinePath,
    Task1PathFollower,
)
from drone_control.competition_2026_d.payload_actuator import ActuatorState
from drone_control.competition_2026_d.task1_mission import (
    B,
    B_PRE,
    C,
    D,
    Task1Config,
    Task1Input,
    Task1MissionDirector,
    Task1Phase,
)


def mission_input(now, position, **kwargs):
    return Task1Input(now=now, position_xyz_m=position, **kwargs)


def test_task1_b_pre_is_left_shifted_and_keeps_forward_margin():
    assert B_PRE == (0.275, 1.90)
    assert math.isclose(B[1] - B_PRE[1], 0.35, abs_tol=1e-9)


def test_polyline_projection_does_not_regress_progress():
    path = PolylinePath(((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)))
    follower = Task1PathFollower(path)
    follower.reset(progress_m=0.8, timestamp=0.0)
    command = follower.command(
        (0.2, 0.1), nominal_speed_m_s=0.13, timestamp=0.1
    )
    assert command.progress_m >= 0.77
    assert math.hypot(command.vx_m_s, command.vy_m_s) <= 0.20 + 1e-9


def test_path_reset_clamps_inherited_intercept_velocity():
    path = PolylinePath(((0.0, 0.0), (1.0, 0.0)))
    follower = Task1PathFollower(path)
    follower.reset(
        timestamp=0.0, velocity_xy_m_s=(0.38, 0.0)
    )
    command = follower.command(
        (0.0, 0.0), nominal_speed_m_s=0.13, timestamp=0.1
    )
    assert math.hypot(command.vx_m_s, command.vy_m_s) <= 0.20 + 1e-9


def test_hold_uses_fixed_two_seconds_despite_velocity_noise():
    director = Task1MissionDirector()
    director._transition(Task1Phase.HOLD_3S, 0.0, "test")
    director.tick(
        mission_input(0.0, (0.0, 0.0, 1.5), velocity_xy_m_s=(0.2, 0.0))
    )
    director.tick(
        mission_input(1.9, (0.0, 0.0, 1.5), velocity_xy_m_s=(0.2, 0.0))
    )
    assert director.phase == Task1Phase.HOLD_3S
    director.tick(
        mission_input(2.01, (0.0, 0.0, 1.5), velocity_xy_m_s=(0.2, 0.0))
    )
    assert director.phase == Task1Phase.INTERCEPT_B_PRE


def test_hold_waits_for_safe_height_without_restarting_timer():
    director = Task1MissionDirector()
    director._transition(Task1Phase.HOLD_3S, 0.0, "test")
    director.tick(mission_input(2.1, (0.0, 0.0, 1.3)))
    assert director.phase == Task1Phase.HOLD_3S
    director.tick(mission_input(2.2, (0.0, 0.0, 1.4)))
    assert director.phase == Task1Phase.INTERCEPT_B_PRE


def test_takeoff_and_hold_use_t265_position_feedback():
    director = Task1MissionDirector()
    director._transition(Task1Phase.TAKEOFF, 0.0, "test")
    takeoff = director.tick(mission_input(0.1, (0.20, -0.10, 0.8)))
    assert takeoff.vx_m_s < 0.0
    assert takeoff.vy_m_s > 0.0
    assert math.hypot(takeoff.vx_m_s, takeoff.vy_m_s) <= 0.15 + 1e-9

    director._transition(Task1Phase.HOLD_3S, 1.0, "test")
    hold = director.tick(mission_input(1.1, (-0.12, 0.08, 1.5)))
    assert hold.vx_m_s > 0.0
    assert hold.vy_m_s < 0.0


def test_b_pre_waits_indefinitely_for_current_vision_by_default():
    director = Task1MissionDirector()
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")
    command = director.tick(mission_input(4.1, (*B_PRE, 1.5)))
    assert command.phase == Task1Phase.ACQUIRE_TARGET
    command = director.tick(mission_input(30.0, (*B_PRE, 1.5)))
    assert command.phase == Task1Phase.ACQUIRE_TARGET
    assert not command.target_acquired


def test_b_pre_timeout_fallback_remains_available_when_explicitly_enabled():
    director = Task1MissionDirector(Task1Config(acquire_timeout_s=4.0))
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")
    command = director.tick(mission_input(4.1, (*B_PRE, 1.5)))
    assert command.phase == Task1Phase.FOLLOW_B_C
    command = director.tick(mission_input(4.2, (*B_PRE, 1.5)))
    assert command.target_world_height_m == 1.5
    assert not command.target_acquired


def test_path_only_holds_b_pre_until_t265_height_reaches_one_meter():
    config = Task1Config(path_only_b_pre_descent=True)
    director = Task1MissionDirector(config)
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")

    descending = director.tick(mission_input(0.1, (*B_PRE, 1.35)))
    assert descending.phase == Task1Phase.ACQUIRE_TARGET
    assert descending.target_world_height_m == 1.0
    assert (descending.vx_m_s, descending.vy_m_s) == (0.0, 0.0)

    ready = director.tick(mission_input(1.0, (*B_PRE, 1.08)))
    assert ready.phase == Task1Phase.FOLLOW_B_C
    assert ready.target_acquired


def test_b_pre_wait_anchor_is_not_armed_while_aircraft_is_still_fast():
    director = Task1MissionDirector(
        Task1Config(hold_stable_speed_m_s=0.12)
    )
    director._transition(Task1Phase.INTERCEPT_B_PRE, 0.0, "test")

    fast = director.tick(
        mission_input(
            0.1,
            (*B_PRE, 1.5),
            velocity_xy_m_s=(0.0, 0.28),
        )
    )
    assert fast.phase == Task1Phase.INTERCEPT_B_PRE

    stable = director.tick(
        mission_input(
            0.2,
            (*B_PRE, 1.5),
            velocity_xy_m_s=(0.03, 0.04),
        )
    )
    assert stable.phase == Task1Phase.ACQUIRE_TARGET


def test_intercept_ignores_edge_detection_and_requires_centered_streak():
    director = Task1MissionDirector(
        Task1Config(
            vision_confirm_frames=2,
            intercept_vision_confirm_frames=5,
            intercept_vision_max_error_m=0.35,
            drop_max_error_m=0.30,
        )
    )
    director._transition(Task1Phase.INTERCEPT_B_PRE, 0.0, "test")

    edge = director.tick(
        mission_input(
            0.1,
            (B_PRE[0], B_PRE[1] - 0.30, 1.5),
            velocity_xy_m_s=(0.0, 0.25),
            vision_seq=1,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(-0.40, 0.20),
        )
    )
    assert edge.phase == Task1Phase.INTERCEPT_B_PRE
    assert edge.vy_m_s > 0.0

    for seq in range(2, 6):
        approach = director.tick(
            mission_input(
                seq * 0.1,
                (B_PRE[0], B_PRE[1] - 0.30, 1.5),
                velocity_xy_m_s=(0.0, 0.25),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(-0.20, 0.10),
            )
        )
        assert approach.phase == Task1Phase.INTERCEPT_B_PRE

    approach = director.tick(
        mission_input(
            0.6,
            (B_PRE[0], B_PRE[1] - 0.30, 1.5),
            velocity_xy_m_s=(0.0, 0.25),
            vision_seq=6,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(-0.20, 0.10),
        )
    )
    assert approach.phase == Task1Phase.ACQUIRE_TARGET
    assert (approach.vx_m_s, approach.vy_m_s) == (0.0, 0.0)
    assert approach.reason == "target_centered_during_intercept"

    tracking = director.tick(
        mission_input(
            0.7,
            (B_PRE[0], B_PRE[1] - 0.30, 1.5),
            vision_seq=7,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(-0.20, 0.10),
        )
    )
    assert tracking.vx_m_s < 0.0
    assert tracking.vy_m_s > 0.0
    assert math.hypot(tracking.vx_m_s, tracking.vy_m_s) <= 0.12 + 1e-9
    assert (tracking.base_vx_m_s, tracking.base_vy_m_s) == (0.0, 0.0)


def test_b_pre_gate_requires_current_centered_vision():
    director = Task1MissionDirector(
        Task1Config(vision_confirm_frames=2, drop_max_error_m=0.30)
    )
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")

    for seq in (1, 2):
        off_center = director.tick(
            mission_input(
                seq * 0.1,
                (*B_PRE, 1.5),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.31, 0.0),
            )
        )
    assert off_center.phase == Task1Phase.ACQUIRE_TARGET

    for seq in (3, 4):
        centered = director.tick(
            mission_input(
                seq * 0.1,
                (*B_PRE, 1.5),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.20, 0.10),
            )
        )
    assert centered.phase == Task1Phase.FOLLOW_B_C
    assert centered.target_acquired


def test_vision_track_only_stays_in_visual_servo_after_centering():
    director = Task1MissionDirector(
        Task1Config(
            vision_track_only=True,
            payload_drop_enabled=False,
            follow_height_m=1.5,
            vision_confirm_frames=2,
            acquire_max_error_m=0.30,
        )
    )
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")

    for seq in (1, 2, 3):
        command = director.tick(
            mission_input(
                seq * 0.2,
                (*B_PRE, 1.5),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.05, 0.02),
            )
        )
    assert command.phase == Task1Phase.ACQUIRE_TARGET
    assert command.target_acquired
    assert command.target_world_height_m == 1.5

    moving = director.tick(
        mission_input(
            0.8,
            (*B_PRE, 1.5),
            vision_seq=4,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.35, -0.15),
        )
    )
    assert moving.phase == Task1Phase.ACQUIRE_TARGET
    assert moving.vx_m_s > 0.0
    assert math.hypot(moving.vx_m_s, moving.vy_m_s) <= 0.12 + 1e-9


def test_pure_vision_waits_for_intercept_velocity_to_slow_before_tracking():
    director = Task1MissionDirector(
        Task1Config(
            vision_track_only=True,
            payload_drop_enabled=False,
            vision_takeover_max_speed_m_s=0.08,
        )
    )
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")

    braking = director.tick(
        mission_input(
            0.1,
            (*B_PRE, 1.5),
            velocity_xy_m_s=(0.0, 0.22),
            vision_seq=1,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.30, 0.0),
        )
    )
    assert braking.phase == Task1Phase.ACQUIRE_TARGET
    assert braking.reason == "pure_vision_takeover_braking"
    assert (braking.vx_m_s, braking.vy_m_s) == (0.0, 0.0)
    assert (braking.vision_trim_vx_m_s, braking.vision_trim_vy_m_s) == (
        0.0,
        0.0,
    )

    tracking = director.tick(
        mission_input(
            0.3,
            (*B_PRE, 1.5),
            velocity_xy_m_s=(0.0, 0.07),
            vision_seq=2,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.30, 0.0),
        )
    )
    assert tracking.reason == "pure_vision_takeover_ready"
    assert tracking.vx_m_s > 0.0
    assert tracking.base_vx_m_s == 0.0
    assert tracking.base_vy_m_s == 0.0


def test_pure_vision_drop_never_enters_b_c_path_and_releases_after_gate():
    director = Task1MissionDirector(
        Task1Config(
            vision_track_only=True,
            payload_drop_enabled=True,
            follow_height_m=1.5,
            vision_confirm_frames=2,
            drop_max_error_m=0.20,
            drop_confirm_duration_s=3.0,
        )
    )
    director._acquire_anchor = (0.20, 1.70)
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")

    for seq, now in enumerate((0.10, 1.10, 2.10), start=1):
        tracking = director.tick(
            mission_input(
                now,
                (0.20, 1.70, 1.5),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.12, -0.04),
            )
        )
        assert tracking.phase == Task1Phase.ACQUIRE_TARGET
        assert tracking.base_vx_m_s == 0.0
        assert tracking.base_vy_m_s == 0.0
        assert tracking.vx_m_s == tracking.vision_trim_vx_m_s
        assert tracking.vy_m_s == tracking.vision_trim_vy_m_s

    committed = director.tick(
        mission_input(
            3.11,
            (0.20, 1.70, 1.5),
            vision_seq=4,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.10, -0.03),
        )
    )
    assert committed.phase == Task1Phase.RELEASING
    assert committed.reason == "drop_gate_latched_on_pure_vision"
    assert committed.drop_committed
    assert committed.release_requested
    assert committed.target_world_height_m == 1.5
    assert committed.base_vx_m_s == 0.0
    assert committed.base_vy_m_s == 0.0

    releasing = director.tick(
        mission_input(
            3.31,
            (0.21, 1.69, 1.5),
            vision_seq=5,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(-0.10, 0.05),
            payload_state=ActuatorState.LOCKED,
        )
    )
    assert releasing.phase == Task1Phase.RELEASING
    assert releasing.target_xy_m == (0.20, 1.70)
    assert releasing.base_vx_m_s == 0.0
    assert releasing.base_vy_m_s == 0.0

    returning = director.tick(
        mission_input(
            3.40,
            (0.21, 1.69, 1.5),
            vision_seq=6,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(-0.08, 0.04),
            payload_state=ActuatorState.RELEASED,
        )
    )
    assert returning.phase == Task1Phase.CLIMB
    assert returning.drop_released
    assert returning.mission_success


def test_pure_vision_drop_gate_requires_follow_height_for_full_duration():
    director = Task1MissionDirector(
        Task1Config(
            vision_track_only=True,
            payload_drop_enabled=True,
            follow_height_m=1.5,
            vision_confirm_frames=2,
            drop_max_error_m=0.20,
            drop_confirm_duration_s=3.0,
        )
    )
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")

    for seq, now in enumerate((0.1, 1.1, 2.1, 3.2), start=1):
        wrong_height = director.tick(
            mission_input(
                now,
                (*B_PRE, 1.25),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.05, 0.02),
            )
        )
    assert wrong_height.phase == Task1Phase.ACQUIRE_TARGET
    assert not wrong_height.release_requested

    for seq, now in enumerate((3.3, 4.3, 5.3), start=5):
        correct_height = director.tick(
            mission_input(
                now,
                (*B_PRE, 1.5),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.05, 0.02),
            )
        )
        assert correct_height.phase == Task1Phase.ACQUIRE_TARGET

    release = director.tick(
        mission_input(
            6.31,
            (*B_PRE, 1.5),
            vision_seq=8,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.05, 0.02),
        )
    )
    assert release.phase == Task1Phase.RELEASING
    assert release.release_requested


def test_acquire_visual_servo_filters_frames_and_updates_at_five_hz():
    director = Task1MissionDirector(
        Task1Config(
            acquire_vision_min_quality=40,
            acquire_vision_control_period_s=0.20,
            acquire_vision_filter_window_s=0.60,
        )
    )
    director._transition(Task1Phase.ACQUIRE_TARGET, 0.0, "test")

    first = director.tick(
        mission_input(
            0.00,
            (*B_PRE, 1.5),
            vision_seq=1,
            vision_found=True,
            vision_quality=40,
            vision_error_xy_m=(0.40, 0.0),
        )
    )
    noisy = director.tick(
        mission_input(
            0.05,
            (*B_PRE, 1.5),
            vision_seq=2,
            vision_found=True,
            vision_quality=40,
            vision_error_xy_m=(-0.40, 0.0),
        )
    )
    assert noisy.vx_m_s == first.vx_m_s

    trend = director.tick(
        mission_input(
            0.21,
            (*B_PRE, 1.5),
            vision_seq=3,
            vision_found=True,
            vision_quality=40,
            vision_error_xy_m=(0.35, 0.0),
        )
    )
    assert trend.vx_m_s > 0.0
    assert trend.vy_m_s == 0.0


def test_curve_and_straight_segments_can_use_different_speeds():
    director = Task1MissionDirector(
        Task1Config(car_speed_m_s=0.08, curve_speed_m_s=0.06)
    )
    director.target_acquired = True
    director._transition(Task1Phase.FOLLOW_B_C, 0.0, "test")
    director.bc_follower.reset(timestamp=0.0)
    curve = director.tick(mission_input(1.0, (*B, 1.0)))
    assert math.isclose(
        math.hypot(curve.vx_m_s, curve.vy_m_s), 0.06, abs_tol=1e-9
    )

    director._transition(Task1Phase.DROP_WINDOW_C_D, 2.0, "test")
    director.cd_follower.reset(timestamp=2.0)
    director.tick(mission_input(3.0, (*C, 1.0)))
    straight = director.tick(mission_input(3.2, (*C, 1.0)))
    assert math.isclose(
        math.hypot(straight.vx_m_s, straight.vy_m_s), 0.08, abs_tol=1e-9
    )


def test_c_sync_uses_limited_trim_then_releases_at_follow_height_as_fallback():
    director = Task1MissionDirector(
        Task1Config(
            drop_max_error_m=0.30,
            vision_confirm_frames=2,
            drop_confirm_duration_s=0.40,
        )
    )
    director.target_acquired = True
    director._transition(Task1Phase.SYNC_TARGET_AT_C, 0.0, "test")

    off_center = director.tick(
        mission_input(
            0.1,
            (*C, 1.0),
            vision_seq=1,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.31, 0.0),
        )
    )
    assert off_center.phase == Task1Phase.SYNC_TARGET_AT_C
    assert 0.0 < off_center.vx_m_s <= 0.03
    assert off_center.vision_trim_vx_m_s == off_center.vx_m_s

    first = director.tick(
        mission_input(
            0.20,
            (*C, 1.0),
            vision_seq=2,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.20, 0.10),
        )
    )
    assert first.phase == Task1Phase.SYNC_TARGET_AT_C

    second = director.tick(
        mission_input(
            0.35,
            (*C, 1.0),
            vision_seq=3,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.18, 0.08),
        )
    )
    assert second.phase == Task1Phase.SYNC_TARGET_AT_C

    fallback = director.tick(
        mission_input(
            0.61,
            (*C, 1.0),
            vision_seq=4,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.10, 0.05),
        )
    )
    assert fallback.phase == Task1Phase.RELEASING
    assert fallback.drop_committed
    assert fallback.target_deck_height_m is None
    assert fallback.reason == "c_fallback_drop_gate_latched_at_follow_height"


def test_b_c_vision_trim_is_additive_and_strictly_limited():
    without_vision = Task1MissionDirector()
    with_vision = Task1MissionDirector()
    for director in (without_vision, with_vision):
        director._transition(Task1Phase.FOLLOW_B_C, 0.0, "test")
        director.bc_follower.reset(timestamp=0.0)
    with_vision.target_acquired = True

    plain = without_vision.tick(mission_input(0.1, (*B_PRE, 1.5)))
    detected = with_vision.tick(
        mission_input(
            0.1,
            (*B_PRE, 1.5),
            vision_seq=1,
            vision_found=True,
            vision_quality=100,
            vision_error_xy_m=(0.30, -0.20),
        )
    )
    assert detected.base_vx_m_s == plain.vx_m_s
    assert detected.base_vy_m_s == plain.vy_m_s
    assert (detected.vx_m_s, detected.vy_m_s) != (
        plain.vx_m_s,
        plain.vy_m_s,
    )
    assert math.hypot(
        detected.vision_trim_vx_m_s,
        detected.vision_trim_vy_m_s,
    ) <= 0.03 + 1e-9
    assert plain.target_world_height_m == 1.5
    assert detected.target_world_height_m == 1.0


def test_b_c_centered_target_releases_without_drop_descent():
    director = Task1MissionDirector(
        Task1Config(
            vision_confirm_frames=2,
            drop_confirm_duration_s=0.40,
            drop_max_error_m=0.15,
        )
    )
    director.target_acquired = True
    director._transition(Task1Phase.FOLLOW_B_C, 0.0, "test")
    director.bc_follower.reset(timestamp=0.0)

    for now, seq in ((0.10, 10), (0.30, 11)):
        command = director.tick(
            mission_input(
                now,
                (*B, 1.0),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.05, 0.02),
            )
        )
        assert command.phase == Task1Phase.FOLLOW_B_C

    committed = director.tick(
        mission_input(
            0.51,
            (*B, 1.0),
            vision_seq=12,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.04, 0.02),
        )
    )
    assert committed.phase == Task1Phase.RELEASING
    assert committed.drop_committed
    assert committed.release_requested
    assert committed.target_world_height_m == 1.0
    assert committed.target_deck_height_m is None
    assert committed.reason == "drop_gate_latched_on_b_c_at_follow_height"

    releasing = director.tick(
        mission_input(0.52, (*B, 1.0), payload_state=ActuatorState.LOCKED)
    )
    assert releasing.release_requested
    assert releasing.target_deck_height_m is None

    climb = director.tick(
        mission_input(0.60, (*B, 1.0), payload_state=ActuatorState.RELEASED)
    )
    assert climb.phase == Task1Phase.CLIMB
    assert climb.mission_success


def test_path_joint_mode_waits_at_b_pre_and_flies_before_drop():
    director = Task1MissionDirector(
        Task1Config(
            cruise_height_m=1.5,
            follow_height_m=1.5,
            intercept_vision_early_stop_enabled=False,
            vision_confirm_frames=2,
            drop_confirm_duration_s=0.20,
            min_follow_before_drop_s=3.0,
            drop_max_error_m=0.30,
        )
    )
    director._transition(Task1Phase.INTERCEPT_B_PRE, 0.0, "test")

    # 即使途中已经居中识别，也必须继续飞到B_PRE。
    for now, seq in ((0.1, 1), (0.2, 2), (0.3, 3)):
        command = director.tick(
            mission_input(
                now,
                (0.0, 1.0, 1.5),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.01, 0.01),
            )
        )
    assert command.phase == Task1Phase.INTERCEPT_B_PRE

    director.tick(
        mission_input(
            1.0,
            (*B_PRE, 1.5),
            velocity_xy_m_s=(0.0, 0.0),
        )
    )
    assert director.phase == Task1Phase.ACQUIRE_TARGET

    for now, seq in ((1.1, 10), (1.2, 11)):
        command = director.tick(
            mission_input(
                now,
                (*B_PRE, 1.5),
                vision_seq=seq,
                vision_found=True,
                vision_quality=90,
                vision_error_xy_m=(0.02, 0.01),
            )
        )
    assert command.phase == Task1Phase.FOLLOW_B_C

    # FOLLOW_B_C开始后的前三秒不能投放。
    before = director.tick(
        mission_input(
            4.0,
            (*B, 1.5),
            vision_seq=20,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.04, 0.02),
        )
    )
    assert before.phase == Task1Phase.FOLLOW_B_C
    assert not before.release_requested

    first = director.tick(
        mission_input(
            4.21,
            (*B, 1.5),
            vision_seq=21,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.04, 0.02),
        )
    )
    assert first.phase == Task1Phase.FOLLOW_B_C
    released = director.tick(
        mission_input(
            4.42,
            (*B, 1.5),
            vision_seq=22,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.04, 0.02),
        )
    )
    assert released.phase == Task1Phase.RELEASING
    assert released.release_requested


def test_b_c_drop_gate_cannot_release_above_follow_height():
    director = Task1MissionDirector(
        Task1Config(
            vision_confirm_frames=2,
            drop_confirm_duration_s=0.40,
            drop_max_error_m=0.15,
        )
    )
    director.target_acquired = True
    director._transition(Task1Phase.FOLLOW_B_C, 0.0, "test")
    director.bc_follower.reset(timestamp=0.0)

    for now, seq in ((0.10, 1), (0.30, 2), (0.51, 3)):
        command = director.tick(
            mission_input(
                now,
                (*B, 1.20),
                vision_seq=seq,
                vision_found=True,
                vision_quality=95,
                vision_error_xy_m=(0.02, 0.01),
            )
        )
    assert command.phase == Task1Phase.FOLLOW_B_C
    assert not command.drop_committed
    assert not command.release_requested


def test_vision_trim_is_zero_inside_deadband_and_decays_on_loss():
    director = Task1MissionDirector()
    director.target_acquired = True
    director._transition(Task1Phase.FOLLOW_B_C, 0.0, "test")
    director.bc_follower.reset(timestamp=0.0)

    inside = director.tick(
        mission_input(
            0.10,
            (*B_PRE, 1.0),
            vision_seq=1,
            vision_found=True,
            vision_quality=95,
            vision_error_xy_m=(0.02, 0.01),
        )
    )
    assert (inside.vision_trim_vx_m_s, inside.vision_trim_vy_m_s) == (
        0.0,
        0.0,
    )

    active = director.tick(
        mission_input(
            0.20,
            (*B_PRE, 1.0),
            vision_seq=2,
            vision_found=True,
            vision_quality=95,
            vision_error_xy_m=(0.30, 0.0),
        )
    )
    assert active.vision_trim_vx_m_s > 0.0

    lost = director.tick(
        mission_input(0.30, (*B_PRE, 1.0), vision_found=False)
    )
    assert abs(lost.vision_trim_vx_m_s) < abs(active.vision_trim_vx_m_s)


def test_drop_gate_latches_and_visual_is_ignored_after_descent_starts():
    director = Task1MissionDirector(
        Task1Config(
            drop_during_bc_enabled=False,
            drop_at_follow_height=False,
        )
    )
    director.target_acquired = True
    director._transition(Task1Phase.DROP_WINDOW_C_D, 0.0, "test")
    director.cd_follower.reset(timestamp=0.0)

    first = director.tick(
        mission_input(
            0.1,
            (*C, 1.0),
            vision_seq=10,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.05, 0.02),
        )
    )
    assert first.phase == Task1Phase.DROP_WINDOW_C_D
    second = director.tick(
        mission_input(
            0.2,
            (C[0] - 0.01, C[1], 1.0),
            vision_seq=11,
            vision_found=True,
            vision_quality=90,
            vision_error_xy_m=(0.05, 0.02),
        )
    )
    assert second.phase == Task1Phase.DROP_DESCENT
    assert second.drop_committed

    release = director.tick(
        mission_input(
            0.3,
            (C[0] - 0.02, C[1], 0.9),
            vision_found=False,
            deck_relative_height_m=0.65,
        )
    )
    assert release.phase == Task1Phase.RELEASING
    assert release.release_requested

    climb = director.tick(
        mission_input(
            0.4,
            (C[0] - 0.03, C[1], 0.9),
            vision_found=False,
            deck_relative_height_m=0.65,
            payload_state=ActuatorState.RELEASED,
        )
    )
    assert climb.phase == Task1Phase.CLIMB
    assert climb.drop_released
    assert climb.mission_success


def test_drop_gate_is_disabled_for_visual_path_test():
    director = Task1MissionDirector(
        Task1Config(payload_drop_enabled=False)
    )
    director.target_acquired = True
    director._transition(Task1Phase.DROP_WINDOW_C_D, 0.0, "test")
    director.cd_follower.reset(timestamp=0.0)

    for seq in (1, 2):
        command = director.tick(
            mission_input(
                seq * 0.1,
                (*C, 1.0),
                vision_seq=seq,
                vision_found=True,
                vision_quality=100,
                vision_error_xy_m=(0.0, 0.0),
            )
        )
    assert command.phase == Task1Phase.DROP_WINDOW_C_D
    assert not command.drop_committed
    assert not command.release_requested


def test_reaching_d_without_gate_skips_release_and_returns():
    director = Task1MissionDirector()
    director.target_acquired = True
    director._transition(Task1Phase.DROP_WINDOW_C_D, 0.0, "test")
    director.cd_follower.reset(
        progress_m=director.cd_follower.path.length_m, timestamp=0.0
    )
    command = director.tick(mission_input(0.1, (*D, 1.0)))
    assert command.phase == Task1Phase.CLIMB
    assert not command.release_requested
    assert not command.mission_success
    assert command.reason == "d_reached_without_drop_gate"


def test_final_waypoint_requires_tight_stable_gate_before_descend():
    director = Task1MissionDirector(
        Task1Config(
            final_height_m=0.15,
            final_landing_radius_m=0.08,
            final_landing_max_speed_m_s=0.05,
            final_landing_stable_s=0.50,
        )
    )
    director._transition(Task1Phase.LAND_H, 0.0, "test")

    outside = director.tick(
        mission_input(0.1, (0.09, 0.0, 0.15), velocity_xy_m_s=(0.0, 0.0))
    )
    assert outside.target_xy_m == (0.0, 0.0)
    assert outside.target_world_height_m == 0.15
    assert not outside.land_requested

    too_fast = director.tick(
        mission_input(0.2, (0.04, 0.0, 0.15), velocity_xy_m_s=(0.06, 0.0))
    )
    assert not too_fast.land_requested

    first_stable = director.tick(
        mission_input(0.3, (0.04, 0.0, 0.15), velocity_xy_m_s=(0.03, 0.0))
    )
    assert not first_stable.land_requested
    almost = director.tick(
        mission_input(0.79, (0.03, 0.0, 0.15), velocity_xy_m_s=(0.02, 0.0))
    )
    assert not almost.land_requested
    ready = director.tick(
        mission_input(0.81, (0.03, 0.0, 0.15), velocity_xy_m_s=(0.02, 0.0))
    )
    assert ready.land_requested
