from drone_control.competition_2026_d.task1_runtime import (
    HeightReferenceConfig,
    Task1T265SafetyMonitor,
    WorldDeckHeightController,
    observation_to_gate_sample,
)
from drone_control.competition_2026_d.task1_flight import Task1FlightMission
from drone_control.competition_2026_d.task1_mission import (
    Task1Config,
    Task1MissionDirector,
)
from drone_control.competition_2026_d.vision.platform_observation import (
    FeatureFlag,
    PlatformObservation,
)


def observation(**kwargs):
    values = dict(
        stream_id=1,
        seq=10,
        capture_ms=100,
        found=True,
        cx=300,
        cy=220,
        outer_px=100,
        inner_px=50,
        angle_cdeg=0,
        quality=90,
        flags=int(FeatureFlag.APRILTAG_VALID),
        received_monotonic=1.0,
    )
    values.update(kwargs)
    return PlatformObservation(**values)


def test_observation_is_converted_to_gate_error_but_not_velocity():
    sample = observation_to_gate_sample(
        observation(),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert sample.found
    assert sample.error_xy_m == (-0.04, -0.04)


def test_usb_visual_axes_match_down_positive_x_and_right_positive_y():
    left_down = observation_to_gate_sample(
        observation(cx=300, cy=260),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert left_down.error_xy_m == (0.04, -0.04)

    right_up = observation_to_gate_sample(
        observation(cx=340, cy=220),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert right_up.error_xy_m == (-0.04, 0.04)


def test_visual_target_center_applies_physical_xy_offset():
    centered_on_offset = observation_to_gate_sample(
        observation(cx=360, cy=215),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
        target_offset_xy_m=(-0.05, 0.08),
    )
    assert centered_on_offset.error_xy_m == (0.0, 0.0)

    geometric_center = observation_to_gate_sample(
        observation(cx=320, cy=240),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
        target_offset_xy_m=(-0.05, 0.08),
    )
    assert geometric_center.error_xy_m == (0.05, -0.08)


def test_observation_without_height_can_confirm_identity_not_center_error():
    sample = observation_to_gate_sample(
        observation(),
        now=1.05,
        relative_height_m=None,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert sample.found
    assert sample.error_xy_m is None
    assert sample.reason == "height_unavailable"


def test_task1_final_descend_keeps_limited_t265_horizontal_feedback():
    class FakeRealsense:
        @staticmethod
        def get_tracking_confidence():
            return 3

    flight = object.__new__(Task1FlightMission)
    flight.realsense = FakeRealsense()
    flight.director = Task1MissionDirector(
        Task1Config(final_descend_horizontal_max_speed_m_s=0.06)
    )

    vx_cms, vy_cms = flight._descend_horizontal_command([0.20, -0.10, 0.15])
    assert vx_cms < 0
    assert vy_cms > 0
    assert (vx_cms * vx_cms + vy_cms * vy_cms) ** 0.5 <= 6.1


def test_ambiguous_observation_is_rejected():
    sample = observation_to_gate_sample(
        observation(flags=int(FeatureFlag.AMBIGUOUS)),
        now=1.05,
        relative_height_m=1.0,
        max_age_s=0.15,
        min_quality=55,
        image_center_px=(320.0, 240.0),
        focal_px=(500.0, 500.0),
    )
    assert not sample.found
    assert sample.ambiguous


def test_platform_entering_laser_view_does_not_command_climb():
    controller = WorldDeckHeightController()
    first = controller.command(
        timestamp=0.0,
        current_world_height_m=1.0,
        current_laser_height_m=1.0,
        target_world_height_m=1.0,
        target_deck_height_m=None,
    )
    assert first.laser_setpoint_m == 1.0
    over_deck = controller.command(
        timestamp=0.1,
        current_world_height_m=1.0,
        current_laser_height_m=0.72,
        target_world_height_m=1.0,
        target_deck_height_m=None,
    )
    assert abs(over_deck.laser_setpoint_m - 0.72) < 1e-9


def test_first_height_sample_holds_current_laser_reference():
    controller = WorldDeckHeightController()
    command = controller.command(
        timestamp=0.0,
        current_world_height_m=0.15,
        current_laser_height_m=0.15,
        target_world_height_m=1.5,
        target_deck_height_m=None,
    )
    assert command.laser_setpoint_m == 0.15
    assert command.mode == "initial_hold"


def test_drop_height_uses_slew_limited_deck_relative_reference():
    controller = WorldDeckHeightController()
    controller.reset(timestamp=0.0, laser_height_m=1.0)
    command = controller.command(
        timestamp=1.0,
        current_world_height_m=1.0,
        current_laser_height_m=1.0,
        target_world_height_m=1.0,
        target_deck_height_m=0.65,
    )
    # dt is capped at 0.2 s: 0.09 m/s * 0.2 s = 0.018 m.
    assert abs(command.laser_setpoint_m - 0.982) < 1e-9
    assert command.mode == "deck_relative"


def test_task1_ascent_can_be_faster_without_accelerating_descent():
    controller = WorldDeckHeightController(
        HeightReferenceConfig(normal_ascent_slew_m_s=0.40)
    )
    controller.reset(timestamp=0.0, laser_height_m=1.0)
    climb = controller.command(
        timestamp=0.2,
        current_world_height_m=1.0,
        current_laser_height_m=1.0,
        target_world_height_m=1.5,
        target_deck_height_m=None,
    )
    assert abs(climb.laser_setpoint_m - 1.08) < 1e-9

    descent = controller.command(
        timestamp=0.4,
        current_world_height_m=1.5,
        current_laser_height_m=1.08,
        target_world_height_m=1.0,
        target_deck_height_m=None,
    )
    assert abs(descent.laser_setpoint_m - 1.03) < 1e-9


def test_t265_safety_detects_position_jump_even_with_valid_confidence():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=(0.0, 0.0),
    ) is None
    reason = monitor.update(
        timestamp=0.1,
        world_position_xyz_m=(0.7, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    )
    assert reason.startswith("t265_position_jump")


def test_t265_safety_detects_hold_drift():
    monitor = Task1T265SafetyMonitor()
    reason = monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.41, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=(0.0, 0.0),
    )
    assert reason == "t265_hold_geofence_exceeded"


def test_t265_safety_detects_ground_height_disagreement():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.5),
        laser_height_m=1.45,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    reason = None
    for timestamp in (1.0, 1.2, 1.4, 1.6):
        reason = monitor.update(
            timestamp=timestamp,
            world_position_xyz_m=(0.0, 0.0, -0.7),
            laser_height_m=0.09,
            ground_reference_expected=True,
            hold_anchor_xy_m=None,
        )
    assert reason.startswith("t265_laser_height_mismatch")


def test_t265_safety_ignores_single_laser_height_glitch():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.53),
        laser_height_m=1.63,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    assert monitor.update(
        timestamp=0.1,
        world_position_xyz_m=(0.02, 0.0, 1.53),
        laser_height_m=1.33,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    assert monitor.update(
        timestamp=0.2,
        world_position_xyz_m=(0.04, 0.0, 1.53),
        laser_height_m=1.63,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None


def test_platform_height_change_is_ignored_outside_ground_reference_phase():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.0),
        laser_height_m=0.95,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None
    assert monitor.update(
        timestamp=1.0,
        world_position_xyz_m=(0.1, 0.0, 1.0),
        laser_height_m=0.55,
        ground_reference_expected=False,
        hold_anchor_xy_m=None,
    ) is None


def test_new_ground_reference_phase_rebaselines_after_platform_edge():
    monitor = Task1T265SafetyMonitor()
    assert monitor.update(
        timestamp=0.0,
        world_position_xyz_m=(0.0, 0.0, 1.20),
        laser_height_m=1.15,
        ground_reference_expected=True,
        hold_anchor_xy_m=None,
    ) is None

    # During return flight the laser crosses from the raised H platform to
    # the lower floor.  Height consistency is intentionally inactive here.
    assert monitor.update(
        timestamp=1.0,
        world_position_xyz_m=(0.1, 0.1, 1.20),
        laser_height_m=1.40,
        ground_reference_expected=False,
        hold_anchor_xy_m=None,
    ) is None

    # LAND_H starts on that new surface.  It must establish a fresh baseline
    # instead of comparing against the takeoff-platform offset.
    for timestamp in (2.0, 2.2, 2.4, 2.6):
        assert monitor.update(
            timestamp=timestamp,
            world_position_xyz_m=(0.05, 0.05, 1.20),
            laser_height_m=1.40,
            ground_reference_expected=True,
            hold_anchor_xy_m=(0.0, 0.0),
        ) is None
