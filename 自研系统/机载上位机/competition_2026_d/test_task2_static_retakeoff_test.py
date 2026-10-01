import pytest

from .task2_static_retakeoff_test import (
    StaticFlightConfig,
    StaticRetakeoffMission,
    _build_parser,
    _config_from_args,
    mission,
    sp_side,
)


def _make_controller(config=None, anchor=(0.02, -0.03)):
    return StaticRetakeoffMission(
        [0] * 14,
        [170, 2, 0, 100, 100, 0, 100, 0, 100, 0, 255],
        None,
        None,
        config or StaticFlightConfig(),
        anchor,
    )


def test_default_static_test_uses_1m_and_five_second_unlocked_deck_hold():
    args = _build_parser().parse_args(
        ["--propellers-installed", "--static-platform"]
    )
    config = _config_from_args(args)
    assert config.height_m == 1.00
    assert config.airborne_hold_s == 2.0
    assert config.deck_hold_s == 5.0


def test_static_test_requires_both_explicit_safety_flags():
    args = _build_parser().parse_args(["--propellers-installed"])
    with pytest.raises(SystemExit):
        _config_from_args(args)


@pytest.mark.parametrize("height", [0.30, 0.39, 1.01])
def test_height_outside_low_altitude_range_is_rejected(height):
    args = _build_parser().parse_args(
        [
            "--propellers-installed",
            "--static-platform",
            "--height",
            str(height),
        ]
    )
    with pytest.raises(SystemExit):
        _config_from_args(args)


def test_static_controller_uses_existing_mature_flight_methods_unchanged():
    assert StaticRetakeoffMission.takeoff is mission.takeoff
    assert StaticRetakeoffMission.navigate is mission.navigate
    assert StaticRetakeoffMission.descend is mission.descend
    assert StaticRetakeoffMission.land is mission.land


class _TakeoffT265:
    def get_tracking_confidence(self):
        return 3

    def get_orientation(self):
        return (0.0, 0.0, 0.0)


def test_begin_takeoff_only_triggers_firmware_and_keeps_xy_zero():
    controller = StaticRetakeoffMission(
        [0] * 14,
        [170, 2, 0, 0, 0, 0, 0, 0, 0, 0, 255],
        _TakeoffT265(),
        None,
        StaticFlightConfig(),
        (0.0, 0.0),
    )
    controller.begin_firmware_takeoff()
    assert controller.se_fc[2] == 1
    assert controller.se_fc[3] == sp_side
    assert controller.se_fc[4] == sp_side
    assert controller.se_fc[5] == 35
    assert controller.se_fc[7] == 0


def test_static_waypoints_hold_anchor_at_cruise_and_final_height():
    controller = _make_controller(anchor=(0.02, -0.03))
    assert controller.targets == [
        [0.02, -0.03, 1.0],
        [0.02, -0.03, 0.15],
    ]
    controller.target_index = 0
    assert controller._waypoint_hold_s() == 2.0
    controller.target_index = 1
    assert controller._waypoint_hold_s() == 0.0


def test_prepare_reascent_keeps_task_active_without_second_unlock_edge():
    t265_marker = object()
    controller = StaticRetakeoffMission(
        [0] * 14,
        [170, 2, 1, 120, 80, 15, 110, 101, 100, 0, 255],
        t265_marker,
        None,
        StaticFlightConfig(),
        (0.0, 0.0),
    )
    controller.target_index = 2
    controller._arrival_window.append(True)
    controller._vel_window.append((1.0, 1.0))
    controller.prepare_unlocked_reascent(0.08)

    assert controller.realsense is t265_marker
    assert controller.cycle_number == 2
    assert controller.target_index == 0
    assert controller.state == "NAVIGATE"
    assert list(controller._arrival_window) == []
    assert list(controller._vel_window) == []
    assert controller.se_fc[2] == 1
    assert controller.se_fc[3:7] == [sp_side, sp_side, 8, 110]
    assert controller.se_fc[7] == 0
