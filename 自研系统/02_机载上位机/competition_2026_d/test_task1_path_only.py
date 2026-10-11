from drone_control.competition_2026_d.task1_mission import Task1Config
from drone_control.competition_2026_d.task1_path_only_test import (
    build_path_test_config,
)


def test_path_only_descends_at_b_pre_and_disables_vision_wait():
    config = build_path_test_config(
        Task1Config(),
        cruise_height_m=1.5,
        follow_height_m=1.0,
        path_speed_m_s=0.13,
        intercept_speed_m_s=0.30,
        return_speed_m_s=0.25,
    )
    assert config.cruise_height_m == 1.5
    assert config.follow_height_m == 1.0
    assert config.path_only_b_pre_descent
    assert config.car_speed_m_s == 0.13
    assert config.car_speed_scale == 1.0
    assert config.intercept_speed_m_s == 0.30
    assert config.return_speed_m_s == 0.25
    assert not config.payload_drop_enabled


def test_visual_path_mode_waits_indefinitely_and_does_not_drop():
    config = build_path_test_config(
        Task1Config(),
        cruise_height_m=1.5,
        follow_height_m=1.0,
        path_speed_m_s=0.13,
        intercept_speed_m_s=0.30,
        return_speed_m_s=0.25,
        wait_for_target=True,
        curve_speed_m_s=0.06,
        path_lookahead_m=0.12,
    )
    assert config.acquire_timeout_s == float("inf")
    assert not config.path_only_b_pre_descent
    assert not config.payload_drop_enabled
    assert config.curve_speed_m_s == 0.06
    assert config.car_speed_m_s == 0.13
    assert config.path_lookahead_m == 0.12


def test_visual_path_mode_can_explicitly_enable_real_payload_drop():
    config = build_path_test_config(
        Task1Config(),
        cruise_height_m=1.5,
        follow_height_m=1.0,
        path_speed_m_s=0.075,
        intercept_speed_m_s=0.30,
        return_speed_m_s=0.25,
        wait_for_target=True,
        curve_speed_m_s=0.06,
        path_lookahead_m=0.12,
        payload_drop_enabled=True,
        drop_max_error_m=0.30,
        drop_confirm_duration_s=3.0,
        drop_at_follow_height=True,
    )
    assert config.payload_drop_enabled
    assert config.car_speed_m_s == 0.075
    assert config.drop_max_error_m == 0.30
    assert config.drop_confirm_duration_s == 3.0
    assert config.drop_during_bc_enabled
    assert config.drop_at_follow_height


def test_vision_track_only_locks_height_and_disables_path_and_payload():
    config = build_path_test_config(
        Task1Config(),
        cruise_height_m=1.5,
        follow_height_m=1.0,
        path_speed_m_s=0.075,
        intercept_speed_m_s=0.20,
        return_speed_m_s=0.35,
        payload_drop_enabled=True,
        vision_track_only=True,
    )
    assert config.vision_track_only
    assert config.follow_height_m == 1.5
    assert config.acquire_timeout_s == float("inf")
    assert not config.path_only_b_pre_descent
    assert not config.c_sync_vision_enabled
    assert not config.payload_drop_enabled
