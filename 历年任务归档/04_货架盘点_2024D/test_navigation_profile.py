import pytest

from Lcode.navigation_profile import NavigationProfileConfig


def test_default_profile_preserves_precision_behavior():
    config = NavigationProfileConfig.from_env({})
    assert config.profile == "precision"
    assert [config.waypoint_mode(i, 4) for i in range(4)] == ["precision"] * 4


def test_cruise_profile_protects_first_and_last_waypoint():
    config = NavigationProfileConfig(profile="cruise")
    assert [config.waypoint_mode(i, 5) for i in range(5)] == [
        "precision",
        "cruise",
        "cruise",
        "cruise",
        "precision",
    ]


def test_short_route_becomes_all_precision_when_protected_ranges_overlap():
    config = NavigationProfileConfig(
        profile="cruise", precision_head=2, precision_tail=2
    )
    assert [config.waypoint_mode(i, 3) for i in range(3)] == ["precision"] * 3


def test_cruise_timeout_keeps_4m_baseline_and_scales_for_long_route():
    config = NavigationProfileConfig(profile="cruise")
    assert config.cruise_timeout_s(4.0) == pytest.approx(25.0)
    assert config.cruise_timeout_s(10.0) == pytest.approx(55.0)


def test_from_env_parses_cruise_overrides():
    config = NavigationProfileConfig.from_env(
        {
            "DRONE_NAV_PROFILE": "cruise",
            "DRONE_CRUISE_PRECISION_HEAD": "2",
            "DRONE_CRUISE_PRECISION_TAIL": "2",
            "DRONE_CRUISE_RADIUS_M": "0.2",
            "DRONE_CRUISE_CONFIRM_CYCLES": "4",
            "DRONE_CRUISE_REQUIRE_Z": "1",
            "DRONE_CRUISE_TIMEOUT_S": "30",
            "DRONE_CRUISE_MIN_PROGRESS_MPS": "0.1",
            "DRONE_CRUISE_TIMEOUT_MARGIN_S": "6",
        }
    )
    assert config.profile == "cruise"
    assert config.precision_head == 2
    assert config.precision_tail == 2
    assert config.cruise_radius_m == pytest.approx(0.2)
    assert config.cruise_confirm_cycles == 4
    assert config.cruise_require_z is True
    assert config.cruise_timeout_s(4.0) == pytest.approx(46.0)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"profile": "fast"},
        {"precision_head": -1},
        {"precision_head": 0},
        {"precision_tail": 0},
        {"cruise_radius_m": 0.01},
        {"cruise_confirm_cycles": 0},
        {"cruise_confirm_cycles": 1},
        {"cruise_timeout_base_s": 1.0},
        {"cruise_min_progress_mps": 0.01},
        {"cruise_min_progress_mps": 0.5},
    ],
)
def test_invalid_config_is_rejected(kwargs):
    with pytest.raises(ValueError):
        NavigationProfileConfig(**kwargs)
