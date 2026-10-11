from Lcode.action_executor import ActionPolicy
from Lcode.preflight import (
    CompetitionPreflight,
    PreflightConfig,
    ServiceReadiness,
)


def test_preflight_accepts_route_without_geofence_checks(tmp_path):
    report = CompetitionPreflight(PreflightConfig(min_free_mb=1)).run(
        point_ids=["HOME", "P1", "HOME"],
        waypoints=[[0, 0, 1], [100, -100, 1], [0, 0, 1]],
        actions=["depart", "observe", "return"],
        action_policy=ActionPolicy(),
        supported_actions=ActionPolicy().allowed_actions,
        session_dir=tmp_path,
    )
    assert report.ok
    geofence = next(check for check in report.checks if check.name == "geofence")
    assert "disabled" in geofence.detail


def test_preflight_rejects_unknown_action_and_bad_home(tmp_path):
    report = CompetitionPreflight(PreflightConfig(min_free_mb=1)).run(
        point_ids=["P1"],
        waypoints=[[0, 0, 1]],
        actions=["magnet"],
        action_policy=ActionPolicy(),
        supported_actions=ActionPolicy().allowed_actions,
        session_dir=tmp_path,
    )
    assert not report.ok
    assert {failure.name for failure in report.failures} >= {
        "home_endpoints",
        "actions_supported",
    }


def test_optional_service_warns_but_required_service_fails(tmp_path):
    common = dict(
        point_ids=["HOME", "HOME"],
        waypoints=[[0, 0, 1], [0, 0, 1]],
        actions=["depart", "return"],
        action_policy=ActionPolicy(),
        supported_actions=ActionPolicy().allowed_actions,
        session_dir=tmp_path,
    )
    optional = CompetitionPreflight(PreflightConfig(min_free_mb=1)).run(
        **common,
        services={"video": ServiceReadiness(False, False, "camera missing")},
    )
    required = CompetitionPreflight(PreflightConfig(min_free_mb=1)).run(
        **common,
        services={"video": ServiceReadiness(False, True, "camera missing")},
    )
    assert optional.ok and optional.warnings
    assert not required.ok
