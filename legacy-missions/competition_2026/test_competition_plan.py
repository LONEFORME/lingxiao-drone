import json

import pytest

from Lcode.competition_plan import (
    CompetitionPlanError,
    load_competition_config,
    plan_mission,
)


def write_config(tmp_path):
    path = tmp_path / "competition.json"
    path.write_text(
        json.dumps(
            {
                "name": "test",
                "cruise_height_m": 1.2,
                "home_hold_s": 0.5,
                "home": {"x": 0, "y": 0},
                "scout_order": ["P2", "P1"],
                "points": [
                    {"id": "P1", "x": 1, "y": 0, "hold_s": 2},
                    {"id": "P2", "x": 0, "y": 1, "z": 1.1, "hold_s": 3},
                ],
            }
        ),
        encoding="utf-8",
    )
    return path


def test_scout_plan_uses_configured_order_and_returns_home(tmp_path):
    config = load_competition_config(write_config(tmp_path))
    mission = plan_mission(config, "scout")

    assert mission.point_ids == ("HOME", "P2", "P1", "HOME")
    assert mission.waypoints == (
        (0.0, 0.0, 1.2),
        (0.0, 1.0, 1.1),
        (1.0, 0.0, 1.2),
        (0.0, 0.0, 1.2),
    )
    assert mission.hold_s == (0.5, 3.0, 2.0, 0.5)


def test_execute_plan_uses_operator_selected_order(tmp_path):
    config = load_competition_config(write_config(tmp_path))
    mission = plan_mission(config, "execute", ("P1", "P2"))

    assert mission.point_ids == ("HOME", "P1", "P2", "HOME")
    assert mission.actions == ("depart", "observe", "observe", "return")


@pytest.mark.parametrize("selected", [(), ("PX",), ("P1", "P1")])
def test_execute_plan_rejects_invalid_selection(tmp_path, selected):
    config = load_competition_config(write_config(tmp_path))
    with pytest.raises(CompetitionPlanError):
        plan_mission(config, "execute", selected)

