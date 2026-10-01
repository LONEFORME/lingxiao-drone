import json
from pathlib import Path
import sys
import time
import types

import competition_main
from Lcode.mission_events import ACTION_REQUESTED, MissionEvent
from test_waypoint_snapshot import FakeVideoSource


def write_config(tmp_path, auto_snapshot=None, with_video=False):
    config = {
        "name": "entry test",
        "cruise_height_m": 1.0,
        "home": {"id": "HOME", "x": 0, "y": 0},
        "points": [
            {"id": "P1", "x": 0.2, "y": 0, "action": "observe"}
        ],
    }
    if auto_snapshot is not None:
        config["auto_snapshot"] = auto_snapshot
    if with_video:
        config["video"] = {
            "active_profile": "fake",
            "profiles": {
                "fake": {
                    "receiver": {
                        "enabled": True,
                        "backend": "fake",
                        "source": "memory",
                    }
                }
            },
        }
    path = tmp_path / "competition.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    return path


def install_fake_flight(monkeypatch, publish_action=False):
    module = types.ModuleType("main")

    def fake_flight(**kwargs):
        if publish_action:
            kwargs["event_sink"](
                MissionEvent(ACTION_REQUESTED, "P1", 1, "observe")
            )
            time.sleep(0.05)

    module.main = fake_flight
    monkeypatch.setitem(sys.modules, "main", module)


def test_default_disabled_does_not_create_video_source(tmp_path, monkeypatch):
    config = write_config(tmp_path)
    install_fake_flight(monkeypatch)

    def unexpected_create(_config):
        raise AssertionError("disabled snapshot policy created a video source")

    monkeypatch.setattr(competition_main, "create_video_source", unexpected_create)
    assert competition_main.main(
        [
            "--phase",
            "scout",
            "--config",
            str(config),
            "--sessions-root",
            str(tmp_path / "sessions"),
        ]
    ) == 0


def test_enabled_snapshot_writes_into_current_session(tmp_path, monkeypatch):
    config = write_config(
        tmp_path,
        auto_snapshot={"enabled": True, "required": True},
        with_video=True,
    )
    source = FakeVideoSource()
    monkeypatch.setattr(competition_main, "create_video_source", lambda _config: source)
    install_fake_flight(monkeypatch, publish_action=True)

    sessions_root = tmp_path / "sessions"
    assert competition_main.main(
        [
            "--phase",
            "scout",
            "--config",
            str(config),
            "--sessions-root",
            str(sessions_root),
        ]
    ) == 0

    sessions = list(sessions_root.iterdir())
    assert len(sessions) == 1
    images = list((sessions[0] / "snapshots").glob("*.jpg"))
    assert [image.name for image in images] == ["001_P1.jpg"]
    result = json.loads((sessions[0] / "scout_result.json").read_text(encoding="utf-8"))
    assert result["auto_snapshot"]["saved"] == 1
