import json

from Lcode.mission_events import MissionEvent, MissionEventBus, WAYPOINT_ARRIVED
from Lcode.mission_session import MissionSession
from Mission_GPT import mission


def test_event_bus_dispatches_event_off_caller_thread():
    received = []
    bus = MissionEventBus()
    bus.subscribe(received.append)
    bus.start()

    assert bus.publish(MissionEvent(WAYPOINT_ARRIVED, "P2", 1, "observe"))
    bus.close()

    assert [event.point_id for event in received] == ["P2"]


def test_session_reuses_directory_for_two_phases(tmp_path):
    session = MissionSession.create(tmp_path, "round-1")
    scout_run_id = session.begin(
        "test mission", "scout", {"point_ids": ["HOME", "P1"]}
    )
    session.record_event(MissionEvent(WAYPOINT_ARRIVED, "P1", 1, "observe"))
    session.finish("scout", "finished")

    resumed = MissionSession.create(tmp_path, session.path)
    execute_run_id = resumed.begin(
        "test mission", "execute", {"point_ids": ["HOME", "P1"]}
    )
    resumed.finish("execute", "finished")

    metadata = json.loads((session.path / "session.json").read_text(encoding="utf-8"))
    event = json.loads((session.path / "events.jsonl").read_text(encoding="utf-8"))
    assert metadata["phases"]["scout"]["status"] == "finished"
    assert metadata["phases"]["execute"]["status"] == "finished"
    assert metadata["phases"]["scout"]["run_id"] == scout_run_id
    assert metadata["phases"]["execute"]["run_id"] == execute_run_id
    assert scout_run_id != execute_run_id
    assert event["point_id"] == "P1"
    assert (session.path / "scout_plan.json").exists()
    assert (session.path / "execute_plan.json").exists()
    assert session.snapshots_dir.is_dir()


def test_flight_mission_events_include_point_metadata():
    received = []
    flight = mission(
        [0] * 14,
        [0] * 11,
        targets=[[1.0, 2.0, 1.0]],
        waypoint_holds=[2.0],
        point_ids=["P3"],
        waypoint_actions=["inspect"],
        event_sink=received.append,
    )

    flight._reset_arrival_tracking([0.0, 0.0, 1.0])
    flight._advance_waypoint("timeout", [0.5, 1.0, 1.0], flight.targets[0], 1.1)

    assert [event.event for event in received] == [
        "WAYPOINT_APPROACHING",
        "WAYPOINT_LEFT",
    ]
    assert all(event.point_id == "P3" for event in received)
    assert all(event.action == "inspect" for event in received)
    assert received[-1].details["reason"] == "timeout"
