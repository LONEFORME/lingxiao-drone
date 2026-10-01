from Lcode.mission_outcome import MissionOutcomeTracker, MissionStatus


def test_route_finalization_waits_for_background_warnings():
    tracker = MissionOutcomeTracker()
    assert tracker.start()
    assert tracker.record_waypoint_timeout("P2")
    assert tracker.route_completed()
    assert tracker.snapshot().status == MissionStatus.ROUTE_COMPLETED
    assert tracker.finalize(extra_action_failures=1)
    result = tracker.snapshot()
    assert result.status == MissionStatus.COMPLETED_WITH_WARNINGS
    assert result.route_completed
    assert result.waypoint_timeouts == 1
    assert result.action_failures == 1


def test_emergency_terminal_cannot_be_overwritten_by_completion():
    tracker = MissionOutcomeTracker()
    tracker.start()
    assert tracker.emergency_stopped("fc_timeout")
    assert not tracker.route_completed()
    assert not tracker.finalize()
    assert tracker.snapshot().status == MissionStatus.EMERGENCY_STOPPED


def test_cancel_before_start_is_terminal_and_serializable():
    tracker = MissionOutcomeTracker()
    assert tracker.cancel("button_gate")
    result = tracker.snapshot()
    assert result.status == MissionStatus.CANCELLED
    assert result.as_dict()["status"] == "cancelled"
    assert not tracker.start()
