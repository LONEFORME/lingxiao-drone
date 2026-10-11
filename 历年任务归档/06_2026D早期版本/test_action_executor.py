import threading
import time

from Lcode.action_executor import ActionPolicy, ActionResult, WaypointActionExecutor
from Lcode.mission_events import (
    ACTION_COMPLETED,
    ACTION_FAILED,
    ACTION_REQUESTED,
    MissionEvent,
)


def request(action="observe", index=1):
    return MissionEvent(ACTION_REQUESTED, f"P{index}", index, action)


def wait_for(predicate, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_builtin_action_completes_without_blocking_flight():
    results = []
    executor = WaypointActionExecutor(ActionPolicy(), results.append)
    assert executor.start()
    executor.handle_event(request("observe"))
    assert wait_for(lambda: any(item.event == ACTION_COMPLETED for item in results))
    assert executor.stop()
    assert executor.stats()["completed"] == 1


def test_unknown_action_fails_immediately():
    results = []
    executor = WaypointActionExecutor(ActionPolicy(), results.append)
    executor.start()
    executor.handle_event(request("magnet"))
    assert results[-1].event == ACTION_FAILED
    assert results[-1].details["error"] == "unsupported_action"
    executor.stop()


def test_queue_full_publishes_failure_without_waiting():
    entered = threading.Event()
    release = threading.Event()

    def blocking(_event):
        entered.set()
        release.wait(1.0)
        return ActionResult(True, "done")

    results = []
    executor = WaypointActionExecutor(
        ActionPolicy(queue_size=1), results.append, {"observe": blocking}
    )
    executor.start()
    executor.handle_event(request(index=1))
    assert entered.wait(0.5)
    executor.handle_event(request(index=2))
    started = time.monotonic()
    executor.handle_event(request(index=3))
    assert time.monotonic() - started < 0.05
    assert any(
        item.event == ACTION_FAILED
        and item.details.get("error") == "action_queue_full"
        for item in results
    )
    release.set()
    executor.stop()
