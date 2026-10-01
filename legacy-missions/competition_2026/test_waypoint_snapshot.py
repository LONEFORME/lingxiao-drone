import json
from pathlib import Path
import threading
import time

import pytest

from Lcode.mission_events import (
    ACTION_REQUESTED,
    SNAPSHOT_CIRCUIT_OPEN,
    SNAPSHOT_FAILED,
    SNAPSHOT_SAVED,
    MissionEvent,
)
from Lcode.video_source import SnapshotResult, VideoFrame, VideoSource
from Lcode.waypoint_snapshot import (
    SnapshotPolicy,
    SnapshotPolicyError,
    WaypointSnapshotConsumer,
    load_snapshot_policy,
)


class FakeVideoSource(VideoSource):
    def __init__(self, failures=0, block_event=None):
        self.failures = failures
        self.block_event = block_event
        self.entered = threading.Event()
        self.running = False
        self.snapshot_calls = []

    def start(self):
        self.running = True
        return True

    def read_frame(self, timeout_s=0.5):
        return None

    def snapshot(self, point_id, output_dir, timeout_s=1.0):
        self.snapshot_calls.append((point_id, timeout_s))
        self.entered.set()
        if self.block_event is not None:
            self.block_event.wait(1.0)
        if self.failures:
            self.failures -= 1
            return SnapshotResult(point_id, None, None, "simulated_failure")
        output = Path(output_dir)
        temporary = output / f"{point_id}.jpg.tmp"
        final = output / f"{point_id}.jpg"
        temporary.write_bytes(b"fake-jpeg")
        temporary.replace(final)
        return SnapshotResult(point_id, final, time.time())

    def is_running(self):
        return self.running

    def stop(self):
        self.running = False


class SlowStartVideoSource(FakeVideoSource):
    def __init__(self, release):
        super().__init__()
        self.release = release

    def start(self):
        self.release.wait(1.0)
        return super().start()


def event(point_id="P1", target_index=1, action="observe"):
    return MissionEvent(ACTION_REQUESTED, point_id, target_index, action)


def wait_for(predicate, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_snapshot_success_filters_actions_and_deduplicates(tmp_path):
    source = FakeVideoSource()
    results = []
    consumer = WaypointSnapshotConsumer(
        source,
        SnapshotPolicy(),
        tmp_path,
        "run-a",
        results.append,
    )
    assert consumer.start()

    consumer.handle_event(event("../../P 1", 2))
    consumer.handle_event(event("../../P 1", 2))
    consumer.handle_event(event("P3", 3, "drop_payload"))

    assert wait_for(lambda: any(item.event == SNAPSHOT_SAVED for item in results))
    assert consumer.stop()
    saved = next(item for item in results if item.event == SNAPSHOT_SAVED)
    assert Path(saved.details["path"]).parent == tmp_path.resolve()
    assert Path(saved.details["path"]).name == "002_P_1.jpg"
    assert saved.details["run_id"] == "run-a"
    assert consumer.stats()["duplicates"] == 1
    assert consumer.stats()["filtered"] == 1
    assert len(source.snapshot_calls) == 1


def test_consecutive_failures_open_circuit_and_drop_pending(tmp_path):
    source = FakeVideoSource(failures=3)
    results = []
    policy = SnapshotPolicy(max_consecutive_failures=2)
    consumer = WaypointSnapshotConsumer(source, policy, tmp_path, "run-b", results.append)
    assert consumer.start()

    consumer.handle_event(event("P1", 1))
    consumer.handle_event(event("P2", 2))
    consumer.handle_event(event("P3", 3))

    assert wait_for(lambda: any(item.event == SNAPSHOT_CIRCUIT_OPEN for item in results))
    consumer.stop()
    stats = consumer.stats()
    assert stats["failed"] == 2
    assert stats["circuit_open"] is True
    assert stats["dropped"] == 1
    assert sum(item.event == SNAPSHOT_FAILED for item in results) == 3


def test_full_queue_drops_without_blocking_event_handler(tmp_path):
    release = threading.Event()
    source = FakeVideoSource(block_event=release)
    results = []
    consumer = WaypointSnapshotConsumer(
        source,
        SnapshotPolicy(queue_size=1),
        tmp_path,
        "run-c",
        results.append,
    )
    assert consumer.start()
    consumer.handle_event(event("P1", 1))
    assert source.entered.wait(0.5)

    consumer.handle_event(event("P2", 2))
    started = time.monotonic()
    consumer.handle_event(event("P3", 3))
    assert time.monotonic() - started < 0.05
    assert any(
        item.event == SNAPSHOT_FAILED
        and item.details.get("error") == "snapshot_queue_full"
        for item in results
    )

    release.set()
    assert wait_for(lambda: consumer.stats()["saved"] >= 1)
    consumer.stop()


def test_policy_loads_defaults_and_rejects_invalid_values(tmp_path):
    config = tmp_path / "config.json"
    config.write_text("{}", encoding="utf-8")
    assert load_snapshot_policy(config) == SnapshotPolicy()

    config.write_text(
        json.dumps({"auto_snapshot": {"enabled": False, "queue_size": 0}}),
        encoding="utf-8",
    )
    with pytest.raises(SnapshotPolicyError):
        load_snapshot_policy(config)


def test_new_run_id_does_not_inherit_deduplication(tmp_path):
    first_source = FakeVideoSource()
    first = WaypointSnapshotConsumer(
        first_source, SnapshotPolicy(), tmp_path / "first", "run-1", lambda _: None
    )
    assert first.start()
    first.handle_event(event("P1", 1))
    assert wait_for(lambda: first.stats()["saved"] == 1)
    first.stop()

    second_source = FakeVideoSource()
    second = WaypointSnapshotConsumer(
        second_source, SnapshotPolicy(), tmp_path / "second", "run-2", lambda _: None
    )
    assert second.start()
    second.handle_event(event("P1", 1))
    assert wait_for(lambda: second.stats()["saved"] == 1)
    second.stop()
    assert len(second_source.snapshot_calls) == 1


def test_snapshot_limit_rejects_additional_points(tmp_path):
    release = threading.Event()
    source = FakeVideoSource(block_event=release)
    results = []
    consumer = WaypointSnapshotConsumer(
        source,
        SnapshotPolicy(max_snapshots=1),
        tmp_path,
        "run-limit",
        results.append,
    )
    assert consumer.start()
    consumer.handle_event(event("P1", 1))
    assert source.entered.wait(0.5)
    consumer.handle_event(event("P2", 2))
    assert any(
        item.event == SNAPSHOT_FAILED
        and item.details.get("error") == "snapshot_limit_reached"
        for item in results
    )
    release.set()
    consumer.stop()


def test_start_rejects_unwritable_output_target(tmp_path):
    output_file = tmp_path / "not-a-directory"
    output_file.write_text("occupied", encoding="utf-8")
    consumer = WaypointSnapshotConsumer(
        FakeVideoSource(),
        SnapshotPolicy(),
        output_file,
        "run-dir-fail",
        lambda _: None,
    )
    assert not consumer.start()
    assert consumer.last_error.startswith("snapshot_directory_unavailable:")


def test_start_timeout_is_detected_before_flight(tmp_path):
    release = threading.Event()
    consumer = WaypointSnapshotConsumer(
        SlowStartVideoSource(release),
        SnapshotPolicy(start_timeout_s=0.02),
        tmp_path,
        "run-start-timeout",
        lambda _: None,
    )
    assert not consumer.start()
    assert consumer.startup_timed_out
    assert consumer.last_error == "video_source_start_timeout"
    release.set()
    consumer.stop()
