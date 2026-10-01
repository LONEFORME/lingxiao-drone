"""Asynchronous, hardware-neutral waypoint snapshot consumer."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import queue
import re
import threading
import time
from typing import Callable, Mapping, Optional

from Lcode.mission_events import (
    ACTION_REQUESTED,
    SNAPSHOT_CIRCUIT_OPEN,
    SNAPSHOT_FAILED,
    SNAPSHOT_SAVED,
    MissionEvent,
)
from Lcode.video_source import SnapshotResult, VideoSource


class SnapshotPolicyError(ValueError):
    pass


@dataclass(frozen=True)
class SnapshotPolicy:
    enabled: bool = False
    required: bool = False
    trigger_actions: tuple[str, ...] = ("observe", "snapshot", "inspect")
    timeout_s: float = 1.0
    start_timeout_s: float = 3.0
    stop_timeout_s: float = 0.5
    queue_size: int = 8
    max_snapshots: int = 32
    max_consecutive_failures: int = 3

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "SnapshotPolicy":
        if raw is None:
            return cls()
        if not isinstance(raw, Mapping):
            raise SnapshotPolicyError("auto_snapshot must be an object")
        enabled = _bool_value(raw.get("enabled", False), "enabled")
        required = _bool_value(raw.get("required", False), "required")
        actions_raw = raw.get("trigger_actions", ["observe", "snapshot", "inspect"])
        if not isinstance(actions_raw, list):
            raise SnapshotPolicyError("auto_snapshot.trigger_actions must be a list")
        actions = tuple(
            str(action).strip().lower() for action in actions_raw if str(action).strip()
        )
        if not actions:
            raise SnapshotPolicyError("auto_snapshot.trigger_actions cannot be empty")
        if len(actions) != len(set(actions)):
            raise SnapshotPolicyError("auto_snapshot.trigger_actions cannot contain duplicates")
        policy = cls(
            enabled=enabled,
            required=required,
            trigger_actions=actions,
            timeout_s=_positive_float(raw.get("timeout_s", 1.0), "timeout_s"),
            start_timeout_s=_positive_float(
                raw.get("start_timeout_s", 3.0), "start_timeout_s"
            ),
            stop_timeout_s=_positive_float(
                raw.get("stop_timeout_s", 0.5), "stop_timeout_s"
            ),
            queue_size=_positive_int(raw.get("queue_size", 8), "queue_size"),
            max_snapshots=_positive_int(
                raw.get("max_snapshots", 32), "max_snapshots"
            ),
            max_consecutive_failures=_positive_int(
                raw.get("max_consecutive_failures", 3),
                "max_consecutive_failures",
            ),
        )
        if required and not enabled:
            raise SnapshotPolicyError("required auto_snapshot must also be enabled")
        return policy

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "required": self.required,
            "trigger_actions": list(self.trigger_actions),
            "timeout_s": self.timeout_s,
            "start_timeout_s": self.start_timeout_s,
            "stop_timeout_s": self.stop_timeout_s,
            "queue_size": self.queue_size,
            "max_snapshots": self.max_snapshots,
            "max_consecutive_failures": self.max_consecutive_failures,
        }


def load_snapshot_policy(path: str | Path) -> SnapshotPolicy:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SnapshotPolicyError(f"cannot load snapshot config {config_path}: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise SnapshotPolicyError("competition config root must be an object")
    return SnapshotPolicy.from_mapping(raw.get("auto_snapshot"))


class WaypointSnapshotConsumer:
    """Queue waypoint snapshots without waiting in the event bus thread."""

    _STOP = object()

    def __init__(
        self,
        source: VideoSource,
        policy: SnapshotPolicy,
        output_dir: str | Path,
        run_id: str,
        result_sink: Callable[[MissionEvent], object],
    ):
        if not run_id.strip():
            raise ValueError("run_id cannot be empty")
        self.source = source
        self.policy = policy
        self.output_dir = Path(output_dir).resolve()
        self.run_id = run_id.strip()
        self.result_sink = result_sink
        self._queue: queue.Queue[object] = queue.Queue(policy.queue_size)
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._accepting = False
        self._stopped = False
        self._seen: set[tuple[str, int]] = set()
        self._consecutive_failures = 0
        self._circuit_open = False
        self.startup_timed_out = False
        self.last_error: Optional[str] = None
        self._stats = {
            "accepted": 0,
            "saved": 0,
            "failed": 0,
            "dropped": 0,
            "duplicates": 0,
            "filtered": 0,
            "circuit_open": False,
        }

    def start(self) -> bool:
        with self._lock:
            if self._thread is not None or self._stopped:
                raise RuntimeError("snapshot consumer instances cannot be restarted")
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            probe = self.output_dir / f".snapshot_write_probe_{self.run_id}.tmp"
            probe.write_bytes(b"")
            probe.unlink()
        except OSError as exc:
            self.last_error = f"snapshot_directory_unavailable:{exc}"
            return False

        completed, value, error = _bounded_call(self.source.start, self.policy.start_timeout_s)
        if not completed:
            self.startup_timed_out = True
            self.last_error = "video_source_start_timeout"
            return False
        if error is not None:
            self.last_error = f"video_source_start_error:{error}"
            return False
        if not value:
            self.last_error = "video_source_start_failed"
            return False

        with self._lock:
            self._accepting = True
            self._thread = threading.Thread(
                target=self._run, name="waypoint-snapshots", daemon=True
            )
            self._thread.start()
        return True

    def handle_event(self, event: MissionEvent) -> None:
        if event.event != ACTION_REQUESTED:
            return
        if event.action.strip().lower() not in self.policy.trigger_actions:
            with self._lock:
                self._stats["filtered"] += 1
            return

        failure: Optional[str] = None
        with self._lock:
            if not self._accepting:
                return
            key = (self.run_id, event.target_index)
            if key in self._seen:
                self._stats["duplicates"] += 1
                return
            self._seen.add(key)
            if self._circuit_open:
                self._stats["dropped"] += 1
                failure = "snapshot_circuit_open"
            elif self._stats["accepted"] >= self.policy.max_snapshots:
                self._stats["dropped"] += 1
                failure = "snapshot_limit_reached"
            else:
                try:
                    self._queue.put_nowait(event)
                    self._stats["accepted"] += 1
                except queue.Full:
                    self._stats["dropped"] += 1
                    failure = "snapshot_queue_full"
        if failure:
            self._emit(SNAPSHOT_FAILED, event, error=failure)

    def stop(self) -> bool:
        with self._lock:
            if self._stopped:
                thread = self._thread
                return thread is None or not thread.is_alive()
            self._stopped = True
            self._accepting = False
            thread = self._thread
        if thread is not None:
            self._discard_pending("snapshot_shutdown")
            self._queue.put_nowait(self._STOP)
            thread.join(self.policy.stop_timeout_s)

        _bounded_call(self.source.stop, self.policy.stop_timeout_s)
        return thread is None or not thread.is_alive()

    def stats(self) -> dict[str, object]:
        with self._lock:
            return dict(self._stats)

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._STOP:
                    return
                if isinstance(item, MissionEvent):
                    self._take_snapshot(item)
            finally:
                self._queue.task_done()

    def _take_snapshot(self, event: MissionEvent) -> None:
        point_name = _safe_point_name(event.point_id, event.target_index)
        started = time.monotonic()
        try:
            result = self.source.snapshot(
                point_name, self.output_dir, timeout_s=self.policy.timeout_s
            )
        except Exception as exc:
            result = SnapshotResult(point_name, None, None, f"snapshot_exception:{exc}")
        elapsed = time.monotonic() - started
        if elapsed > self.policy.timeout_s and result.ok:
            result = SnapshotResult(point_name, None, None, "snapshot_timeout_contract_breached")

        error = _validate_snapshot_result(result, self.output_dir)
        if error is None:
            with self._lock:
                self._stats["saved"] += 1
                self._consecutive_failures = 0
            self._emit(
                SNAPSHOT_SAVED,
                event,
                path=str(result.path.resolve()),
                captured_at=result.captured_at,
                elapsed_s=round(elapsed, 4),
            )
            return

        open_circuit = False
        with self._lock:
            self._stats["failed"] += 1
            self._consecutive_failures += 1
            if self._consecutive_failures >= self.policy.max_consecutive_failures:
                self._circuit_open = True
                self._accepting = False
                self._stats["circuit_open"] = True
                open_circuit = True
        self._emit(SNAPSHOT_FAILED, event, error=error, elapsed_s=round(elapsed, 4))
        if open_circuit:
            self._discard_pending("snapshot_circuit_open")
            self._emit(
                SNAPSHOT_CIRCUIT_OPEN,
                event,
                error="max_consecutive_failures_reached",
            )

    def _discard_pending(self, error: str) -> None:
        discarded: list[MissionEvent] = []
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            else:
                self._queue.task_done()
                if isinstance(item, MissionEvent):
                    discarded.append(item)
        if discarded:
            with self._lock:
                self._stats["dropped"] += len(discarded)
            for event in discarded:
                self._emit(SNAPSHOT_FAILED, event, error=error)

    def _emit(self, event_name: str, source_event: MissionEvent, **details: object) -> None:
        payload = {"run_id": self.run_id, **details}
        try:
            self.result_sink(
                MissionEvent(
                    event=event_name,
                    point_id=source_event.point_id,
                    target_index=source_event.target_index,
                    action=source_event.action,
                    details=payload,
                )
            )
        except Exception:
            return


def _validate_snapshot_result(result: SnapshotResult, output_dir: Path) -> Optional[str]:
    if not result.ok:
        return result.error or "snapshot_failed"
    assert result.path is not None
    try:
        resolved = result.path.resolve()
        resolved.relative_to(output_dir)
    except (OSError, ValueError):
        return "snapshot_path_outside_session"
    if not resolved.is_file():
        return "snapshot_file_missing"
    return None


def _safe_point_name(point_id: str, target_index: int) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", point_id.strip()).strip("_-")
    cleaned = (cleaned or "POINT")[:48]
    return f"{target_index:03d}_{cleaned}"


def _bounded_call(
    function: Callable[[], object], timeout_s: float
) -> tuple[bool, object, Optional[BaseException]]:
    result_queue: queue.Queue[tuple[object, Optional[BaseException]]] = queue.Queue(1)

    def invoke() -> None:
        try:
            result_queue.put((function(), None))
        except BaseException as exc:
            result_queue.put((None, exc))

    thread = threading.Thread(target=invoke, name="bounded-video-call", daemon=True)
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        return False, None, None
    value, error = result_queue.get_nowait()
    return True, value, error


def _bool_value(value: object, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise SnapshotPolicyError(f"auto_snapshot.{field_name} must be boolean")
    return value


def _positive_float(value: object, field_name: str) -> float:
    try:
        normalized = float(value)
    except (TypeError, ValueError) as exc:
        raise SnapshotPolicyError(f"auto_snapshot.{field_name} must be numeric") from exc
    if normalized <= 0:
        raise SnapshotPolicyError(f"auto_snapshot.{field_name} must be positive")
    return normalized


def _positive_int(value: object, field_name: str) -> int:
    if isinstance(value, bool):
        raise SnapshotPolicyError(f"auto_snapshot.{field_name} must be an integer")
    try:
        normalized = int(value)
    except (TypeError, ValueError) as exc:
        raise SnapshotPolicyError(f"auto_snapshot.{field_name} must be an integer") from exc
    if normalized <= 0 or normalized != value:
        raise SnapshotPolicyError(f"auto_snapshot.{field_name} must be a positive integer")
    return normalized
