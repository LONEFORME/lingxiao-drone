"""Asynchronous waypoint action execution outside the flight-control loop."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import queue
import threading
import time
from typing import Callable, Mapping, Optional

from Lcode.mission_events import (
    ACTION_COMPLETED,
    ACTION_FAILED,
    ACTION_REQUESTED,
    MissionEvent,
)


class ActionPolicyError(ValueError):
    pass


@dataclass(frozen=True)
class ActionPolicy:
    enabled: bool = True
    queue_size: int = 16
    stop_timeout_s: float = 0.5
    signal_color: str = "B"
    signal_duration_s: float = 0.3
    allowed_actions: tuple[str, ...] = (
        "depart",
        "return",
        "noop",
        "observe",
        "signal",
    )

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "ActionPolicy":
        if raw is None:
            return cls()
        if not isinstance(raw, Mapping):
            raise ActionPolicyError("actions must be an object")
        enabled = _bool(raw.get("enabled", True), "enabled")
        actions_raw = raw.get("allowed_actions", list(cls().allowed_actions))
        if not isinstance(actions_raw, list):
            raise ActionPolicyError("actions.allowed_actions must be a list")
        allowed = tuple(
            str(action).strip().lower()
            for action in actions_raw
            if str(action).strip()
        )
        if not allowed or len(allowed) != len(set(allowed)):
            raise ActionPolicyError(
                "actions.allowed_actions must be unique and non-empty"
            )
        color = str(raw.get("signal_color", "B")).strip().upper()
        if color not in {"R", "G", "B", "W"}:
            raise ActionPolicyError("actions.signal_color must be R/G/B/W")
        return cls(
            enabled=enabled,
            queue_size=_positive_int(raw.get("queue_size", 16), "queue_size"),
            stop_timeout_s=_positive_float(
                raw.get("stop_timeout_s", 0.5), "stop_timeout_s"
            ),
            signal_color=color,
            signal_duration_s=_positive_float(
                raw.get("signal_duration_s", 0.3), "signal_duration_s"
            ),
            allowed_actions=allowed,
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "queue_size": self.queue_size,
            "stop_timeout_s": self.stop_timeout_s,
            "signal_color": self.signal_color,
            "signal_duration_s": self.signal_duration_s,
            "allowed_actions": list(self.allowed_actions),
        }


@dataclass(frozen=True)
class ActionResult:
    ok: bool
    detail: str = ""


ActionHandler = Callable[[MissionEvent], ActionResult]


def load_action_policy(path: str | Path) -> ActionPolicy:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ActionPolicyError(f"cannot load action config {config_path}: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise ActionPolicyError("competition config root must be an object")
    return ActionPolicy.from_mapping(raw.get("actions"))


class WaypointActionExecutor:
    _STOP = object()

    def __init__(
        self,
        policy: ActionPolicy,
        result_sink: Callable[[MissionEvent], object],
        handlers: Optional[Mapping[str, ActionHandler]] = None,
    ):
        self.policy = policy
        self.result_sink = result_sink
        self._queue: queue.Queue[object] = queue.Queue(policy.queue_size)
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._accepting = False
        self._stopped = False
        self._handlers: dict[str, ActionHandler] = {
            "depart": self._acknowledge,
            "return": self._acknowledge,
            "noop": self._acknowledge,
            "observe": self._acknowledge,
            "signal": self._signal,
        }
        if handlers:
            for name, handler in handlers.items():
                self._handlers[str(name).strip().lower()] = handler
        self._stats = {
            "accepted": 0,
            "completed": 0,
            "failed": 0,
            "dropped": 0,
            "unknown": 0,
        }

    @property
    def supported_actions(self) -> frozenset[str]:
        return frozenset(self._handlers)

    def start(self) -> bool:
        with self._lock:
            if self._thread is not None or self._stopped:
                return False
            self._accepting = True
            self._thread = threading.Thread(
                target=self._run, name="waypoint-actions", daemon=True
            )
            self._thread.start()
            return True

    def handle_event(self, event: MissionEvent) -> None:
        if event.event != ACTION_REQUESTED:
            return
        action = event.action.strip().lower()
        if action not in self.policy.allowed_actions or action not in self._handlers:
            with self._lock:
                self._stats["unknown"] += 1
                self._stats["failed"] += 1
            self._emit(ACTION_FAILED, event, error="unsupported_action")
            return
        with self._lock:
            if not self._accepting:
                return
            try:
                self._queue.put_nowait(event)
                self._stats["accepted"] += 1
                return
            except queue.Full:
                self._stats["dropped"] += 1
                self._stats["failed"] += 1
        self._emit(ACTION_FAILED, event, error="action_queue_full")

    def stop(self) -> bool:
        with self._lock:
            if self._stopped:
                thread = self._thread
                return thread is None or not thread.is_alive()
            self._stopped = True
            self._accepting = False
            thread = self._thread
        self._discard_pending()
        if thread is not None:
            self._queue.put_nowait(self._STOP)
            thread.join(self.policy.stop_timeout_s)
        return thread is None or not thread.is_alive()

    def stats(self) -> dict[str, int]:
        with self._lock:
            return {key: int(value) for key, value in self._stats.items()}

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._STOP:
                    return
                if isinstance(item, MissionEvent):
                    self._execute(item)
            finally:
                self._queue.task_done()

    def _execute(self, event: MissionEvent) -> None:
        action = event.action.strip().lower()
        handler = self._handlers.get(action)
        try:
            result = (
                handler(event)
                if handler is not None
                else ActionResult(False, "unsupported_action")
            )
        except Exception as exc:
            result = ActionResult(False, f"action_exception:{exc}")
        with self._lock:
            self._stats["completed" if result.ok else "failed"] += 1
        if result.ok:
            self._emit(ACTION_COMPLETED, event, detail=result.detail)
        else:
            self._emit(ACTION_FAILED, event, error=result.detail or "action_failed")

    def _discard_pending(self) -> None:
        discarded: list[MissionEvent] = []
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            self._queue.task_done()
            if isinstance(item, MissionEvent):
                discarded.append(item)
        if discarded:
            with self._lock:
                self._stats["dropped"] += len(discarded)
                self._stats["failed"] += len(discarded)
            for event in discarded:
                self._emit(ACTION_FAILED, event, error="action_shutdown")

    @staticmethod
    def _acknowledge(event: MissionEvent) -> ActionResult:
        return ActionResult(True, f"{event.action.strip().lower()}_accepted")

    def _signal(self, _event: MissionEvent) -> ActionResult:
        try:
            from Lcode.gpio_led import LedPriority, acquire_rgb_led, release_rgb_led

            token = acquire_rgb_led(
                self.policy.signal_color,
                owner="waypoint_action",
                priority=LedPriority.ACTION,
            )
            if token is None:
                return ActionResult(False, "led_unavailable_or_preempted")
            time.sleep(self.policy.signal_duration_s)
            release_rgb_led(token)
            return ActionResult(True, "signal_completed")
        except Exception as exc:
            return ActionResult(False, f"signal_exception:{exc}")

    def _emit(self, name: str, source: MissionEvent, **details: object) -> None:
        try:
            self.result_sink(
                MissionEvent(
                    event=name,
                    point_id=source.point_id,
                    target_index=source.target_index,
                    action=source.action,
                    details=details,
                )
            )
        except Exception:
            return


def _bool(value: object, field: str) -> bool:
    if not isinstance(value, bool):
        raise ActionPolicyError(f"actions.{field} must be boolean")
    return value


def _positive_int(value: object, field: str) -> int:
    if isinstance(value, bool):
        raise ActionPolicyError(f"actions.{field} must be a positive integer")
    try:
        normalized = int(value)
    except (TypeError, ValueError) as exc:
        raise ActionPolicyError(f"actions.{field} must be a positive integer") from exc
    if normalized <= 0 or normalized != value:
        raise ActionPolicyError(f"actions.{field} must be a positive integer")
    return normalized


def _positive_float(value: object, field: str) -> float:
    try:
        normalized = float(value)
    except (TypeError, ValueError) as exc:
        raise ActionPolicyError(f"actions.{field} must be positive") from exc
    if normalized <= 0:
        raise ActionPolicyError(f"actions.{field} must be positive")
    return normalized
