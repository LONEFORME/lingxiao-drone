"""Non-throwing mission outcome tracking for the flight/control boundary."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import threading
import time
from typing import Callable


class MissionStatus(str, Enum):
    NOT_STARTED = "not_started"
    RUNNING = "running"
    ROUTE_COMPLETED = "route_completed"
    COMPLETED = "completed"
    COMPLETED_WITH_WARNINGS = "completed_with_warnings"
    CANCELLED = "cancelled"
    PREFLIGHT_FAILED = "preflight_failed"
    HARDWARE_FAILED = "hardware_failed"
    EMERGENCY_STOPPED = "emergency_stopped"
    INTERRUPTED = "interrupted"


TERMINAL_STATUSES = {
    MissionStatus.COMPLETED,
    MissionStatus.COMPLETED_WITH_WARNINGS,
    MissionStatus.CANCELLED,
    MissionStatus.PREFLIGHT_FAILED,
    MissionStatus.HARDWARE_FAILED,
    MissionStatus.EMERGENCY_STOPPED,
    MissionStatus.INTERRUPTED,
}


@dataclass(frozen=True)
class MissionResult:
    status: MissionStatus
    reason: str
    route_completed: bool
    waypoint_timeouts: int
    action_failures: int
    warnings: tuple[str, ...]
    started_at: float | None
    finished_at: float | None

    @property
    def terminal(self) -> bool:
        return self.status in TERMINAL_STATUSES

    def as_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["status"] = self.status.value
        result["warnings"] = list(self.warnings)
        return result


class MissionOutcomeTracker:
    """Track mission state without performing logging or I/O.

    Every mutation is best-effort and returns False instead of raising. This
    makes calls safe after the lock/stop command and hardware cleanup.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._status = MissionStatus.NOT_STARTED
        self._reason = ""
        self._route_completed = False
        self._waypoint_timeouts = 0
        self._action_failures = 0
        self._warnings: list[str] = []
        self._started_at: float | None = None
        self._finished_at: float | None = None
        self._last_snapshot = self._build_result()

    def start(self) -> bool:
        return self._mutate(self._start)

    def cancel(self, reason: str) -> bool:
        return self._mutate(lambda: self._set_terminal(MissionStatus.CANCELLED, reason))

    def preflight_failed(self, reason: str) -> bool:
        return self._mutate(
            lambda: self._set_terminal(MissionStatus.PREFLIGHT_FAILED, reason)
        )

    def hardware_failed(self, reason: str) -> bool:
        return self._mutate(
            lambda: self._set_terminal(MissionStatus.HARDWARE_FAILED, reason)
        )

    def emergency_stopped(self, reason: str) -> bool:
        return self._mutate(
            lambda: self._set_terminal(MissionStatus.EMERGENCY_STOPPED, reason)
        )

    def interrupted(self, reason: str) -> bool:
        return self._mutate(
            lambda: self._set_terminal(MissionStatus.INTERRUPTED, reason)
        )

    def route_completed(self) -> bool:
        def update() -> bool:
            if self._status != MissionStatus.RUNNING:
                return False
            self._status = MissionStatus.ROUTE_COMPLETED
            self._route_completed = True
            self._reason = "route_completed"
            return True

        return self._mutate(update)

    def record_waypoint_timeout(self, point_id: str) -> bool:
        def update() -> bool:
            self._waypoint_timeouts += 1
            self._append_warning(f"waypoint_timeout:{point_id}")
            return True

        return self._mutate(update)

    def record_action_failure(self, point_id: str, error: str) -> bool:
        def update() -> bool:
            self._action_failures += 1
            self._append_warning(f"action_failed:{point_id}:{error}")
            return True

        return self._mutate(update)

    def record_warning(self, warning: str) -> bool:
        return self._mutate(lambda: self._append_warning(warning))

    def finalize(self, extra_action_failures: int = 0) -> bool:
        def update() -> bool:
            if self._status != MissionStatus.ROUTE_COMPLETED:
                return False
            self._action_failures += max(0, int(extra_action_failures))
            has_warnings = bool(
                self._warnings or self._waypoint_timeouts or self._action_failures
            )
            self._status = (
                MissionStatus.COMPLETED_WITH_WARNINGS
                if has_warnings
                else MissionStatus.COMPLETED
            )
            self._reason = self._status.value
            self._finished_at = time.time()
            return True

        return self._mutate(update)

    def snapshot(self) -> MissionResult:
        try:
            acquired = self._lock.acquire(timeout=0.01)
        except BaseException:
            return self._last_snapshot
        if not acquired:
            return self._last_snapshot
        try:
            self._last_snapshot = self._build_result()
            return self._last_snapshot
        except BaseException:
            return self._last_snapshot
        finally:
            self._lock.release()

    def _start(self) -> bool:
        if self._status != MissionStatus.NOT_STARTED:
            return False
        self._status = MissionStatus.RUNNING
        self._reason = "running"
        self._started_at = time.time()
        return True

    def _set_terminal(self, status: MissionStatus, reason: str) -> bool:
        if self._status in TERMINAL_STATUSES:
            return False
        self._status = status
        self._reason = str(reason)
        self._finished_at = time.time()
        return True

    def _append_warning(self, warning: str) -> bool:
        text = str(warning).strip()
        if text and text not in self._warnings:
            self._warnings.append(text)
        return True

    def _mutate(self, operation: Callable[[], bool]) -> bool:
        try:
            acquired = self._lock.acquire(timeout=0.01)
        except BaseException:
            return False
        if not acquired:
            return False
        try:
            changed = bool(operation())
            self._last_snapshot = self._build_result()
            return changed
        except BaseException:
            return False
        finally:
            self._lock.release()

    def _build_result(self) -> MissionResult:
        return MissionResult(
            status=self._status,
            reason=self._reason,
            route_completed=self._route_completed,
            waypoint_timeouts=self._waypoint_timeouts,
            action_failures=self._action_failures,
            warnings=tuple(self._warnings),
            started_at=self._started_at,
            finished_at=self._finished_at,
        )
