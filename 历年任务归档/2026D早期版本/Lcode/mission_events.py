"""Non-blocking waypoint events shared by flight and competition modules."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import queue
import threading
import time
from typing import Callable, Mapping, Optional


WAYPOINT_APPROACHING = "WAYPOINT_APPROACHING"
WAYPOINT_ARRIVED = "WAYPOINT_ARRIVED"
HOLD_STARTED = "HOLD_STARTED"
ACTION_REQUESTED = "ACTION_REQUESTED"
ACTION_COMPLETED = "ACTION_COMPLETED"
ACTION_FAILED = "ACTION_FAILED"
WAYPOINT_LEFT = "WAYPOINT_LEFT"
SNAPSHOT_SAVED = "SNAPSHOT_SAVED"
SNAPSHOT_FAILED = "SNAPSHOT_FAILED"
SNAPSHOT_CIRCUIT_OPEN = "SNAPSHOT_CIRCUIT_OPEN"
TASK_STARTED = "TASK_STARTED"
TASK_FINISHED = "TASK_FINISHED"
SERVICE_STATUS = "SERVICE_STATUS"


@dataclass(frozen=True)
class MissionEvent:
    event: str
    point_id: str
    target_index: int
    action: str
    timestamp: float = field(default_factory=time.time)
    details: Mapping[str, object] = field(default_factory=dict)

    def as_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["details"] = dict(self.details)
        return result


MissionEventHandler = Callable[[MissionEvent], None]


class MissionEventBus:
    """Dispatch events away from the flight-control thread.

    ``publish`` never waits. If consumers cannot keep up, the newest event is
    dropped and ``dropped_events`` is incremented for diagnostics.
    """

    _STOP = object()

    def __init__(self, max_queue_size: int = 256):
        if max_queue_size <= 0:
            raise ValueError("max_queue_size must be positive")
        self._queue: queue.Queue[object] = queue.Queue(max_queue_size)
        self._handlers: list[MissionEventHandler] = []
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.dropped_events = 0

    def subscribe(self, handler: MissionEventHandler) -> None:
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("handlers must be registered before start")
            self._handlers.append(handler)

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self._thread = threading.Thread(
                target=self._run, name="mission-events", daemon=True
            )
            self._thread.start()

    def publish(self, event: MissionEvent) -> bool:
        try:
            self._queue.put_nowait(event)
            return True
        except queue.Full:
            self.dropped_events += 1
            return False

    def close(self, timeout_s: float = 2.0) -> None:
        thread = self._thread
        if thread is None:
            return
        try:
            self._queue.put(self._STOP, timeout=max(0.0, timeout_s))
        except queue.Full:
            return
        thread.join(timeout=max(0.0, timeout_s))

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._STOP:
                    return
                if not isinstance(item, MissionEvent):
                    continue
                for handler in tuple(self._handlers):
                    try:
                        handler(item)
                    except Exception:
                        # A recorder or future video consumer must never stop
                        # delivery to the remaining consumers.
                        continue
            finally:
                self._queue.task_done()
