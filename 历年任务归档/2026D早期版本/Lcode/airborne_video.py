"""Lifecycle manager for optional airborne camera-to-publisher streaming."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import queue
import threading
import time
from typing import Mapping, Optional

from Lcode.video_source import (
    VideoPublisher,
    VideoPublisherConfig,
    VideoSource,
    VideoSourceConfig,
)


class AirborneVideoError(RuntimeError):
    pass


@dataclass(frozen=True)
class AirborneVideoConfig:
    enabled: bool = False
    required: bool = False
    source: VideoSourceConfig = VideoSourceConfig()
    publisher: VideoPublisherConfig = VideoPublisherConfig()
    max_fps: float = 5.0
    read_timeout_s: float = 0.5
    start_timeout_s: float = 3.0
    stop_timeout_s: float = 0.5
    max_consecutive_failures: int = 5

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "AirborneVideoConfig":
        if raw is None:
            return cls()
        if not isinstance(raw, Mapping):
            raise AirborneVideoError("airborne_video must be an object")
        enabled = _bool(raw.get("enabled", False), "enabled")
        required = _bool(raw.get("required", False), "required")
        config = cls(
            enabled=enabled,
            required=required,
            source=VideoSourceConfig.from_mapping(raw.get("source")),
            publisher=VideoPublisherConfig.from_mapping(raw.get("publisher")),
            max_fps=_positive_float(raw.get("max_fps", 5.0), "max_fps"),
            read_timeout_s=_positive_float(
                raw.get("read_timeout_s", 0.5), "read_timeout_s"
            ),
            start_timeout_s=_positive_float(
                raw.get("start_timeout_s", 3.0), "start_timeout_s"
            ),
            stop_timeout_s=_positive_float(
                raw.get("stop_timeout_s", 0.5), "stop_timeout_s"
            ),
            max_consecutive_failures=_positive_int(
                raw.get("max_consecutive_failures", 5),
                "max_consecutive_failures",
            ),
        )
        if required and not enabled:
            raise AirborneVideoError(
                "required airborne_video must also be enabled"
            )
        if enabled and (not config.source.enabled or not config.publisher.enabled):
            raise AirborneVideoError(
                "enabled airborne_video requires enabled source and publisher"
            )
        return config

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "required": self.required,
            "source": self.source.as_dict(),
            "publisher": self.publisher.as_dict(),
            "max_fps": self.max_fps,
            "read_timeout_s": self.read_timeout_s,
            "start_timeout_s": self.start_timeout_s,
            "stop_timeout_s": self.stop_timeout_s,
            "max_consecutive_failures": self.max_consecutive_failures,
        }


def load_airborne_video_config(path: str | Path) -> AirborneVideoConfig:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise AirborneVideoError(f"cannot load airborne_video config: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise AirborneVideoError("competition config root must be an object")
    return AirborneVideoConfig.from_mapping(raw.get("airborne_video"))


class AirborneVideoManager:
    def __init__(
        self,
        config: AirborneVideoConfig,
        source: VideoSource,
        publisher: VideoPublisher,
    ):
        self.config = config
        self.source = source
        self.publisher = publisher
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.last_error: Optional[str] = None
        self.startup_timed_out = False
        self._stats = {
            "frames_read": 0,
            "frames_published": 0,
            "failures": 0,
            "circuit_open": False,
        }

    @property
    def ready(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def start(self) -> bool:
        source_call = _bounded_call(self.source.start, self.config.start_timeout_s)
        if not source_call.completed:
            self.startup_timed_out = True
            self.last_error = "video_source_start_timeout"
            return False
        if source_call.error is not None or not source_call.value:
            self.last_error = (
                f"video_source_start_error:{source_call.error}"
                if source_call.error
                else "video_source_start_failed"
            )
            return False

        publisher_call = _bounded_call(
            self.publisher.start, self.config.start_timeout_s
        )
        if not publisher_call.completed:
            self.startup_timed_out = True
            self.last_error = "video_publisher_start_timeout"
            _bounded_call(self.source.stop, self.config.stop_timeout_s)
            return False
        if publisher_call.error is not None or not publisher_call.value:
            self.last_error = (
                f"video_publisher_start_error:{publisher_call.error}"
                if publisher_call.error
                else "video_publisher_start_failed"
            )
            _bounded_call(self.source.stop, self.config.stop_timeout_s)
            return False

        self._thread = threading.Thread(
            target=self._run, name="airborne-video", daemon=True
        )
        self._thread.start()
        return True

    def stop(self) -> bool:
        self._stop_event.set()
        _bounded_call(self.source.stop, self.config.stop_timeout_s)
        thread = self._thread
        if thread is not None:
            thread.join(self.config.stop_timeout_s)
        _bounded_call(self.publisher.stop, self.config.stop_timeout_s)
        return thread is None or not thread.is_alive()

    def stats(self) -> dict[str, object]:
        with self._lock:
            return dict(self._stats)

    def _run(self) -> None:
        interval = 1.0 / self.config.max_fps
        consecutive_failures = 0
        while not self._stop_event.is_set():
            cycle_started = time.monotonic()
            try:
                frame = self.source.read_frame(self.config.read_timeout_s)
            except Exception as exc:
                frame = None
                self.last_error = f"video_read_exception:{exc}"
            if frame is None:
                consecutive_failures += 1
                self._record_failure()
            else:
                with self._lock:
                    self._stats["frames_read"] += 1
                try:
                    published = bool(self.publisher.publish_frame(frame))
                except Exception as exc:
                    published = False
                    self.last_error = f"video_publish_exception:{exc}"
                if published:
                    consecutive_failures = 0
                    with self._lock:
                        self._stats["frames_published"] += 1
                else:
                    consecutive_failures += 1
                    self._record_failure()
            if consecutive_failures >= self.config.max_consecutive_failures:
                with self._lock:
                    self._stats["circuit_open"] = True
                self.last_error = self.last_error or "video_failure_circuit_open"
                return
            remaining = interval - (time.monotonic() - cycle_started)
            if remaining > 0:
                self._stop_event.wait(remaining)
            else:
                time.sleep(0)

    def _record_failure(self) -> None:
        with self._lock:
            self._stats["failures"] += 1


@dataclass(frozen=True)
class _BoundedCallResult:
    completed: bool
    value: object = None
    error: Optional[BaseException] = None


def _bounded_call(function, timeout_s: float) -> _BoundedCallResult:
    result_queue: queue.Queue[tuple[object, Optional[BaseException]]] = queue.Queue(1)

    def invoke() -> None:
        try:
            result_queue.put((function(), None))
        except BaseException as exc:
            result_queue.put((None, exc))

    thread = threading.Thread(target=invoke, name="video-lifecycle-call", daemon=True)
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        return _BoundedCallResult(False)
    value, error = result_queue.get_nowait()
    return _BoundedCallResult(True, value, error)


def _bool(value: object, field: str) -> bool:
    if not isinstance(value, bool):
        raise AirborneVideoError(f"airborne_video.{field} must be boolean")
    return value


def _positive_int(value: object, field: str) -> int:
    if isinstance(value, bool):
        raise AirborneVideoError(f"airborne_video.{field} must be positive")
    try:
        normalized = int(value)
    except (TypeError, ValueError) as exc:
        raise AirborneVideoError(f"airborne_video.{field} must be positive") from exc
    if normalized <= 0 or normalized != value:
        raise AirborneVideoError(f"airborne_video.{field} must be positive")
    return normalized


def _positive_float(value: object, field: str) -> float:
    try:
        normalized = float(value)
    except (TypeError, ValueError) as exc:
        raise AirborneVideoError(f"airborne_video.{field} must be positive") from exc
    if normalized <= 0:
        raise AirborneVideoError(f"airborne_video.{field} must be positive")
    return normalized
