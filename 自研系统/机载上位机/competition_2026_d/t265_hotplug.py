"""T265拔插后的非阻塞初始化与就绪门禁。"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from typing import Callable


class T265State(enum.Enum):
    DISARMED = "disarmed"
    WAIT_HOTPLUG = "wait_hotplug"
    INITIALIZING = "initializing"
    WAIT_CONFIDENCE = "wait_confidence"
    READY = "ready"
    FAILED = "failed"


@dataclass(frozen=True)
class T265HotplugConfig:
    confidence_min: int = 2
    confidence_hold_s: float = 1.0
    init_timeout_s: float = 8.0
    hotplug_timeout_s: float = 60.0


class T265HotplugManager:
    """调用arm()前绝不创建T265管线，适合上电自启动程序。"""

    def __init__(
        self,
        device_present: Callable[[], bool],
        t265_factory: Callable[[], object],
        config: T265HotplugConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.device_present = device_present
        self.t265_factory = t265_factory
        self.config = config or T265HotplugConfig()
        self.clock = clock
        self.state = T265State.DISARMED
        self.sensor = None
        self.reason = ""
        self._state_since = self.clock()
        self._confidence_since: float | None = None

    def arm(self) -> None:
        if self.state not in (T265State.DISARMED, T265State.FAILED):
            return
        self.sensor = None
        self.reason = ""
        self.state = T265State.WAIT_HOTPLUG
        self._state_since = self.clock()
        self._confidence_since = None

    def poll(self) -> T265State:
        now = self.clock()
        if self.state == T265State.WAIT_HOTPLUG:
            if now - self._state_since > self.config.hotplug_timeout_s:
                return self._fail("hotplug_timeout")
            if self.device_present():
                self.state = T265State.INITIALIZING
                self._state_since = now
        if self.state == T265State.INITIALIZING:
            try:
                self.sensor = self.t265_factory()
                if not bool(self.sensor.start()):
                    return self._fail("start_failed")
            except Exception as exc:
                return self._fail(f"start_exception:{type(exc).__name__}")
            self.state = T265State.WAIT_CONFIDENCE
            self._state_since = now
        if self.state == T265State.WAIT_CONFIDENCE:
            if now - self._state_since > self.config.init_timeout_s:
                return self._fail("confidence_timeout")
            confidence = int(getattr(self.sensor, "last_confidence", 0))
            if confidence >= self.config.confidence_min:
                if self._confidence_since is None:
                    self._confidence_since = now
                elif now - self._confidence_since >= self.config.confidence_hold_s:
                    self.state = T265State.READY
                    self._state_since = now
            else:
                self._confidence_since = None
        return self.state

    @property
    def ready(self) -> bool:
        return self.state == T265State.READY

    def shutdown(self) -> None:
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except Exception:
                pass
        self.sensor = None
        self.state = T265State.DISARMED
        self._state_since = self.clock()

    def _fail(self, reason: str) -> T265State:
        self.reason = reason
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except Exception:
                pass
        self.sensor = None
        self.state = T265State.FAILED
        self._state_since = self.clock()
        return self.state
