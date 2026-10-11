"""投放稳定窗口与舵机单次动作编排。"""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass
from typing import Callable

from .payload_actuator import ActuatorState, PayloadActuator


class DropState(enum.Enum):
    ARMED = "armed"
    STABLE_WINDOW = "stable_window"
    RELEASING = "releasing"
    RELEASED = "released"
    UNCERTAIN = "uncertain"


@dataclass(frozen=True)
class DropConfig:
    max_position_error_m: float = 0.12
    max_relative_speed_m_s: float = 0.12
    min_quality: int = 65
    min_stable_s: float = 0.30
    min_altitude_m: float = 1.40
    max_altitude_m: float = 1.60


class DropController:
    def __init__(
        self,
        actuator: PayloadActuator,
        config: DropConfig | None = None,
        on_released: Callable[[], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.actuator = actuator
        self.config = config or DropConfig()
        self.on_released = on_released
        self.clock = clock
        self.state = DropState.ARMED
        self._stable_since: float | None = None

    def tick(
        self,
        *,
        vision_fresh: bool,
        vision_quality: int,
        error_xy_m: tuple[float, float],
        relative_velocity_xy_m_s: tuple[float, float],
        altitude_m: float,
        inside_drop_region: bool,
    ) -> DropState:
        if self.state in (DropState.RELEASED, DropState.UNCERTAIN):
            return self.state
        if self.state == DropState.RELEASING:
            result = self.actuator.poll()
            if result == ActuatorState.RELEASED:
                self.state = DropState.RELEASED
                if self.on_released is not None:
                    self.on_released()
            elif result == ActuatorState.UNCERTAIN:
                self.state = DropState.UNCERTAIN
            return self.state
        cfg = self.config
        stable = (
            inside_drop_region
            and vision_fresh
            and vision_quality >= cfg.min_quality
            and math.hypot(*error_xy_m) <= cfg.max_position_error_m
            and math.hypot(*relative_velocity_xy_m_s) <= cfg.max_relative_speed_m_s
            and cfg.min_altitude_m <= altitude_m <= cfg.max_altitude_m
        )
        now = self.clock()
        if not stable:
            self._stable_since = None
            self.state = DropState.ARMED
            return self.state
        if self._stable_since is None:
            self._stable_since = now
            self.state = DropState.STABLE_WINDOW
            return self.state
        if now - self._stable_since >= cfg.min_stable_s and self.actuator.release_once():
            self.state = DropState.RELEASING
        return self.state
