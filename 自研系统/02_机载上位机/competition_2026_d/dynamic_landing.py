"""动态降落安全状态机，包含近地限时预测而非长期开环。"""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass
from typing import Callable


class LandingState(enum.Enum):
    LANDING_GATE = "landing_gate"
    DESCEND_HIGH = "descend_high"
    DESCEND_MID = "descend_mid"
    DESCEND_LOW = "descend_low"
    TERMINAL_PREDICT = "terminal_predict"
    TOUCHDOWN_CANDIDATE = "touchdown_candidate"
    DECK_RIDE = "deck_ride"
    RETAKEOFF_GATE = "retakeoff_gate"
    CONTROLLED_ABORT = "controlled_abort"


@dataclass(frozen=True)
class LandingConfig:
    mid_height_m: float = 0.80
    low_height_m: float = 0.35
    visual_min_height_m: float = 0.16
    contact_height_m: float = 0.10
    descend_high_m_s: float = 0.30
    descend_mid_m_s: float = 0.20
    descend_low_m_s: float = 0.08
    descend_slew_m_s2: float = 0.40
    reacquire_climb_m_s: float = 0.12
    terminal_max_s: float = 0.50
    terminal_max_drop_m: float = 0.10
    terminal_max_error_m: float = 0.10
    terminal_max_relative_speed_m_s: float = 0.10
    terminal_max_uncertainty_m: float = 0.08
    touchdown_height_margin_m: float = 0.03
    touchdown_max_vz_m_s: float = 0.08
    touchdown_max_relative_speed_m_s: float = 0.10
    touchdown_max_tilt_deg: float = 8.0
    touchdown_hold_s: float = 0.40
    deck_ride_s: float = 5.0
    max_terminal_retries: int = 1
    # 联合降落测试专用：上层完成视觉触发后持续下降到激光触地，
    # 不再因视觉短时丢失或终端预测限时而暂停/爬升。
    direct_descent_after_gate: bool = False


@dataclass(frozen=True)
class LandingInput:
    relative_height_m: float
    vertical_speed_m_s: float
    relative_velocity_xy_m_s: tuple[float, float]
    position_error_xy_m: tuple[float, float]
    estimate_uncertainty_m: float
    visual_usable: bool
    visual_too_close: bool
    car_motion_fresh: bool
    roll_deg: float
    pitch_deg: float
    contact_evidence: bool
    t265_healthy: bool
    abort_requested: bool = False


@dataclass(frozen=True)
class LandingCommand:
    state: LandingState
    vertical_speed_m_s: float
    allow_horizontal_vision: bool
    hold_car_velocity: bool
    touchdown_confirmed: bool = False
    reason: str = ""


class DynamicLandingController:
    def __init__(
        self,
        config: LandingConfig | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config or LandingConfig()
        self.clock = clock
        self.reset()

    def reset(self) -> None:
        self.state = LandingState.LANDING_GATE
        self._state_since = self.clock()
        self._terminal_start_height = 0.0
        self._terminal_retries = 0
        self._touchdown_since: float | None = None

    def tick(self, data: LandingInput) -> LandingCommand:
        now = self.clock()
        cfg = self.config
        if data.abort_requested or (not data.t265_healthy and self.state != LandingState.TERMINAL_PREDICT):
            return self._transition(LandingState.CONTROLLED_ABORT, 0.0, False, False, "abort_or_t265")
        if self.state == LandingState.LANDING_GATE:
            gate = cfg.direct_descent_after_gate or (
                data.visual_usable
                and math.hypot(*data.position_error_xy_m) <= 0.12
                and math.hypot(*data.relative_velocity_xy_m_s) <= 0.12
            )
            if not gate:
                return LandingCommand(self.state, 0.0, True, False, reason="gate_not_stable")
            self._set_state(LandingState.DESCEND_HIGH, now)
        if self.state in (LandingState.DESCEND_HIGH, LandingState.DESCEND_MID, LandingState.DESCEND_LOW):
            if cfg.direct_descent_after_gate:
                self.state = LandingState.DESCEND_HIGH
                if self._touchdown_gate(data):
                    if self._touchdown_since is None:
                        self._touchdown_since = now
                    self.state = LandingState.TOUCHDOWN_CANDIDATE
                    return LandingCommand(
                        self.state,
                        0.0,
                        data.visual_usable,
                        True,
                        reason="direct_descent_contact",
                    )
                return LandingCommand(
                    self.state,
                    -cfg.descend_high_m_s,
                    data.visual_usable,
                    True,
                    reason="direct_descent_to_contact",
                )
            if not data.visual_usable:
                if data.visual_too_close and self._terminal_gate(data):
                    self._set_state(LandingState.TERMINAL_PREDICT, now)
                    self._terminal_start_height = data.relative_height_m
                else:
                    return LandingCommand(self.state, cfg.reacquire_climb_m_s, False, False, reason="reacquire")
            else:
                if data.relative_height_m <= cfg.visual_min_height_m and self._terminal_gate(data):
                    self._set_state(LandingState.TERMINAL_PREDICT, now)
                    self._terminal_start_height = data.relative_height_m
                elif data.relative_height_m <= cfg.low_height_m:
                    self.state = LandingState.DESCEND_LOW
                    return LandingCommand(self.state, -cfg.descend_low_m_s, True, True)
                elif data.relative_height_m <= cfg.mid_height_m:
                    self.state = LandingState.DESCEND_MID
                    return LandingCommand(self.state, -cfg.descend_mid_m_s, True, True)
                else:
                    self.state = LandingState.DESCEND_HIGH
                    return LandingCommand(self.state, -cfg.descend_high_m_s, True, True)
        if self.state == LandingState.TERMINAL_PREDICT:
            elapsed = now - self._state_since
            dropped = self._terminal_start_height - data.relative_height_m
            if self._touchdown_gate(data):
                if self._touchdown_since is None:
                    self._touchdown_since = now
                self.state = LandingState.TOUCHDOWN_CANDIDATE
            elif (
                elapsed >= cfg.terminal_max_s
                or dropped >= cfg.terminal_max_drop_m
                or data.estimate_uncertainty_m > cfg.terminal_max_uncertainty_m
                or math.hypot(*data.relative_velocity_xy_m_s) > cfg.terminal_max_relative_speed_m_s
            ):
                if self._terminal_retries < cfg.max_terminal_retries and data.t265_healthy:
                    self._terminal_retries += 1
                    self._set_state(LandingState.DESCEND_LOW, now)
                    return LandingCommand(self.state, cfg.reacquire_climb_m_s, False, False, reason="terminal_reacquire")
                return self._transition(LandingState.CONTROLLED_ABORT, 0.0, False, False, "terminal_limit")
            else:
                return LandingCommand(self.state, -cfg.descend_low_m_s, False, True)
        if self.state == LandingState.TOUCHDOWN_CANDIDATE:
            if not self._touchdown_gate(data):
                self._touchdown_since = None
                if cfg.direct_descent_after_gate:
                    self._set_state(LandingState.DESCEND_HIGH, now)
                    return LandingCommand(
                        self.state,
                        -cfg.descend_high_m_s,
                        data.visual_usable,
                        True,
                        reason="direct_contact_lost_resume_descent",
                    )
                self._set_state(LandingState.TERMINAL_PREDICT, now)
                self._terminal_start_height = data.relative_height_m
                return LandingCommand(self.state, -cfg.descend_low_m_s, False, True, reason="touchdown_rejected")
            if self._touchdown_since is None:
                self._touchdown_since = now
            if now - self._touchdown_since < cfg.touchdown_hold_s:
                return LandingCommand(self.state, 0.0, False, True)
            self._set_state(LandingState.DECK_RIDE, now)
            return LandingCommand(self.state, 0.0, False, True, True, "touchdown_confirmed")
        if self.state == LandingState.DECK_RIDE:
            if now - self._state_since >= cfg.deck_ride_s:
                self._set_state(LandingState.RETAKEOFF_GATE, now)
            return LandingCommand(self.state, 0.0, False, True)
        return LandingCommand(self.state, 0.0, False, False)

    def _terminal_gate(self, data: LandingInput) -> bool:
        cfg = self.config
        return (
            math.hypot(*data.position_error_xy_m) <= cfg.terminal_max_error_m
            and math.hypot(*data.relative_velocity_xy_m_s) <= cfg.terminal_max_relative_speed_m_s
            and data.estimate_uncertainty_m <= cfg.terminal_max_uncertainty_m
            and (data.car_motion_fresh or data.visual_usable)
        )

    def _touchdown_gate(self, data: LandingInput) -> bool:
        cfg = self.config
        if cfg.direct_descent_after_gate:
            return (
                data.relative_height_m
                <= cfg.contact_height_m + cfg.touchdown_height_margin_m
                and data.contact_evidence
            )
        return (
            data.relative_height_m <= cfg.contact_height_m + cfg.touchdown_height_margin_m
            and abs(data.vertical_speed_m_s) <= cfg.touchdown_max_vz_m_s
            and math.hypot(*data.relative_velocity_xy_m_s) <= cfg.touchdown_max_relative_speed_m_s
            and abs(data.roll_deg) <= cfg.touchdown_max_tilt_deg
            and abs(data.pitch_deg) <= cfg.touchdown_max_tilt_deg
            and data.contact_evidence
        )

    def _set_state(self, state: LandingState, now: float) -> None:
        if state != self.state:
            self.state = state
            self._state_since = now

    def _transition(self, state, vz, vision, car_velocity, reason) -> LandingCommand:
        self._set_state(state, self.clock())
        return LandingCommand(state, vz, vision, car_velocity, False, reason)
