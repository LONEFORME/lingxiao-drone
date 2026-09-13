"""任务二无人机坐标遥测与阶段事件发布。

与任务一的区别：DYNAMIC_LANDING 阶段会根据 DynamicLandingController 的
内部状态进一步细分为 DESCEND / TERMINAL_PREDICT / TOUCHDOWN / DECK_RIDE /
CONTROLLED_ABORT，让小车端能看到动态降落进度并配合。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    MessageType,
    UavPhase,
    pack_payload,
)

from .dynamic_landing import LandingState
from .task2_mission import Task2Phase


@dataclass(frozen=True)
class Task2TelemetrySample:
    phase: Task2Phase
    base_state: str
    position_xyz_m: tuple[float, float, float]
    landing_state: LandingState | None = None
    mission_success: bool = False


# Task2Phase → UavPhase（DYNAMIC_LANDING 单独处理）
_PHASE_MAP = {
    Task2Phase.WAIT_START: UavPhase.READY,
    Task2Phase.TAKEOFF: UavPhase.TAKEOFF,
    Task2Phase.HOLD_3S: UavPhase.HOVER,
    Task2Phase.INTERCEPT_B_PRE: UavPhase.INTERCEPT,
    Task2Phase.ACQUIRE_TARGET: UavPhase.SEARCH_TARGET,
    Task2Phase.FOLLOW_B_C: UavPhase.FORMATION_FOLLOW,
    Task2Phase.TRANSIT_C: UavPhase.INTERCEPT,
    Task2Phase.SYNC_TARGET_AT_C: UavPhase.FORMATION_FOLLOW,
    Task2Phase.OPEN_LOOP_C_D: UavPhase.DESCEND,
    Task2Phase.SAFE_HOVER_D: UavPhase.HOVER,
    Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF: UavPhase.HOVER,
    Task2Phase.LANDED_ON_PLATFORM: UavPhase.COMPLETE,
    Task2Phase.ACTIVATE_TRACKER: UavPhase.DESCEND,
    Task2Phase.RETAKEOFF: UavPhase.RETAKEOFF,
    Task2Phase.STABILIZE_AFTER_RETAKEOFF: UavPhase.HOVER,
    Task2Phase.VISUAL_RETURN_A: UavPhase.FORMATION_FOLLOW,
    Task2Phase.CLIMB_150CM: UavPhase.RETURN_H,
    Task2Phase.RETURN_H: UavPhase.RETURN_H,
    Task2Phase.LAND_H: UavPhase.LAND_H,
    Task2Phase.COMPLETE: UavPhase.COMPLETE,
}

# DYNAMIC_LANDING 内部 LandingState → UavPhase
_LANDING_STATE_MAP = {
    LandingState.LANDING_GATE: UavPhase.DESCEND,
    LandingState.DESCEND_HIGH: UavPhase.DESCEND,
    LandingState.DESCEND_MID: UavPhase.DESCEND,
    LandingState.DESCEND_LOW: UavPhase.DESCEND,
    LandingState.TERMINAL_PREDICT: UavPhase.TERMINAL_PREDICT,
    LandingState.TOUCHDOWN_CANDIDATE: UavPhase.TOUCHDOWN,
    LandingState.DECK_RIDE: UavPhase.DECK_RIDE,
    LandingState.RETAKEOFF_GATE: UavPhase.RETAKEOFF,
    LandingState.CONTROLLED_ABORT: UavPhase.CONTROLLED_ABORT,
}


class Task2TelemetryPublisher:
    def __init__(
        self,
        link,
        *,
        session_id: int,
        state_interval_s: float = 0.10,
        clock=time.monotonic,
    ) -> None:
        self.link = link
        self.session_id = int(session_id)
        self.state_interval_s = max(0.05, float(state_interval_s))
        self.clock = clock
        self.started_at = self.clock()
        self._last_state_at: float | None = None
        self._last_event_phase: UavPhase | None = None
        self._complete_sent = False

    def update(self, sample: Task2TelemetrySample) -> bool:
        now = self.clock()
        state_sent = False
        if (
            self._last_state_at is None
            or now - self._last_state_at >= self.state_interval_s
        ):
            state_sent = self._publish_state(sample, now)

        phase = self._phase(sample)
        event_sent = False
        if phase != self._last_event_phase:
            event_sent = self._publish_event(phase, now)
        return state_sent or event_sent

    def finish(self, *, mission_success: bool, faulted: bool) -> bool:
        del mission_success
        if faulted:
            return False
        if self._complete_sent:
            return True
        self._complete_sent = self._publish_event(
            UavPhase.COMPLETE, self.clock()
        )
        return self._complete_sent

    def _publish_event(self, phase: UavPhase, now: float) -> bool:
        sent = (
            self.link.publish(
                MessageType.UAV_EVENT,
                pack_payload(
                    MessageType.UAV_EVENT,
                    (int(phase), self._elapsed_ms(now)),
                ),
                session_id=self.session_id,
                dest=Device.CAR,
                flags=Flag.ACK_REQUIRED | Flag.EVENT,
            )
            is not None
        )
        if sent:
            self._last_event_phase = phase
            if phase == UavPhase.COMPLETE:
                self._complete_sent = True
        return sent

    def _publish_state(
        self,
        sample: Task2TelemetrySample,
        now: float,
    ) -> bool:
        x, y, z = sample.position_xyz_m
        payload = pack_payload(
            MessageType.UAV_STATE,
            (
                _scaled_i32(x, 1000.0),
                _scaled_i32(y, 1000.0),
                _scaled_i16(z, 1000.0),
            ),
        )
        sent = (
            self.link.publish(
                MessageType.UAV_STATE,
                payload,
                session_id=self.session_id,
                dest=Device.CAR,
            )
            is not None
        )
        if sent:
            self._last_state_at = now
        return sent

    @staticmethod
    def _phase(sample: Task2TelemetrySample) -> UavPhase:
        # base_state DESCEND/LAND（basic 两级降落接管）优先映射到 LAND_H
        if sample.base_state in ("DESCEND", "LAND"):
            return UavPhase.LAND_H
        if sample.phase == Task2Phase.DYNAMIC_LANDING:
            if sample.landing_state is not None:
                return _LANDING_STATE_MAP.get(
                    sample.landing_state, UavPhase.DESCEND
                )
            return UavPhase.DESCEND
        return _PHASE_MAP[sample.phase]

    def _elapsed_ms(self, now: float) -> int:
        return max(0, min(0xFFFFFFFF, int((now - self.started_at) * 1000)))


def _clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, int(value)))


def _scaled_i16(value: float, scale: float) -> int:
    return _clamp(round(float(value) * scale), -0x8000, 0x7FFF)


def _scaled_i32(value: float, scale: float) -> int:
    return _clamp(
        round(float(value) * scale),
        -0x80000000,
        0x7FFFFFFF,
    )
