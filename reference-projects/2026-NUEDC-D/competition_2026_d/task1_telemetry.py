"""任务一无人机坐标遥测与阶段事件发布。"""

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

from .task1_mission import Task1Phase


@dataclass(frozen=True)
class Task1TelemetrySample:
    phase: Task1Phase
    base_state: str
    position_xyz_m: tuple[float, float, float]


_PHASE_MAP = {
    Task1Phase.WAIT_START: UavPhase.READY,
    Task1Phase.TAKEOFF: UavPhase.TAKEOFF,
    Task1Phase.HOLD_3S: UavPhase.HOVER,
    Task1Phase.INTERCEPT_B_PRE: UavPhase.INTERCEPT,
    Task1Phase.ACQUIRE_TARGET: UavPhase.SEARCH_TARGET,
    Task1Phase.FOLLOW_B_C: UavPhase.FORMATION_FOLLOW,
    Task1Phase.SYNC_TARGET_AT_C: UavPhase.FORMATION_FOLLOW,
    Task1Phase.DROP_WINDOW_C_D: UavPhase.FORMATION_FOLLOW,
    Task1Phase.DROP_DESCENT: UavPhase.DESCEND,
    Task1Phase.RELEASING: UavPhase.DROP,
    Task1Phase.CLIMB: UavPhase.RETURN_H,
    Task1Phase.RETURN_H: UavPhase.RETURN_H,
    Task1Phase.LAND_H: UavPhase.LAND_H,
    Task1Phase.COMPLETE: UavPhase.COMPLETE,
}


class Task1TelemetryPublisher:
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

    def update(self, sample: Task1TelemetrySample) -> bool:
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
        sample: Task1TelemetrySample,
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
    def _phase(sample: Task1TelemetrySample) -> UavPhase:
        if sample.base_state in ("DESCEND", "LAND"):
            return UavPhase.LAND_H
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
