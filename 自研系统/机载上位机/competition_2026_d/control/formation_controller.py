"""小车速度前馈 + 相对位置/速度反馈的无副作用伴飞控制器。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..vision.platform_tracker import PlatformEstimate


@dataclass(frozen=True)
class FormationConfig:
    kp: float = 0.75
    kd: float = 0.22
    max_speed_m_s: float = 0.40
    max_accel_m_s2: float = 0.45
    max_jerk_m_s3: float = 1.2
    max_estimate_age_s: float = 0.20
    max_uncertainty_m: float = 0.30
    position_deadband_m: float = 0.0
    target_velocity_feedforward_gain: float = 1.0


@dataclass(frozen=True)
class FormationCommand:
    vx_m_s: float
    vy_m_s: float
    valid: bool
    reason: str


class FormationController:
    def __init__(self, config: FormationConfig | None = None) -> None:
        self.config = config or FormationConfig()
        self._last_time: float | None = None
        self._last_velocity = (0.0, 0.0)
        self._last_accel = (0.0, 0.0)

    def reset(self, timestamp: float | None = None, velocity=(0.0, 0.0)) -> None:
        self._last_time = timestamp
        self._last_velocity = (float(velocity[0]), float(velocity[1]))
        self._last_accel = (0.0, 0.0)

    def command(
        self,
        estimate: PlatformEstimate | None,
        drone_position_xy: tuple[float, float],
        drone_velocity_xy: tuple[float, float],
        timestamp: float,
        desired_offset_xy: tuple[float, float] = (0.0, 0.0),
    ) -> FormationCommand:
        cfg = self.config
        if estimate is None:
            return self._brake(timestamp, "no_estimate")
        if timestamp - estimate.timestamp > cfg.max_estimate_age_s:
            return self._brake(timestamp, "stale_estimate")
        if estimate.uncertainty_m > cfg.max_uncertainty_m:
            return self._brake(timestamp, "uncertain_estimate")
        ex = estimate.x_m + desired_offset_xy[0] - drone_position_xy[0]
        ey = estimate.y_m + desired_offset_xy[1] - drone_position_xy[1]
        if math.hypot(ex, ey) <= cfg.position_deadband_m:
            ex, ey = 0.0, 0.0
        feedforward_vx = cfg.target_velocity_feedforward_gain * estimate.vx_m_s
        feedforward_vy = cfg.target_velocity_feedforward_gain * estimate.vy_m_s
        evx = feedforward_vx - drone_velocity_xy[0]
        evy = feedforward_vy - drone_velocity_xy[1]
        target_vx = feedforward_vx + cfg.kp * ex + cfg.kd * evx
        target_vy = feedforward_vy + cfg.kp * ey + cfg.kd * evy
        target_vx, target_vy = self._limit_norm(target_vx, target_vy, cfg.max_speed_m_s)
        vx, vy = self._shape_velocity(target_vx, target_vy, timestamp)
        return FormationCommand(vx, vy, True, "ok")

    def _brake(self, timestamp: float, reason: str) -> FormationCommand:
        vx, vy = self._shape_velocity(0.0, 0.0, timestamp)
        return FormationCommand(vx, vy, False, reason)

    def shape_velocity(
        self, target_vx: float, target_vy: float, timestamp: float
    ) -> FormationCommand:
        """给粗跟踪等外部模式复用同一速度/加速度/jerk约束。"""
        target_vx, target_vy = self._limit_norm(
            target_vx, target_vy, self.config.max_speed_m_s
        )
        vx, vy = self._shape_velocity(target_vx, target_vy, timestamp)
        return FormationCommand(vx, vy, True, "shaped_target")

    def _shape_velocity(
        self, target_vx: float, target_vy: float, timestamp: float
    ) -> tuple[float, float]:
        cfg = self.config
        if self._last_time is None or timestamp <= self._last_time:
            self.reset(timestamp, self._last_velocity)
            return self._last_velocity
        dt = min(timestamp - self._last_time, 0.2)
        desired_ax = (target_vx - self._last_velocity[0]) / dt
        desired_ay = (target_vy - self._last_velocity[1]) / dt
        desired_ax, desired_ay = self._limit_norm(desired_ax, desired_ay, cfg.max_accel_m_s2)
        max_da = cfg.max_jerk_m_s3 * dt
        ax = self._last_accel[0] + max(-max_da, min(max_da, desired_ax - self._last_accel[0]))
        ay = self._last_accel[1] + max(-max_da, min(max_da, desired_ay - self._last_accel[1]))
        ax, ay = self._limit_norm(ax, ay, cfg.max_accel_m_s2)
        vx = self._last_velocity[0] + ax * dt
        vy = self._last_velocity[1] + ay * dt
        vx, vy = self._limit_norm(vx, vy, cfg.max_speed_m_s)
        self._last_time = timestamp
        self._last_velocity = (vx, vy)
        self._last_accel = (ax, ay)
        return vx, vy

    @staticmethod
    def _limit_norm(x: float, y: float, limit: float) -> tuple[float, float]:
        norm = math.hypot(x, y)
        if norm <= limit or norm <= 1e-9:
            return x, y
        scale = limit / norm
        return x * scale, y * scale
