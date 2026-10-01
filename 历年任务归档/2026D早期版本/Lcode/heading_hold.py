"""基于 T265 的起飞航向保持外环。

控制器只生成整数 yaw 角速度指令（deg/s）；凌霄 IMU 继续负责内层角速度闭环。
默认开启，可用 DRONE_HEADING_HOLD=0 显式关闭并恢复零 yaw 指令。
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math
import os
from typing import Optional


def wrap_degrees(angle_deg: float) -> float:
    """把角度归一化到 [-180, 180)。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class HeadingHoldConfig:
    enabled: bool = True
    kp: float = 0.25
    deadband_deg: float = 1.5
    max_rate_dps: int = 1
    fault_error_deg: float = 8.0
    runaway_window_s: float = 1.0
    runaway_growth_deg: float = 3.0

    def __post_init__(self) -> None:
        if not 0.0 < self.kp <= 0.5:
            raise ValueError("heading hold kp 必须在 (0, 0.5] 内")
        if not 0.5 <= self.deadband_deg <= 5.0:
            raise ValueError("heading hold deadband_deg 必须在 [0.5, 5.0] 内")
        if not 1 <= self.max_rate_dps <= 3:
            raise ValueError("heading hold max_rate_dps 必须在 [1, 3] 内")
        if self.fault_error_deg <= self.deadband_deg:
            raise ValueError("heading hold fault_error_deg 必须大于死区")
        if self.runaway_window_s <= 0 or self.runaway_growth_deg <= 0:
            raise ValueError("heading hold runaway 参数必须为正数")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None) -> "HeadingHoldConfig":
        env = os.environ if environ is None else environ
        enabled_text = env.get("DRONE_HEADING_HOLD", "1").strip().lower()
        if enabled_text not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_HEADING_HOLD 只能是 0/1/false/true")
        return cls(
            enabled=enabled_text in {"1", "true"},
            kp=float(env.get("DRONE_HEADING_HOLD_KP", "0.25")),
            deadband_deg=float(env.get("DRONE_HEADING_HOLD_DEADBAND_DEG", "1.5")),
            max_rate_dps=int(env.get("DRONE_HEADING_HOLD_MAX_DPS", "1")),
        )


@dataclass(frozen=True)
class HeadingHoldStatus:
    command_dps: int
    enabled: bool
    armed: bool
    target_deg: Optional[float]
    current_deg: Optional[float]
    error_deg: Optional[float]
    degraded_reason: Optional[str]
    fault_reason: Optional[str]


class HeadingHoldController:
    def __init__(self, config: HeadingHoldConfig) -> None:
        self.config = config
        self.armed = False
        self.target_deg: Optional[float] = None
        self.fault_reason: Optional[str] = None
        self.degraded_reason: Optional[str] = None
        self._runaway_sign = 0
        self._runaway_start_time: Optional[float] = None
        self._runaway_start_abs_error: Optional[float] = None

    def reset_for_new_mission(self) -> None:
        self.armed = False
        self.target_deg = None
        self.fault_reason = None
        self.degraded_reason = None
        self._reset_runaway_window()

    def arm(self, current_yaw_rad: float, now: float) -> HeadingHoldStatus:
        del now
        current_deg = self._yaw_rad_to_deg(current_yaw_rad)
        if not self.config.enabled:
            self.degraded_reason = "disabled"
            return self._status(0, current_deg, None)
        if self.fault_reason is not None:
            return self._status(0, current_deg, None)
        if not self.armed:
            self.target_deg = current_deg
            self.armed = True
            self.degraded_reason = None
            self._reset_runaway_window()
        return self._status(0, current_deg, wrap_degrees(self.target_deg - current_deg))

    def disarm(self, reason: str) -> None:
        self.armed = False
        self.degraded_reason = reason
        self._reset_runaway_window()

    def update(self, current_yaw_rad: float, confidence: int, now: float) -> HeadingHoldStatus:
        current_deg = self._yaw_rad_to_deg(current_yaw_rad)
        if not self.config.enabled:
            self.degraded_reason = "disabled"
            return self._status(0, current_deg, None)
        if self.fault_reason is not None:
            return self._status(0, current_deg, self._error_for(current_deg))
        if not self.armed or self.target_deg is None:
            self.degraded_reason = "not_armed"
            self._reset_runaway_window()
            return self._status(0, current_deg, None)
        if confidence < 2:
            self.degraded_reason = "low_confidence"
            self._reset_runaway_window()
            return self._status(0, current_deg, self._error_for(current_deg))

        error_deg = self._error_for(current_deg)
        self.degraded_reason = None
        if abs(error_deg) >= self.config.fault_error_deg:
            return self._latch_fault(
                f"heading_error_{error_deg:+.2f}deg_exceeds_limit", current_deg, error_deg
            )
        command_dps = self._command_for_error(error_deg)
        if self._runaway_detected(command_dps, error_deg, now):
            return self._latch_fault(
                f"heading_error_grew_{self.config.runaway_growth_deg:.1f}deg_"
                f"in_{self.config.runaway_window_s:.1f}s",
                current_deg,
                error_deg,
            )
        return self._status(command_dps, current_deg, error_deg)

    def _command_for_error(self, error_deg: float) -> int:
        if abs(error_deg) <= self.config.deadband_deg:
            return 0
        magnitude = max(1, math.floor(abs(self.config.kp * error_deg) + 0.5))
        magnitude = min(magnitude, self.config.max_rate_dps)
        return magnitude if error_deg > 0 else -magnitude

    def _runaway_detected(self, command_dps: int, error_deg: float, now: float) -> bool:
        # 比例控制尚未达到上限时，误差增长不等于控制失效；只有持续施加
        # 最大修正后误差仍扩大，才判定为 runaway。
        if abs(command_dps) < self.config.max_rate_dps:
            self._reset_runaway_window()
            return False
        sign = 1 if command_dps > 0 else -1
        abs_error = abs(error_deg)
        if self._runaway_sign != sign or self._runaway_start_time is None:
            self._runaway_sign = sign
            self._runaway_start_time = now
            self._runaway_start_abs_error = abs_error
            return False
        if now - self._runaway_start_time < self.config.runaway_window_s:
            return False
        start_error = self._runaway_start_abs_error or 0.0
        grew = abs_error - start_error >= self.config.runaway_growth_deg
        if not grew:
            self._runaway_start_time = now
            self._runaway_start_abs_error = abs_error
        return grew

    def _latch_fault(self, reason: str, current_deg: float, error_deg: float) -> HeadingHoldStatus:
        self.fault_reason = reason
        self.degraded_reason = None
        self._reset_runaway_window()
        return self._status(0, current_deg, error_deg)

    def _error_for(self, current_deg: float) -> Optional[float]:
        if self.target_deg is None:
            return None
        return wrap_degrees(self.target_deg - current_deg)

    def _status(
        self,
        command_dps: int,
        current_deg: Optional[float],
        error_deg: Optional[float],
    ) -> HeadingHoldStatus:
        return HeadingHoldStatus(
            command_dps=command_dps,
            enabled=self.config.enabled,
            armed=self.armed,
            target_deg=self.target_deg,
            current_deg=current_deg,
            error_deg=error_deg,
            degraded_reason=self.degraded_reason,
            fault_reason=self.fault_reason,
        )

    def _reset_runaway_window(self) -> None:
        self._runaway_sign = 0
        self._runaway_start_time = None
        self._runaway_start_abs_error = None

    @staticmethod
    def _yaw_rad_to_deg(yaw_rad: float) -> float:
        if not math.isfinite(yaw_rad):
            raise ValueError("yaw 必须是有限数")
        return wrap_degrees(math.degrees(yaw_rad))
