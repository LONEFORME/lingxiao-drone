"""任务二（动态降落）运行时辅助层。

包含 offset_HA 标定器和激光触地检测器：前者在开机时实飞标定小车 T265
坐标系到无人机 T265 坐标系的水平 XY 偏差，后者从激光高度计读数判定
是否触地，用于 DynamicLandingController 的 LandingInput.contact_evidence。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class OffsetCalibratorConfig:
    min_samples: int = 10
    max_samples: int = 50
    max_position_jitter_m: float = 0.05
    target_convergence_m: float = 0.02


class OffsetCalibrator:
    """开机时标定小车 T265 到无人机 T265 的水平 XY 偏差（offset_HA）。"""

    def __init__(self, config: OffsetCalibratorConfig | None = None) -> None:
        self.config = config or OffsetCalibratorConfig()
        self._samples: list[tuple[float, float]] = []
        self._offset: tuple[float, float] | None = None
        self._last_reject_reason: str | None = None

    def reset(self) -> None:
        self._samples.clear()
        self._offset = None
        self._last_reject_reason = None

    def record_sample(
        self, car_xy_m: tuple[float, float], uav_xy_m: tuple[float, float]
    ) -> bool:
        """记录一次标定样本，返回本次样本是否被接受。"""
        cfg = self.config
        cx, cy = car_xy_m
        ux, uy = uav_xy_m
        if not (
            math.isfinite(cx)
            and math.isfinite(cy)
            and math.isfinite(ux)
            and math.isfinite(uy)
        ):
            self._last_reject_reason = "invalid_input"
            return False
        ox = cx - ux
        oy = cy - uy
        if self._samples:
            last_ox, last_oy = self._samples[-1]
            if math.hypot(ox - last_ox, oy - last_oy) > cfg.max_position_jitter_m:
                self._last_reject_reason = "jitter"
                return False
        self._samples.append((ox, oy))
        if len(self._samples) > cfg.max_samples:
            self._samples.pop(0)
        self._last_reject_reason = None
        if len(self._samples) >= cfg.min_samples:
            recent = self._samples[-cfg.min_samples:]
            n = len(recent)
            mean_ox = sum(s[0] for s in recent) / n
            mean_oy = sum(s[1] for s in recent) / n
            var_ox = sum((s[0] - mean_ox) ** 2 for s in recent) / n
            var_oy = sum((s[1] - mean_oy) ** 2 for s in recent) / n
            if (
                math.sqrt(var_ox) <= cfg.target_convergence_m
                and math.sqrt(var_oy) <= cfg.target_convergence_m
            ):
                total_n = len(self._samples)
                self._offset = (
                    sum(s[0] for s in self._samples) / total_n,
                    sum(s[1] for s in self._samples) / total_n,
                )
        return True

    @property
    def ready(self) -> bool:
        return self._offset is not None

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def offset(self) -> tuple[float, float] | None:
        """返回标定的 offset (ox, oy)，未就绪返回 None。"""
        return self._offset

    def status(self) -> str:
        if self._offset is not None:
            return f"ready offset=({self._offset[0]:.3f}, {self._offset[1]:.3f})"
        if self._last_reject_reason is not None:
            return f"rejected: {self._last_reject_reason}"
        return f"collecting {len(self._samples)}/{self.config.min_samples}"


@dataclass(frozen=True)
class LaserContactConfig:
    contact_height_m: float = 0.10
    confirm_frames: int = 3
    invalid_clears: bool = True
    max_valid_height_m: float = 4.0
    min_valid_height_m: float = 0.02


class LaserContactDetector:
    """从激光高度计读数判定是否触地。"""

    def __init__(self, config: LaserContactConfig | None = None) -> None:
        self.config = config or LaserContactConfig()
        self._consecutive = 0

    def reset(self) -> None:
        self._consecutive = 0

    def update(self, laser_height_m: float | None) -> bool:
        """更新激光读数，返回当前是否判定为触地。"""
        cfg = self.config
        valid = (
            laser_height_m is not None
            and math.isfinite(laser_height_m)
            and cfg.min_valid_height_m <= float(laser_height_m) <= cfg.max_valid_height_m
        )
        if not valid:
            if cfg.invalid_clears:
                self._consecutive = 0
            return False
        if float(laser_height_m) <= cfg.contact_height_m:
            self._consecutive += 1
        else:
            self._consecutive = 0
        return self._consecutive >= cfg.confirm_frames

    @property
    def consecutive_count(self) -> int:
        return self._consecutive
