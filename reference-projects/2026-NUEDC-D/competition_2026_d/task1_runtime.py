"""任务一状态机与真实传感器/执行器之间的窄适配层。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .vision.platform_observation import FeatureFlag, PlatformObservation


@dataclass(frozen=True)
class VisionGateSample:
    seq: int | None
    found: bool
    quality: int
    ambiguous: bool
    error_xy_m: tuple[float, float] | None
    reason: str


def observation_to_gate_sample(
    observation: PlatformObservation | None,
    *,
    now: float,
    relative_height_m: float | None,
    max_age_s: float,
    min_quality: int,
    image_center_px: tuple[float, float],
    focal_px: tuple[float, float],
    target_offset_xy_m: tuple[float, float] = (0.0, 0.0),
) -> VisionGateSample:
    """把 VS1 转成门禁样本，不生成任何速度。

    坐标符号按 2026-07-31 USB 摄像头现场观察：画面下方为 +X，
    画面右侧为 +Y，因此相对误差为 ``(cy-cy0, cx-cx0)``。
    """
    if observation is None:
        return VisionGateSample(None, False, 0, False, None, "no_observation")
    flags = FeatureFlag(observation.flags)
    ambiguous = bool(flags & FeatureFlag.AMBIGUOUS)
    if observation.age_s(now) > max_age_s:
        return VisionGateSample(
            observation.seq, False, observation.quality, ambiguous, None, "stale"
        )
    if not observation.found:
        return VisionGateSample(
            observation.seq, False, observation.quality, ambiguous, None, "not_found"
        )
    if flags & FeatureFlag.TOO_CLOSE:
        return VisionGateSample(
            observation.seq, False, observation.quality, ambiguous, None, "too_close"
        )
    if ambiguous:
        return VisionGateSample(
            observation.seq, False, observation.quality, True, None, "ambiguous"
        )
    if observation.quality < min_quality:
        return VisionGateSample(
            observation.seq,
            False,
            observation.quality,
            False,
            None,
            "low_quality",
        )
    if relative_height_m is None or relative_height_m <= 0.0:
        return VisionGateSample(
            observation.seq,
            True,
            observation.quality,
            False,
            None,
            "height_unavailable",
        )
    fx, fy = focal_px
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("视觉焦距必须为正")
    cx0, cy0 = image_center_px
    measured_x = (observation.cy - cy0) * relative_height_m / fy
    measured_y = (observation.cx - cx0) * relative_height_m / fx
    target_x, target_y = target_offset_xy_m
    # The configured offset is the desired target position relative to the UAV.
    # At (-0.05, +0.08), for example, the detector reports zero control error
    # only when the observed target is 5 cm toward -X and 8 cm toward +Y.
    error_x = measured_x - target_x
    error_y = measured_y - target_y
    return VisionGateSample(
        observation.seq,
        True,
        observation.quality,
        False,
        (error_x, error_y),
        "ok",
    )


@dataclass(frozen=True)
class HeightReferenceConfig:
    world_kp: float = 1.0
    world_deadband_m: float = 0.03
    normal_slew_m_s: float = 0.25
    normal_ascent_slew_m_s: float | None = None
    drop_slew_m_s: float = 0.09
    min_laser_setpoint_m: float = 0.12
    max_laser_setpoint_m: float = 1.60


@dataclass(frozen=True)
class HeightReference:
    laser_setpoint_m: float
    valid: bool
    mode: str
    reason: str


class WorldDeckHeightController:
    """将 T265 世界高度或平台相对高度转换为飞控激光高度设定值。

    平台突然进入激光视野且 T265 世界高度已正确时，设定值立即跟随新的激光
    基准，避免把平台高度误当成飞行器下降并触发爬升。
    """

    def __init__(self, config: HeightReferenceConfig | None = None) -> None:
        self.config = config or HeightReferenceConfig()
        self._last_time: float | None = None
        self._setpoint_m: float | None = None

    def reset(
        self, *, timestamp: float | None = None, laser_height_m: float | None = None
    ) -> None:
        self._last_time = timestamp
        self._setpoint_m = laser_height_m

    def command(
        self,
        *,
        timestamp: float,
        current_world_height_m: float,
        current_laser_height_m: float | None,
        target_world_height_m: float,
        target_deck_height_m: float | None,
    ) -> HeightReference:
        cfg = self.config
        laser_valid = (
            current_laser_height_m is not None
            and math.isfinite(current_laser_height_m)
            and 0.02 <= current_laser_height_m <= 4.0
        )
        if not laser_valid:
            fallback = (
                cfg.min_laser_setpoint_m
                if self._setpoint_m is None
                else self._setpoint_m
            )
            return HeightReference(
                fallback, False, "hold", "laser_height_unavailable"
            )

        laser_height = float(current_laser_height_m)
        if self._setpoint_m is None:
            self._setpoint_m = laser_height
        if self._last_time is None:
            self._last_time = timestamp
            return HeightReference(
                self._setpoint_m, True, "initial_hold", "first_sample"
            )
        if target_deck_height_m is not None:
            candidate = float(target_deck_height_m)
            slew = cfg.drop_slew_m_s
            mode = "deck_relative"
        else:
            world_error = float(target_world_height_m) - float(
                current_world_height_m
            )
            candidate = laser_height + cfg.world_kp * world_error
            slew = (
                cfg.normal_ascent_slew_m_s
                if world_error > 0.0
                and cfg.normal_ascent_slew_m_s is not None
                else cfg.normal_slew_m_s
            )
            mode = "world_height"
            if abs(world_error) <= cfg.world_deadband_m:
                # 世界高度没有变化而激光基准突变，说明地面/平台表面发生切换。
                # 立即重基准，不能让旧激光设定值把飞机抬高。
                self._setpoint_m = candidate

        candidate = min(
            max(candidate, cfg.min_laser_setpoint_m),
            cfg.max_laser_setpoint_m,
        )
        if timestamp <= self._last_time:
            self._last_time = timestamp
            self._setpoint_m = candidate
        else:
            dt = min(timestamp - self._last_time, 0.20)
            max_delta = slew * dt
            delta = candidate - self._setpoint_m
            self._setpoint_m += max(-max_delta, min(max_delta, delta))
            self._last_time = timestamp
        return HeightReference(self._setpoint_m, True, mode, "ok")


@dataclass(frozen=True)
class T265SafetyConfig:
    max_position_step_m: float = 0.30
    max_step_interval_s: float = 0.30
    max_ground_height_offset_change_m: float = 0.25
    height_mismatch_confirm_s: float = 0.50
    height_mismatch_min_samples: int = 4
    max_hold_radius_m: float = 0.40
    height_offset_alpha: float = 0.08


class Task1T265SafetyMonitor:
    """检测置信度无法覆盖的T265跳变、漂移和高度失配。"""

    def __init__(self, config: T265SafetyConfig | None = None) -> None:
        self.config = config or T265SafetyConfig()
        self._last_time: float | None = None
        self._last_position: tuple[float, float, float] | None = None
        self._ground_height_offset_m: float | None = None
        self._height_mismatch_since: float | None = None
        self._height_mismatch_samples = 0
        self.fault_reason: str | None = None

    def update(
        self,
        *,
        timestamp: float,
        world_position_xyz_m: tuple[float, float, float],
        laser_height_m: float | None,
        ground_reference_expected: bool,
        hold_anchor_xy_m: tuple[float, float] | None,
    ) -> str | None:
        if self.fault_reason is not None:
            return self.fault_reason
        cfg = self.config
        position = tuple(float(v) for v in world_position_xyz_m)
        if self._last_time is not None and self._last_position is not None:
            dt = timestamp - self._last_time
            step = math.dist(position, self._last_position)
            if (
                0.0 < dt <= cfg.max_step_interval_s
                and step > cfg.max_position_step_m
            ):
                self.fault_reason = (
                    f"t265_position_jump:{step:.3f}m/{dt:.3f}s"
                )

        if (
            self.fault_reason is None
            and hold_anchor_xy_m is not None
            and math.hypot(
                position[0] - hold_anchor_xy_m[0],
                position[1] - hold_anchor_xy_m[1],
            )
            > cfg.max_hold_radius_m
        ):
            self.fault_reason = "t265_hold_geofence_exceeded"

        laser_valid = (
            laser_height_m is not None
            and math.isfinite(laser_height_m)
            and 0.05 < laser_height_m <= 4.0
        )
        if (
            self.fault_reason is None
            and ground_reference_expected
            and laser_valid
        ):
            offset = position[2] - float(laser_height_m)
            if self._ground_height_offset_m is None:
                self._ground_height_offset_m = offset
            elif (
                abs(offset - self._ground_height_offset_m)
                > cfg.max_ground_height_offset_change_m
            ):
                if self._height_mismatch_since is None:
                    self._height_mismatch_since = timestamp
                    self._height_mismatch_samples = 1
                else:
                    self._height_mismatch_samples += 1
                if (
                    timestamp - self._height_mismatch_since
                    >= cfg.height_mismatch_confirm_s
                    and self._height_mismatch_samples
                    >= cfg.height_mismatch_min_samples
                ):
                    self.fault_reason = (
                        "t265_laser_height_mismatch:"
                        f"offset={offset:+.3f}m,"
                        f"baseline={self._ground_height_offset_m:+.3f}m,"
                        f"samples={self._height_mismatch_samples}"
                    )
            else:
                self._height_mismatch_since = None
                self._height_mismatch_samples = 0
                alpha = cfg.height_offset_alpha
                self._ground_height_offset_m = (
                    alpha * offset
                    + (1.0 - alpha) * self._ground_height_offset_m
                )
        elif not ground_reference_expected or not laser_valid:
            self._height_mismatch_since = None
            self._height_mismatch_samples = 0
            if not ground_reference_expected:
                # The laser may cross a platform edge or a different-height
                # surface while this check is intentionally inactive.  Its
                # old offset is therefore not a valid baseline when a later
                # ground-reference phase starts (for example LAND_H).
                self._ground_height_offset_m = None

        self._last_time = timestamp
        self._last_position = position
        return self.fault_reason
