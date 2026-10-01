"""25cm静态蓝方块的低速XY视觉伺服provider。"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass

from .control.formation_controller import FormationConfig, FormationController
from .vision.cybercam_reader import CyberCamReader
from .vision.platform_observation import FeatureFlag, PlatformObservation


@dataclass(frozen=True)
class StaticSquareServoConfig:
    image_cx_px: float = 320.0
    image_cy_px: float = 240.0
    focal_x_px: float = 500.0
    focal_y_px: float = 500.0
    min_quality_full: int = 55
    min_quality_partial: int = 40
    max_observation_age_s: float = 0.15
    confirm_frames: int = 3
    full_max_speed_m_s: float = 0.15
    partial_max_speed_m_s: float = 0.06
    max_accel_m_s2: float = 0.15
    max_jerk_m_s3: float = 1.0
    kp: float = 0.35
    kd: float = 0.50
    platform_velocity_m_s: tuple[float, float] = (0.0, 0.0)
    target_velocity_feedforward_gain: float = 0.0
    deadband_m: float = 0.05
    velocity_deadband_m_s: float = 0.02
    centered_hold_s: float = 3.0
    max_duration_s: float = 15.0
    soft_radius_m: float = 0.50
    hard_radius_m: float = 0.60
    min_height_m: float = 1.40
    max_height_m: float = 1.60
    low_confidence_grace_s: float = 0.15
    t265_jump_window_s: float = 0.20
    t265_jump_m: float = 0.30
    max_measurement_innovation_m: float = 0.20
    vision_target_source: str = "apriltag"


@dataclass(frozen=True)
class ServoSnapshot:
    mode: str
    reason: str
    active: bool
    finished: bool
    faulted: bool
    observation_seq: int | None
    observation_age_s: float | None
    raw_center_px: tuple[int, int] | None
    filtered_target_xy_m: tuple[float, float] | None
    predicted_target_xy_m: tuple[float, float] | None
    command_m_s: tuple[float, float]
    command_cm_s: tuple[int, int]
    radius_m: float
    target_source: str


class StaticSquareServo:
    """Callable provider；不直接访问飞控发送数组，也不执行任何阻塞I/O。"""

    def __init__(
        self, reader: CyberCamReader, config: StaticSquareServoConfig | None = None
    ) -> None:
        self.reader = reader
        self.config = config or StaticSquareServoConfig()
        cfg = self.config
        if cfg.vision_target_source not in ("apriltag", "blue_square"):
            raise ValueError(
                f"unsupported vision target source: {cfg.vision_target_source}"
            )
        # 静态专项视觉只做“目标中心→画面中心”的相对误差闭环。
        # 不估计目标世界速度，也不使用视觉速度前馈；移动小车的速度前馈
        # 后续由双T265提供，避免下降/姿态变化被误判为目标运动。
        self.controller = FormationController(FormationConfig(
            kp=0.0,
            kd=0.0,
            max_speed_m_s=cfg.full_max_speed_m_s,
            max_accel_m_s2=cfg.max_accel_m_s2,
            max_jerk_m_s3=cfg.max_jerk_m_s3,
            max_estimate_age_s=cfg.max_observation_age_s,
            max_uncertainty_m=0.30,
            position_deadband_m=cfg.deadband_m,
            target_velocity_feedforward_gain=0.0,
        ))
        self._armed = False
        self._finished = False
        self._faulted = False
        self._exiting = False
        self._completion_enabled = True
        self._reason = "not_armed"
        self._mode = "LOST"
        self._start_time: float | None = None
        self._anchor_xy = (0.0, 0.0)
        self._last_pose: tuple[float, float, float] | None = None
        self._low_confidence_since: float | None = None
        self._height_samples = deque(maxlen=5)
        self._full_measurements = deque(maxlen=max(1, cfg.confirm_frames))
        self._last_key: tuple[int, int] | None = None
        self._last_full_time: float | None = None
        self._last_full_temporal = False
        self._last_full_color = False
        self._last_relative_error_m: tuple[float, float] | None = None
        self._last_partial_time: float | None = None
        self._partial_direction = (0.0, 0.0)
        self._centered_since: float | None = None
        self._last_command_m_s = (0.0, 0.0)
        self._last_snapshot = ServoSnapshot(
            "LOST", "not_armed", False, False, False, None, None, None,
            None, None, (0.0, 0.0), (0, 0), 0.0, cfg.vision_target_source,
        )

    @property
    def active(self) -> bool:
        return self._armed and not self._finished

    @property
    def finished(self) -> bool:
        return self._finished

    @property
    def faulted(self) -> bool:
        return self._faulted

    @property
    def reason(self) -> str:
        return self._reason

    def snapshot(self) -> ServoSnapshot:
        return self._last_snapshot

    @property
    def completion_enabled(self) -> bool:
        return self._completion_enabled

    def set_completion_enabled(self, enabled: bool) -> None:
        """允许任务在下降期间持续追踪，到达目标高度后再允许居中退出。"""
        self._completion_enabled = bool(enabled)

    def observation_usable_for_preflight(self, observation, now: float) -> bool:
        if observation is None:
            return False
        flags = FeatureFlag(observation.flags)
        if flags & FeatureFlag.PARTIAL:
            return False
        return observation.usable(
            now,
            self.config.max_observation_age_s,
            self.config.min_quality_full,
            target_source=self.config.vision_target_source,
        )

    def arm(self, now: float, position_xy: tuple[float, float]) -> None:
        if self._faulted:
            raise RuntimeError("视觉故障已锁存，本次任务禁止重新接管")
        self._armed = True
        self._finished = False
        self._exiting = False
        self._reason = "armed"
        self._mode = "LOST"
        self._start_time = float(now)
        self._anchor_xy = (float(position_xy[0]), float(position_xy[1]))
        self._last_pose = None
        self._low_confidence_since = None
        self._height_samples.clear()
        self._full_measurements.clear()
        self._last_key = None
        self._last_full_time = None
        self._last_full_temporal = False
        self._last_full_color = False
        self._last_relative_error_m = None
        self._last_partial_time = None
        self._partial_direction = (0.0, 0.0)
        self._centered_since = None
        self._last_command_m_s = (0.0, 0.0)
        self.controller.reset(now, (0.0, 0.0))

    def abort_before_arm(self, reason: str) -> None:
        """入口预检/稳定等待失败；此时视觉从未输出速度，可直接锁存退出。"""
        if self._armed:
            raise RuntimeError("已接管后必须通过provider故障路径退出")
        self._faulted = True
        self._finished = True
        self._reason = str(reason)
        self._mode = "FAULT"
        self._last_snapshot = ServoSnapshot(
            "FAULT", self._reason, False, True, True, None, None, None,
            None, None, (0.0, 0.0), (0, 0), 0.0,
            self.config.vision_target_source,
        )

    def __call__(self, context: dict) -> dict | None:
        if not self._armed or self._finished:
            return None
        now = float(context["now_monotonic"])
        position = tuple(float(v) for v in context["position_m"])
        velocity = tuple(float(v) for v in context["velocity_m_s"])
        confidence = int(context["t265_confidence"])

        fault = self._check_flight_guards(now, position, velocity, confidence)
        if fault is not None:
            return self._fault_decision(fault, position)

        radius = math.hypot(position[0] - self._anchor_xy[0], position[1] - self._anchor_xy[1])
        if self.config.hard_radius_m > 0.0 and radius >= self.config.hard_radius_m:
            return self._fault_decision("hard_geofence", position)
        if now - self._start_time >= self.config.max_duration_s:
            self._begin_exit("hard_timeout")

        observation = self.reader.latest(now, self.config.max_observation_age_s)
        if not self.reader.is_running():
            return self._fault_decision("cybercam_reader_stopped", position)
        # 旧相对误差超过视觉新鲜度后立即清除，重新捕获从新观测开始。
        if (
            self._last_full_time is not None
            and now - self._last_full_time > self.config.max_observation_age_s
        ):
            self._last_full_time = None
            self._last_full_temporal = False
            self._last_full_color = False
            self._last_relative_error_m = None
            self._full_measurements.clear()
        if observation is not None:
            self._consume_new_observation(now, observation, position, velocity)

        if self._exiting:
            shaped = self.controller.shape_velocity(0.0, 0.0, now)
            command = (shaped.vx_m_s, shaped.vy_m_s)
            if math.hypot(*command) <= 0.005:
                command = (0.0, 0.0)
                self._finished = True
                self._armed = False
            return self._decision(now, observation, position, command, "EXITING")

        command, mode = self._tracking_command(now, observation, position, velocity)
        command = self._apply_soft_geofence(command, position)
        self._last_command_m_s = command
        return self._decision(now, observation, position, command, mode)

    def _check_flight_guards(self, now, position, velocity, confidence) -> str | None:
        values = (*position, *velocity, now)
        if not all(math.isfinite(value) for value in values):
            return "t265_nonfinite"
        if confidence == 0:
            return "t265_confidence_zero"
        if confidence < 2:
            if self._low_confidence_since is None:
                self._low_confidence_since = now
            elif now - self._low_confidence_since > self.config.low_confidence_grace_s:
                return "t265_low_confidence"
        else:
            self._low_confidence_since = None
        if self._last_pose is not None:
            last_t, last_x, last_y = self._last_pose
            dt = now - last_t
            if 0.0 < dt <= self.config.t265_jump_window_s:
                if math.hypot(position[0] - last_x, position[1] - last_y) > self.config.t265_jump_m:
                    return "t265_relocalization_jump"
        self._last_pose = (now, position[0], position[1])

        height = position[2]
        if 0.05 < height <= 10.0:
            self._height_samples.append(height)
        if not self._height_samples:
            return "height_unavailable"
        height_median = statistics.median(self._height_samples)
        if not self.config.min_height_m <= height_median <= self.config.max_height_m:
            return "height_out_of_range"
        return None

    def _consume_new_observation(self, now, observation, position, velocity) -> None:
        key = (observation.stream_id, observation.seq)
        if key == self._last_key:
            return
        self._last_key = key
        flags = FeatureFlag(observation.flags)
        if not observation.found or flags & (FeatureFlag.AMBIGUOUS | FeatureFlag.TOO_CLOSE):
            self._full_measurements.clear()
            return
        surrogate = bool(flags & FeatureFlag.SURROGATE_SQUARE)
        apriltag = bool(flags & FeatureFlag.APRILTAG_VALID)
        if surrogate and apriltag:
            self._full_measurements.clear()
            return
        if self.config.vision_target_source == "apriltag":
            source_matches = apriltag and not bool(flags & FeatureFlag.PARTIAL)
        else:
            source_matches = surrogate
        if not source_matches:
            self._full_measurements.clear()
            return
        error_x_px = self.config.image_cx_px - observation.cx
        # 2026-07-30实飞标定：目标在画面上方时输出+Y会继续把目标推向
        # 上边缘，因此当前飞行链路必须输出-Y；X方向保持不变。
        error_y_px = observation.cy - self.config.image_cy_px
        if flags & FeatureFlag.PARTIAL:
            self._full_measurements.clear()
            if observation.quality < self.config.min_quality_partial:
                return
            norm = math.hypot(error_x_px, error_y_px)
            self._partial_direction = (
                (error_x_px / norm, error_y_px / norm) if norm > 1e-6 else (0.0, 0.0)
            )
            self._last_partial_time = observation.received_monotonic
            return
        if observation.quality < self.config.min_quality_full:
            self._full_measurements.clear()
            return

        height = statistics.median(self._height_samples)
        relative_x = error_x_px * height / self.config.focal_x_px
        relative_y = error_y_px * height / self.config.focal_y_px
        self._full_measurements.append((relative_x, relative_y, observation.quality))
        if len(self._full_measurements) < self.config.confirm_frames:
            return
        median_x = statistics.median(item[0] for item in self._full_measurements)
        median_y = statistics.median(item[1] for item in self._full_measurements)
        self._last_relative_error_m = (median_x, median_y)
        self._last_full_time = observation.received_monotonic
        self._last_full_temporal = bool(flags & FeatureFlag.TEMPORAL_TRACKED)
        self._last_full_color = bool(flags & FeatureFlag.COLOR_SHAPE_TRACKED)

    def _tracking_command(self, now, observation, position, velocity):
        if self._last_partial_time is not None and (
            self._last_full_time is None or self._last_partial_time > self._last_full_time
        ) and now - self._last_partial_time <= self.config.max_observation_age_s:
            raw = (
                self._partial_direction[0] * self.config.partial_max_speed_m_s,
                self._partial_direction[1] * self.config.partial_max_speed_m_s,
            )
            shaped = self.controller.shape_velocity(*raw, now)
            self._centered_since = None
            return (shaped.vx_m_s, shaped.vy_m_s), "PARTIAL_COARSE"

        fresh_full = (
            self._last_full_time is not None
            and self._last_relative_error_m is not None
            and now - self._last_full_time <= self.config.max_observation_age_s
        )
        if fresh_full:
            error_x, error_y = self._last_relative_error_m
            error = math.hypot(error_x, error_y)
            platform_vx, platform_vy = self.config.platform_velocity_m_s
            relative_vx = velocity[0] - platform_vx
            relative_vy = velocity[1] - platform_vy
            relative_speed = math.hypot(relative_vx, relative_vy)
            position_centered = error <= self.config.deadband_m
            velocity_stable = relative_speed <= self.config.velocity_deadband_m_s
            if position_centered and velocity_stable:
                if self._centered_since is None:
                    self._centered_since = now
                elif (
                    self._completion_enabled
                    and now - self._centered_since >= self.config.centered_hold_s
                ):
                    self._begin_exit("centered_hold_complete")
                mode = "CENTERED"
            else:
                self._centered_since = None
                mode = "FULL_TRACK"
            p_vx = 0.0 if position_centered else self.config.kp * error_x
            p_vy = 0.0 if position_centered else self.config.kp * error_y
            # 视觉只提供相对位置反馈；D项直接使用T265实际速度，抑制飞控
            # 约1.2～1.4秒的水平响应滞后。进入位置死区后仍继续制动，
            # 直到实际速度也进入死区，避免零命令时靠惯性滑出中心区域。
            d_vx = 0.0 if velocity_stable else self.config.kd * relative_vx
            d_vy = 0.0 if velocity_stable else self.config.kd * relative_vy
            target_vx = platform_vx + p_vx - d_vx
            target_vy = platform_vy + p_vy - d_vy
            shaped = self.controller.shape_velocity(target_vx, target_vy, now)
            vx, vy = shaped.vx_m_s, shaped.vy_m_s
            temporal = self._last_full_temporal
            color = self._last_full_color
            if color:
                mode = "COLOR_TRACK"
            elif temporal:
                mode = "TEMPORAL_TRACK"
            return (vx, vy), mode

        self._centered_since = None
        # 超过配置的新鲜度门限后安全性优先于平滑性：不能让限加速度器继续发送
        # 由旧观测产生的残余速度。重置整形器并在本周期严格归零。
        self.controller.reset(now, (0.0, 0.0))
        return (0.0, 0.0), "LOST"

    def _apply_soft_geofence(self, command, position):
        if self.config.soft_radius_m <= 0.0:
            return command
        dx = position[0] - self._anchor_xy[0]
        dy = position[1] - self._anchor_xy[1]
        radius = math.hypot(dx, dy)
        if radius < self.config.soft_radius_m or radius <= 1e-9:
            return command
        radial_out = (command[0] * dx + command[1] * dy) / radius
        if radial_out <= 0.0:
            return command
        return (
            command[0] - radial_out * dx / radius,
            command[1] - radial_out * dy / radius,
        )

    def _begin_exit(self, reason: str) -> None:
        if not self._exiting:
            self._exiting = True
            self._reason = reason
            self._mode = "EXITING"

    def _fault_decision(self, reason: str, position) -> dict:
        self._faulted = True
        self._finished = True
        self._armed = False
        self._reason = reason
        self._mode = "FAULT"
        radius = math.hypot(position[0] - self._anchor_xy[0], position[1] - self._anchor_xy[1])
        self._last_snapshot = ServoSnapshot(
            "FAULT", reason, False, True, True, None, None, None, None, None,
            (0.0, 0.0), (0, 0), radius, self.config.vision_target_source,
        )
        return {
            "active": True, "fault": True, "vx_cms": 0, "vy_cms": 0,
            "source": "vision_fault_zero", "reason": reason,
        }

    def _decision(self, now, observation, position, command, mode) -> dict:
        vx_cms = int(round(command[0] * 100.0))
        vy_cms = int(round(command[1] * 100.0))
        radius = math.hypot(position[0] - self._anchor_xy[0], position[1] - self._anchor_xy[1])
        measured_target = None
        if self._last_relative_error_m is not None:
            measured_target = (
                position[0] + self._last_relative_error_m[0],
                position[1] + self._last_relative_error_m[1],
            )
        self._mode = mode
        self._last_snapshot = ServoSnapshot(
            mode,
            self._reason,
            not self._finished,
            self._finished,
            self._faulted,
            observation.seq if observation is not None else None,
            observation.age_s(now) if observation is not None else None,
            (observation.cx, observation.cy) if observation is not None else None,
            measured_target,
            None,
            (float(command[0]), float(command[1])),
            (vx_cms, vy_cms),
            radius,
            self.config.vision_target_source,
        )
        return {
            # LOST阶段交回基础T265位置闭环，避免持续发送零速度造成惯性漂移。
            "active": mode != "LOST",
            "fault": False,
            "vx_cms": vx_cms,
            "vy_cms": vy_cms,
            "source": f"vision_xy_{self.config.vision_target_source}",
            "reason": mode.lower(),
        }
