"""D题任务一的纯任务状态机。

本模块不直接访问串口、GPIO或飞控。飞行适配层每个控制周期提供 T265、激光、
视觉和舵机状态，再执行返回的唯一水平速度、高度目标与一次性投放请求。
正式任务在 B-C 固定路径速度上叠加小幅视觉微调；联合测试可显式启用纯视觉接管。
"""

from __future__ import annotations

import enum
import math
from collections import deque
from dataclasses import dataclass
from statistics import median

from .control.task1_path_controller import (
    PathCommand,
    PolylinePath,
    Task1PathFollower,
    Task1PathFollowerConfig,
)
from .payload_actuator import ActuatorState


H = (0.0, 0.0)
# 相比已实飞的1.75m位置，沿小车行进方向向B靠近15cm；
# 根据2026-07-31联合测试画面，再沿当前坐标系-X方向左移10cm。
# 仍在B点前保留35cm视觉确认窗口，同时为无人机起飞与截获增加时间余量。
B_PRE = (0.275, 1.90)
B = (0.375, 2.25)
BC_1 = (0.595, 2.78)
BC_2 = (1.125, 3.00)
BC_3 = (1.655, 2.78)
C = (1.875, 2.25)
D = (1.875, 0.75)


class Task1Phase(enum.Enum):
    WAIT_START = "wait_start"
    TAKEOFF = "takeoff"
    HOLD_3S = "hold_3s"
    INTERCEPT_B_PRE = "intercept_b_pre"
    ACQUIRE_TARGET = "acquire_target"
    FOLLOW_B_C = "follow_b_c"
    SYNC_TARGET_AT_C = "sync_target_at_c"
    DROP_WINDOW_C_D = "drop_window_c_d"
    DROP_DESCENT = "drop_descent"
    RELEASING = "releasing"
    CLIMB = "climb"
    RETURN_H = "return_h"
    LAND_H = "land_h"
    COMPLETE = "complete"


@dataclass(frozen=True)
class Task1Config:
    cruise_height_m: float = 1.50
    follow_height_m: float = 1.00
    drop_height_m: float = 0.65
    final_height_m: float = 0.15
    hold_duration_s: float = 2.0
    takeoff_ascent_slew_m_s: float = 0.40
    intercept_speed_m_s: float = 0.20
    car_speed_m_s: float = 0.13
    curve_speed_m_s: float | None = None
    car_speed_scale: float = 1.0
    return_speed_m_s: float = 0.35
    point_kp: float = 0.90
    point_arrival_radius_m: float = 0.14
    max_point_speed_m_s: float = 0.40
    hold_position_max_speed_m_s: float = 0.15
    hold_stable_speed_m_s: float = 0.12
    final_landing_radius_m: float = 0.08
    final_landing_max_speed_m_s: float = 0.05
    final_landing_stable_s: float = 0.50
    final_descend_horizontal_max_speed_m_s: float = 0.06
    path_lookahead_m: float = 0.20
    path_cross_track_kp: float = 0.80
    path_max_correction_m_s: float = 0.08
    path_max_speed_m_s: float = 0.20
    path_max_accel_m_s2: float = 0.30
    # 0 表示在 B_PRE 持续等待视觉确认；正赛默认不能因超时越过小车。
    acquire_timeout_s: float = 0.0
    vision_min_quality: int = 55
    vision_confirm_frames: int = 2
    intercept_vision_confirm_frames: int = 5
    intercept_vision_max_error_m: float = 0.35
    acquire_max_error_m: float = 0.30
    drop_max_error_m: float = 0.15
    drop_confirm_duration_s: float = 0.40
    min_follow_before_drop_s: float = 0.0
    vision_trim_kp: float = 0.35
    vision_trim_deadband_m: float = 0.05
    vision_trim_max_speed_m_s: float = 0.03
    vision_trim_max_accel_m_s2: float = 0.08
    acquire_vision_min_quality: int = 40
    acquire_vision_kp: float = 0.45
    acquire_vision_deadband_m: float = 0.04
    acquire_vision_max_speed_m_s: float = 0.12
    acquire_vision_max_accel_m_s2: float = 0.25
    acquire_vision_control_period_s: float = 0.20
    acquire_vision_filter_window_s: float = 0.60
    acquire_vision_loss_grace_s: float = 0.30
    vision_takeover_max_speed_m_s: float = 0.08
    intercept_vision_early_stop_enabled: bool = True
    drop_rel_height_tolerance_m: float = 0.05
    drop_descent_speed_m_s: float = 0.09
    drop_time_margin_s: float = 0.80
    release_timeout_s: float = 1.0
    t265_min_confidence: int = 2
    path_only_b_pre_descent: bool = False
    vision_track_only: bool = False
    c_sync_vision_enabled: bool = True
    payload_drop_enabled: bool = True
    drop_during_bc_enabled: bool = True
    drop_at_follow_height: bool = True


@dataclass(frozen=True)
class Task1Input:
    now: float
    position_xyz_m: tuple[float, float, float]
    velocity_xy_m_s: tuple[float, float] = (0.0, 0.0)
    t265_confidence: int = 3
    car_start: bool = False
    car_speed_m_s: float | None = None
    vision_seq: int | None = None
    vision_found: bool = False
    vision_quality: int = 0
    vision_ambiguous: bool = False
    vision_error_xy_m: tuple[float, float] | None = None
    deck_relative_height_m: float | None = None
    payload_state: ActuatorState = ActuatorState.LOCKED
    landed: bool = False


@dataclass(frozen=True)
class Task1Command:
    phase: Task1Phase
    target_xy_m: tuple[float, float]
    target_world_height_m: float
    target_deck_height_m: float | None
    vx_m_s: float
    vy_m_s: float
    base_vx_m_s: float
    base_vy_m_s: float
    vision_trim_vx_m_s: float
    vision_trim_vy_m_s: float
    takeoff_requested: bool
    release_requested: bool
    land_requested: bool
    fixed_heading: bool
    target_acquired: bool
    drop_committed: bool
    drop_released: bool
    mission_success: bool
    reason: str


class Task1MissionDirector:
    """任务一唯一决策源；默认使用路径微调，也支持显式纯视觉测试。"""

    def __init__(self, config: Task1Config | None = None) -> None:
        self.config = config or Task1Config()
        path_cfg = Task1PathFollowerConfig(
            lookahead_m=self.config.path_lookahead_m,
            cross_track_kp=self.config.path_cross_track_kp,
            max_correction_m_s=self.config.path_max_correction_m_s,
            max_speed_m_s=self.config.path_max_speed_m_s,
            max_accel_m_s2=self.config.path_max_accel_m_s2,
            completion_radius_m=self.config.point_arrival_radius_m,
        )
        self.bc_follower = Task1PathFollower(
            PolylinePath((B_PRE, B, BC_1, BC_2, BC_3, C)), path_cfg
        )
        self.cd_follower = Task1PathFollower(PolylinePath((C, D)), path_cfg)
        self.phase = Task1Phase.WAIT_START
        self._phase_since = 0.0
        self._stable_since: float | None = None
        self._last_vision_seq: int | None = None
        self._vision_confirm_count = 0
        self._vision_confirm_since: float | None = None
        self._vision_trim_xy_m_s = (0.0, 0.0)
        self._vision_trim_time: float | None = None
        self._acquire_vision_xy_m_s = (0.0, 0.0)
        self._acquire_vision_control_time: float | None = None
        self._acquire_vision_last_valid_time: float | None = None
        self._acquire_vision_last_seq: int | None = None
        self._acquire_vision_errors: deque[
            tuple[float, int, float, float]
        ] = deque(maxlen=32)
        self._acquire_anchor = B_PRE
        self._vision_takeover_ready = False
        self._release_motion = "c_hold"
        self.target_acquired = False
        self.drop_committed = False
        self.drop_released = False
        self.mission_success = False
        self.reason = "waiting_for_car_start"
        self._climb_anchor = H

    def tick(self, data: Task1Input) -> Task1Command:
        cfg = self.config
        x, y, z_world = data.position_xyz_m
        position_xy = (x, y)
        raw_car_speed = (
            cfg.car_speed_m_s
            if data.car_speed_m_s is None
            else max(0.0, float(data.car_speed_m_s))
        )
        car_speed = raw_car_speed * max(0.0, cfg.car_speed_scale)
        curve_speed = (
            car_speed
            if cfg.curve_speed_m_s is None
            else max(0.0, cfg.curve_speed_m_s)
            * max(0.0, cfg.car_speed_scale)
        )
        target_xy = position_xy
        target_world_height = cfg.cruise_height_m
        target_deck_height = None
        vx = vy = 0.0
        base_vx = base_vy = 0.0
        trim_vx = trim_vy = 0.0
        takeoff_requested = False
        release_requested = False
        land_requested = False

        if self.phase == Task1Phase.WAIT_START:
            target_xy = H
            if data.car_start and data.t265_confidence >= cfg.t265_min_confidence:
                self._transition(Task1Phase.TAKEOFF, data.now, "car_start_accepted")

        elif self.phase == Task1Phase.TAKEOFF:
            target_xy = H
            vx, vy = self._point_velocity(
                position_xy, H, cfg.hold_position_max_speed_m_s
            )
            takeoff_requested = True
            if z_world >= cfg.cruise_height_m - 0.10:
                self._transition(Task1Phase.HOLD_3S, data.now, "takeoff_height_reached")
                self._stable_since = None

        elif self.phase == Task1Phase.HOLD_3S:
            target_xy = H
            vx, vy = self._point_velocity(
                position_xy, H, cfg.hold_position_max_speed_m_s
            )
            hold_elapsed = data.now - self._phase_since
            height_safe = abs(z_world - cfg.cruise_height_m) <= 0.15
            if hold_elapsed >= cfg.hold_duration_s and height_safe:
                self._transition(
                    Task1Phase.INTERCEPT_B_PRE, data.now, "takeoff_hover_complete"
                )

        elif self.phase == Task1Phase.INTERCEPT_B_PRE:
            target_xy = B_PRE
            vx, vy = self._point_velocity(
                position_xy, B_PRE, cfg.intercept_speed_m_s
            )
            if cfg.intercept_vision_early_stop_enabled and self._confirm_vision(
                data,
                require_centered=True,
                max_error_m=cfg.intercept_vision_max_error_m,
                confirm_frames=cfg.intercept_vision_confirm_frames,
            ):
                # 只有目标进入中央安全区并连续稳定后才提前停止路径；
                # 边缘单帧检测继续飞向B_PRE，避免过早交给纯视觉后立即丢失。
                self._acquire_anchor = position_xy
                target_xy = position_xy
                vx = vy = 0.0
                self._transition(
                    Task1Phase.ACQUIRE_TARGET,
                    data.now,
                    "target_centered_during_intercept",
                )
            elif self._near(position_xy, B_PRE) and math.hypot(
                *data.velocity_xy_m_s
            ) <= cfg.hold_stable_speed_m_s:
                self._acquire_anchor = position_xy
                self._transition(
                    Task1Phase.ACQUIRE_TARGET, data.now, "b_pre_reached"
                )

        elif self.phase == Task1Phase.ACQUIRE_TARGET:
            # 本阶段只由视觉误差生成水平速度；丢失目标时平滑减速至0，
            # 不再由固定B_PRE坐标与视觉争夺控制权。
            target_xy = self._acquire_anchor
            takeover_braking = (
                cfg.vision_track_only
                and not self._vision_takeover_ready
                and math.hypot(*data.velocity_xy_m_s)
                > cfg.vision_takeover_max_speed_m_s
            )
            if takeover_braking:
                target_world_height = cfg.follow_height_m
                vx = vy = 0.0
                self.reason = "pure_vision_takeover_braking"
                self._vision_confirm_count = 0
                self._last_vision_seq = None
                self._vision_confirm_since = None
            else:
                if cfg.vision_track_only and not self._vision_takeover_ready:
                    self._vision_takeover_ready = True
                    self.reason = "pure_vision_takeover_ready"
                if (
                    not cfg.vision_track_only
                    and not cfg.intercept_vision_early_stop_enabled
                ):
                    # 固定路径联合模式在B_PRE只做T265定点等待；视觉只读，
                    # 不在开始伴飞前把飞机从路径起点拉走。
                    vx, vy = self._point_velocity(
                        position_xy,
                        self._acquire_anchor,
                        cfg.hold_position_max_speed_m_s,
                    )
                    base_vx, base_vy = vx, vy
                else:
                    trim_vx, trim_vy = self._acquire_vision_command(data)
                    vx, vy = trim_vx, trim_vy
            if cfg.vision_track_only:
                target_world_height = cfg.follow_height_m
                if takeover_braking:
                    pass
                elif cfg.payload_drop_enabled:
                    at_follow_height = (
                        abs(z_world - cfg.follow_height_m) <= 0.10
                    )
                    if not at_follow_height:
                        self._vision_confirm_count = 0
                        self._last_vision_seq = None
                        self._vision_confirm_since = None
                    elif self._confirm_vision(
                        data,
                        require_centered=True,
                        max_error_m=cfg.drop_max_error_m,
                        min_duration_s=cfg.drop_confirm_duration_s,
                    ):
                        self.target_acquired = True
                        self.drop_committed = True
                        release_requested = True
                        self._release_motion = "vision_hold"
                        self._transition(
                            Task1Phase.RELEASING,
                            data.now,
                            "drop_gate_latched_on_pure_vision",
                        )
                elif self._confirm_vision(
                    data,
                    require_centered=True,
                    max_error_m=cfg.acquire_max_error_m,
                ):
                    self.target_acquired = True
                    self.reason = "vision_track_only_centered"
            elif cfg.path_only_b_pre_descent:
                target_world_height = cfg.follow_height_m
                if abs(z_world - cfg.follow_height_m) <= 0.10:
                    self.target_acquired = True
                    self.bc_follower.reset(
                        timestamp=data.now,
                        velocity_xy_m_s=data.velocity_xy_m_s,
                    )
                    self._transition(
                        Task1Phase.FOLLOW_B_C,
                        data.now,
                        "path_test_b_pre_descent_complete",
                    )
            elif self._confirm_vision(
                data,
                require_centered=True,
                max_error_m=cfg.acquire_max_error_m,
            ):
                self.target_acquired = True
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task1Phase.FOLLOW_B_C, data.now, "target_acquired_at_b_pre"
                )
            elif (
                cfg.acquire_timeout_s > 0.0
                and data.now - self._phase_since >= cfg.acquire_timeout_s
            ):
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task1Phase.FOLLOW_B_C,
                    data.now,
                    "b_pre_timeout_path_fallback",
                )

        elif self.phase == Task1Phase.FOLLOW_B_C:
            if not self.target_acquired and self._confirm_vision(
                data, require_centered=False
            ):
                self.target_acquired = True
                self.reason = "target_acquired_on_b_c"
            target_world_height = (
                cfg.follow_height_m if self.target_acquired else cfg.cruise_height_m
            )
            path = self.bc_follower.command(
                position_xy,
                nominal_speed_m_s=curve_speed,
                timestamp=data.now,
            )
            target_xy, base_vx, base_vy = self._from_path(path)
            trim_vx, trim_vy = self._vision_trim_command(data)
            vx, vy = _limit_norm(
                base_vx + trim_vx,
                base_vy + trim_vy,
                min(
                    cfg.path_max_speed_m_s,
                    curve_speed + cfg.vision_trim_max_speed_m_s,
                ),
            )
            at_follow_height = (
                abs(z_world - cfg.follow_height_m) <= 0.10
            )
            if (
                cfg.payload_drop_enabled
                and cfg.drop_during_bc_enabled
                and cfg.drop_at_follow_height
                and self.target_acquired
                and at_follow_height
                and data.now - self._phase_since
                >= cfg.min_follow_before_drop_s
                and self._confirm_vision(
                    data,
                    require_centered=True,
                    max_error_m=cfg.drop_max_error_m,
                    min_duration_s=cfg.drop_confirm_duration_s,
                )
            ):
                self.drop_committed = True
                release_requested = True
                self._release_motion = "bc"
                self._transition(
                    Task1Phase.RELEASING,
                    data.now,
                    "drop_gate_latched_on_b_c_at_follow_height",
                )
            elif path.completed:
                if cfg.c_sync_vision_enabled:
                    self._transition(
                        Task1Phase.SYNC_TARGET_AT_C,
                        data.now,
                        "c_reached_waiting_for_centered_target",
                    )
                else:
                    self.cd_follower.reset(
                        timestamp=data.now,
                        velocity_xy_m_s=data.velocity_xy_m_s,
                    )
                    self._transition(
                        Task1Phase.DROP_WINDOW_C_D, data.now, "c_reached"
                    )

        elif self.phase == Task1Phase.SYNC_TARGET_AT_C:
            target_xy = C
            target_world_height = cfg.follow_height_m
            vx, vy = self._point_velocity(
                position_xy, C, cfg.hold_position_max_speed_m_s
            )
            base_vx, base_vy = vx, vy
            trim_vx, trim_vy = self._vision_trim_command(data)
            vx, vy = _limit_norm(
                base_vx + trim_vx,
                base_vy + trim_vy,
                min(
                    cfg.path_max_speed_m_s,
                    cfg.hold_position_max_speed_m_s
                    + cfg.vision_trim_max_speed_m_s,
                ),
            )
            if self._confirm_vision(
                data,
                require_centered=True,
                max_error_m=cfg.drop_max_error_m,
                min_duration_s=cfg.drop_confirm_duration_s,
            ):
                if cfg.payload_drop_enabled and cfg.drop_at_follow_height:
                    self.drop_committed = True
                    release_requested = True
                    self._release_motion = "c_hold"
                    self._transition(
                        Task1Phase.RELEASING,
                        data.now,
                        "c_fallback_drop_gate_latched_at_follow_height",
                    )
                else:
                    self.cd_follower.reset(
                        timestamp=data.now,
                        velocity_xy_m_s=data.velocity_xy_m_s,
                    )
                    self._transition(
                        Task1Phase.DROP_WINDOW_C_D,
                        data.now,
                        "centered_target_confirmed_at_c",
                    )

        elif self.phase == Task1Phase.DROP_WINDOW_C_D:
            target_world_height = cfg.follow_height_m
            path = self.cd_follower.command(
                position_xy, nominal_speed_m_s=car_speed, timestamp=data.now
            )
            target_xy, vx, vy = self._from_path(path)
            base_vx, base_vy = vx, vy
            descent_time = max(
                0.0, cfg.follow_height_m - cfg.drop_height_m
            ) / max(cfg.drop_descent_speed_m_s, 1e-3)
            available_time = path.remaining_m / max(car_speed, 1e-3)
            gate_has_time = available_time >= descent_time + cfg.drop_time_margin_s
            if (
                cfg.payload_drop_enabled
                and self.target_acquired
                and gate_has_time
                and self._confirm_vision(data, require_centered=True)
            ):
                self.drop_committed = True
                self._transition(
                    Task1Phase.DROP_DESCENT, data.now, "drop_gate_latched"
                )
            elif path.completed:
                self._begin_return(
                    data.now, position_xy, "d_reached_without_drop_gate"
                )

        elif self.phase == Task1Phase.DROP_DESCENT:
            target_world_height = cfg.follow_height_m
            target_deck_height = cfg.drop_height_m
            path = self.cd_follower.command(
                position_xy, nominal_speed_m_s=car_speed, timestamp=data.now
            )
            target_xy, vx, vy = self._from_path(path)
            base_vx, base_vy = vx, vy
            relative_height = data.deck_relative_height_m
            if (
                relative_height is not None
                and abs(relative_height - cfg.drop_height_m)
                <= cfg.drop_rel_height_tolerance_m
            ):
                release_requested = True
                self._release_motion = "cd"
                self._transition(
                    Task1Phase.RELEASING, data.now, "drop_height_reached"
                )
            elif path.completed:
                self._begin_return(
                    data.now, position_xy, "d_reached_before_drop_height"
                )

        elif self.phase == Task1Phase.RELEASING:
            target_world_height = cfg.follow_height_m
            release_requested = True
            if self._release_motion == "vision_hold":
                target_xy = self._acquire_anchor
                trim_vx, trim_vy = self._acquire_vision_command(data)
                vx, vy = trim_vx, trim_vy
            elif self._release_motion == "bc":
                path = self.bc_follower.command(
                    position_xy,
                    nominal_speed_m_s=curve_speed,
                    timestamp=data.now,
                )
                target_xy, vx, vy = self._from_path(path)
            elif self._release_motion == "cd":
                target_deck_height = cfg.drop_height_m
                path = self.cd_follower.command(
                    position_xy,
                    nominal_speed_m_s=car_speed,
                    timestamp=data.now,
                )
                target_xy, vx, vy = self._from_path(path)
            else:
                target_xy = C
                vx, vy = self._point_velocity(
                    position_xy, C, cfg.hold_position_max_speed_m_s
                )
            if self._release_motion != "vision_hold":
                base_vx, base_vy = vx, vy
            if data.payload_state == ActuatorState.RELEASED:
                self.drop_released = True
                self.mission_success = True
                self._begin_return(data.now, position_xy, "payload_released")
            elif data.payload_state == ActuatorState.UNCERTAIN:
                self._begin_return(
                    data.now, position_xy, "payload_release_uncertain"
                )
            elif data.now - self._phase_since >= cfg.release_timeout_s:
                self._begin_return(
                    data.now, position_xy, "payload_release_timeout"
                )

        elif self.phase == Task1Phase.CLIMB:
            target_xy = self._climb_anchor
            vx, vy = self._point_velocity(
                position_xy,
                self._climb_anchor,
                cfg.hold_position_max_speed_m_s,
            )
            if z_world >= cfg.cruise_height_m - 0.10:
                self._transition(Task1Phase.RETURN_H, data.now, self.reason)

        elif self.phase == Task1Phase.RETURN_H:
            target_xy = H
            vx, vy = self._point_velocity(position_xy, H, cfg.return_speed_m_s)
            if self._near(position_xy, H):
                self._transition(Task1Phase.LAND_H, data.now, "h_reached")

        elif self.phase == Task1Phase.LAND_H:
            target_xy = H
            target_world_height = cfg.final_height_m
            vx, vy = self._point_velocity(position_xy, H, 0.10)
            landing_gate_ready = (
                z_world <= cfg.final_height_m + 0.05
                and math.hypot(*position_xy) <= cfg.final_landing_radius_m
                and math.hypot(*data.velocity_xy_m_s)
                <= cfg.final_landing_max_speed_m_s
            )
            if landing_gate_ready:
                if self._stable_since is None:
                    self._stable_since = data.now
                land_requested = (
                    data.now - self._stable_since
                    >= cfg.final_landing_stable_s
                )
            else:
                self._stable_since = None
            if data.landed:
                self._transition(Task1Phase.COMPLETE, data.now, "landed_at_h")

        elif self.phase == Task1Phase.COMPLETE:
            target_xy = H
            target_world_height = cfg.final_height_m

        return Task1Command(
            phase=self.phase,
            target_xy_m=target_xy,
            target_world_height_m=target_world_height,
            target_deck_height_m=target_deck_height,
            vx_m_s=vx,
            vy_m_s=vy,
            base_vx_m_s=base_vx if (base_vx or base_vy) else vx - trim_vx,
            base_vy_m_s=base_vy if (base_vx or base_vy) else vy - trim_vy,
            vision_trim_vx_m_s=trim_vx,
            vision_trim_vy_m_s=trim_vy,
            takeoff_requested=takeoff_requested,
            release_requested=release_requested,
            land_requested=land_requested,
            fixed_heading=True,
            target_acquired=self.target_acquired,
            drop_committed=self.drop_committed,
            drop_released=self.drop_released,
            mission_success=self.mission_success,
            reason=self.reason,
        )

    def _confirm_vision(
        self,
        data: Task1Input,
        *,
        require_centered: bool,
        max_error_m: float | None = None,
        min_duration_s: float = 0.0,
        confirm_frames: int | None = None,
    ) -> bool:
        valid = (
            data.vision_seq is not None
            and data.vision_found
            and not data.vision_ambiguous
            and data.vision_quality >= self.config.vision_min_quality
        )
        if require_centered:
            error_limit = (
                self.config.drop_max_error_m
                if max_error_m is None
                else max(0.0, float(max_error_m))
            )
            valid = (
                valid
                and data.vision_error_xy_m is not None
                and math.hypot(*data.vision_error_xy_m)
                <= error_limit
            )
        if not valid:
            self._vision_confirm_count = 0
            self._last_vision_seq = None
            self._vision_confirm_since = None
            return False
        if data.vision_seq != self._last_vision_seq:
            self._last_vision_seq = data.vision_seq
            if self._vision_confirm_count == 0:
                self._vision_confirm_since = data.now
            self._vision_confirm_count += 1
        elapsed = (
            0.0
            if self._vision_confirm_since is None
            else max(0.0, data.now - self._vision_confirm_since)
        )
        return (
            self._vision_confirm_count >= (
                self.config.vision_confirm_frames
                if confirm_frames is None
                else max(1, int(confirm_frames))
            )
            and elapsed >= max(0.0, float(min_duration_s))
        )

    def _vision_trim_command(
        self, data: Task1Input
    ) -> tuple[float, float]:
        cfg = self.config
        trim, timestamp = self._slew_vision_command(
            data,
            min_quality=cfg.vision_min_quality,
            kp=cfg.vision_trim_kp,
            deadband_m=cfg.vision_trim_deadband_m,
            max_speed_m_s=cfg.vision_trim_max_speed_m_s,
            max_accel_m_s2=cfg.vision_trim_max_accel_m_s2,
            previous=self._vision_trim_xy_m_s,
            previous_time=self._vision_trim_time,
        )
        self._vision_trim_xy_m_s = trim
        self._vision_trim_time = timestamp
        return trim

    def _acquire_vision_command(
        self, data: Task1Input
    ) -> tuple[float, float]:
        cfg = self.config
        now = float(data.now)
        valid = self._vision_is_trackable(data)
        if valid:
            assert data.vision_seq is not None
            assert data.vision_error_xy_m is not None
            if data.vision_seq != self._acquire_vision_last_seq:
                error_x, error_y = data.vision_error_xy_m
                self._acquire_vision_errors.append(
                    (now, data.vision_seq, error_x, error_y)
                )
                self._acquire_vision_last_seq = data.vision_seq
            self._acquire_vision_last_valid_time = now

        cutoff = now - max(0.05, cfg.acquire_vision_filter_window_s)
        while (
            self._acquire_vision_errors
            and self._acquire_vision_errors[0][0] < cutoff
        ):
            self._acquire_vision_errors.popleft()

        # 检测帧持续收集，但飞行速度只以固定低频率更新。
        if (
            self._acquire_vision_control_time is not None
            and now - self._acquire_vision_control_time
            < cfg.acquire_vision_control_period_s
        ):
            return self._acquire_vision_xy_m_s

        recent_target = (
            self._acquire_vision_last_valid_time is not None
            and now - self._acquire_vision_last_valid_time
            <= cfg.acquire_vision_loss_grace_s
            and bool(self._acquire_vision_errors)
        )
        desired_x = desired_y = 0.0
        if recent_target:
            # 中位数比逐帧值或均值更不容易被单帧跳点拖动。
            error_x = median(item[2] for item in self._acquire_vision_errors)
            error_y = median(item[3] for item in self._acquire_vision_errors)
            error_norm = math.hypot(error_x, error_y)
            if error_norm > cfg.acquire_vision_deadband_m:
                active_scale = (
                    error_norm - cfg.acquire_vision_deadband_m
                ) / error_norm
                desired_x = cfg.acquire_vision_kp * error_x * active_scale
                desired_y = cfg.acquire_vision_kp * error_y * active_scale
                desired_x, desired_y = _limit_norm(
                    desired_x,
                    desired_y,
                    cfg.acquire_vision_max_speed_m_s,
                )

        previous_x, previous_y = self._acquire_vision_xy_m_s
        if self._acquire_vision_control_time is None:
            dt = 0.05
        else:
            dt = min(
                max(0.0, now - self._acquire_vision_control_time),
                max(0.20, cfg.acquire_vision_control_period_s * 2.0),
            )
        max_delta = cfg.acquire_vision_max_accel_m_s2 * dt
        delta_x, delta_y = _limit_norm(
            desired_x - previous_x,
            desired_y - previous_y,
            max_delta,
        )
        command = (previous_x + delta_x, previous_y + delta_y)
        self._acquire_vision_xy_m_s = command
        self._acquire_vision_control_time = now
        return command

    def _vision_is_trackable(self, data: Task1Input) -> bool:
        return (
            data.vision_seq is not None
            and data.vision_found
            and not data.vision_ambiguous
            and data.vision_quality >= self.config.acquire_vision_min_quality
            and data.vision_error_xy_m is not None
        )

    @staticmethod
    def _slew_vision_command(
        data: Task1Input,
        *,
        min_quality: int,
        kp: float,
        deadband_m: float,
        max_speed_m_s: float,
        max_accel_m_s2: float,
        previous: tuple[float, float],
        previous_time: float | None,
    ) -> tuple[tuple[float, float], float]:
        desired_x = desired_y = 0.0
        valid = (
            data.vision_seq is not None
            and data.vision_found
            and not data.vision_ambiguous
            and data.vision_quality >= min_quality
            and data.vision_error_xy_m is not None
        )
        if valid:
            error_x, error_y = data.vision_error_xy_m
            error_norm = math.hypot(error_x, error_y)
            if error_norm > deadband_m:
                active_scale = (
                    error_norm - deadband_m
                ) / error_norm
                desired_x = kp * error_x * active_scale
                desired_y = kp * error_y * active_scale
                desired_x, desired_y = _limit_norm(
                    desired_x,
                    desired_y,
                    max_speed_m_s,
                )

        previous_x, previous_y = previous
        if previous_time is None:
            dt = 0.05
        else:
            dt = min(max(0.0, data.now - previous_time), 0.20)
        max_delta = max_accel_m_s2 * dt
        delta_x, delta_y = _limit_norm(
            desired_x - previous_x,
            desired_y - previous_y,
            max_delta,
        )
        trim = (previous_x + delta_x, previous_y + delta_y)
        return trim, data.now

    def _point_velocity(
        self,
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
        speed_limit: float,
    ) -> tuple[float, float]:
        dx = target_xy[0] - position_xy[0]
        dy = target_xy[1] - position_xy[1]
        vx = self.config.point_kp * dx
        vy = self.config.point_kp * dy
        return _limit_norm(
            vx,
            vy,
            min(self.config.max_point_speed_m_s, max(0.0, speed_limit)),
        )

    def _near(
        self, position_xy: tuple[float, float], target_xy: tuple[float, float]
    ) -> bool:
        return math.hypot(
            position_xy[0] - target_xy[0],
            position_xy[1] - target_xy[1],
        ) <= self.config.point_arrival_radius_m

    @staticmethod
    def _from_path(
        path: PathCommand,
    ) -> tuple[tuple[float, float], float, float]:
        return path.target_xy_m, path.vx_m_s, path.vy_m_s

    def _begin_return(
        self,
        now: float,
        position_xy: tuple[float, float],
        reason: str,
    ) -> None:
        self._climb_anchor = position_xy
        self._transition(Task1Phase.CLIMB, now, reason)

    def _transition(
        self, phase: Task1Phase, now: float, reason: str
    ) -> None:
        self.phase = phase
        self._phase_since = float(now)
        self.reason = reason
        self._stable_since = None
        self._vision_confirm_count = 0
        self._last_vision_seq = None
        self._vision_confirm_since = None
        if phase == Task1Phase.ACQUIRE_TARGET:
            self._vision_takeover_ready = not self.config.vision_track_only
            self._acquire_vision_xy_m_s = (0.0, 0.0)
            self._acquire_vision_control_time = None
            self._acquire_vision_last_valid_time = None
            self._acquire_vision_last_seq = None
            self._acquire_vision_errors.clear()


def _limit_norm(x: float, y: float, limit: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= limit or norm <= 1e-9:
        return x, y
    scale = limit / norm
    return x * scale, y * scale
