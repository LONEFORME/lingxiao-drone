"""D题任务二的纯任务状态机。

本模块不直接访问串口、GPIO或飞控，也不持有 PlatformTracker、
FormationController 或 DynamicLandingController。飞行适配层每个控制周期
提供 T265、激光、视觉、小车坐标和动态降落子状态机的反馈，再执行返回的
水平速度、高度目标与起飞/降落请求。

任务二与任务一的区别：C 点之前复用任务一的路径跟随+视觉对中逻辑；
在 SYNC_TARGET_AT_C 完成后不进入投放，而是激活 T265 坐标系下的精确
伴飞与动态降落。
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


# 场地航点（与任务一一致）
H = (0.0, 0.0)
B_PRE = (0.375, 1.75)
B = (0.375, 2.25)
BC_1 = (0.595, 2.78)
BC_2 = (1.125, 3.00)
BC_3 = (1.655, 2.78)
C = (1.875, 2.25)
D = (1.875, 0.75)
# 任务二视觉降落模式的H→C分段定点。两点位于直线H-C的1/3和2/3处，
# 每个点都要求减速并重新收敛后才继续，降低长距离直飞C点的过冲。
C_TRANSIT_1 = (0.625, 0.75)
C_TRANSIT_2 = (1.250, 1.50)
RETURN_H_FRACTIONS = (2.0 / 3.0, 1.0 / 3.0)


class Task2Phase(enum.Enum):
    WAIT_START = "wait_start"
    TAKEOFF = "takeoff"
    HOLD_3S = "hold_3s"
    INTERCEPT_B_PRE = "intercept_b_pre"
    ACQUIRE_TARGET = "acquire_target"
    FOLLOW_B_C = "follow_b_c"
    TRANSIT_C = "transit_c"
    SYNC_TARGET_AT_C = "sync_target_at_c"
    OPEN_LOOP_C_D = "open_loop_c_d"
    SAFE_HOVER_D = "safe_hover_d"
    SAFE_HOVER_AFTER_RETAKEOFF = "safe_hover_after_retakeoff"
    LANDED_ON_PLATFORM = "landed_on_platform"
    ACTIVATE_TRACKER = "activate_tracker"
    DYNAMIC_LANDING = "dynamic_landing"
    RETAKEOFF = "retakeoff"
    STABILIZE_AFTER_RETAKEOFF = "stabilize_after_retakeoff"
    VISUAL_RETURN_A = "visual_return_a"
    CLIMB_150CM = "climb_150cm"
    RETURN_H = "return_h"
    LAND_H = "land_h"
    COMPLETE = "complete"


@dataclass(frozen=True)
class Task2Config:
    # 路径与巡航参数（与任务一一致）
    cruise_height_m: float = 1.30
    follow_height_m: float = 1.30
    final_height_m: float = 0.15
    hold_duration_s: float = 3.0
    intercept_speed_m_s: float = 0.20
    car_speed_m_s: float = 0.13
    curve_speed_m_s: float | None = None
    car_speed_scale: float = 1.0
    return_speed_m_s: float = 0.35
    point_kp: float = 0.90
    point_arrival_radius_m: float = 0.14
    max_point_speed_m_s: float = 0.40
    hold_position_max_speed_m_s: float = 0.15
    hold_velocity_kd: float = 0.0
    hold_stable_speed_m_s: float = 0.12
    path_lookahead_m: float = 0.20
    path_cross_track_kp: float = 0.80
    path_max_correction_m_s: float = 0.08
    path_max_speed_m_s: float = 0.20
    path_max_accel_m_s2: float = 0.30
    acquire_timeout_s: float = 4.0
    vision_min_quality: int = 55
    vision_confirm_frames: int = 2
    drop_max_error_m: float = 0.12
    t265_min_confidence: int = 2
    path_only_b_pre_descent: bool = False
    c_sync_vision_enabled: bool = True
    # 任务二特有参数
    retakeoff_height_m: float = 1.30
    abort_climb_height_m: float = 1.50
    activate_tracker_timeout_s: float = 3.0
    require_car_position_at_c: bool = True
    safe_open_loop_cd_test: bool = False
    safe_hover_height_m: float = 0.50
    safe_c_offset_x_m: float = 0.0
    safe_c_offset_y_m: float = 0.0
    safe_descent_rate_m_s: float = 0.03
    safe_follow_cutoff_s: float = 25.0
    vision_landing_test: bool = False
    direct_descent_after_trigger: bool = False
    fc_one_key_land_enabled: bool = False
    fc_one_key_land_height_m: float = 0.10
    fc_direct_lock_enabled: bool = False
    fc_direct_lock_height_m: float = 0.05
    fc_direct_lock_stable_height_m: float = 0.10
    fc_direct_lock_stable_hold_s: float = 0.35
    fc_direct_lock_stable_tolerance_m: float = 0.01
    fc_direct_lock_stable_min_samples: int = 4
    fc_direct_lock_laser_dropout_grace_s: float = 0.25
    platform_retakeoff_enabled: bool = False
    platform_locked_hold_s: float = 5.0
    platform_task_reset_hold_s: float = 0.30
    retakeoff_stabilize_s: float = 0.80
    retakeoff_forward_y_speed_m_s: float = -0.05
    retakeoff_t265_jump_protection_enabled: bool = False
    retakeoff_h_offset_x_m: float = -0.50
    retakeoff_h_offset_y_m: float = -0.60
    visual_return_a_enabled: bool = False
    visual_return_a_center_radius_m: float = 0.15
    visual_return_a_stop_speed_m_s: float = 0.015
    visual_return_a_stop_hold_s: float = 1.0
    visual_return_a_min_follow_s: float = 2.0
    visual_return_a_timeout_s: float = 30.0
    visual_return_a_kp: float = 0.65
    visual_return_a_max_speed_m_s: float = 0.15
    visual_return_a_max_accel_m_s2: float = 0.45
    return_h_auto_land_timeout_s: float = 15.0
    land_h_auto_descend_timeout_s: float = 8.0
    hover_wait_auto_land_delay_s: float = 3.0
    vision_target_offset_x_m: float = 0.0
    vision_target_offset_y_m: float = 0.0
    c_sync_vision_kp: float = 0.45
    c_sync_vision_deadband_m: float = 0.03
    c_sync_vision_max_speed_m_s: float = 0.10
    c_sync_vision_max_accel_m_s2: float = 0.25
    c_sync_vision_filter_window_s: float = 0.20
    c_sync_vision_control_period_s: float = 0.08
    c_sync_vision_loss_grace_s: float = 0.20
    landing_xy_speed_high_m_s: float = 0.15
    landing_xy_speed_mid_m_s: float = 0.10
    landing_xy_speed_low_m_s: float = 0.07
    stationary_retakeoff_test: bool = False
    stationary_skip_vision: bool = False
    land_only_after_touchdown: bool = False
    stationary_test_point_x_m: float = 0.0
    stationary_test_point_y_m: float = 0.50


@dataclass(frozen=True)
class Task2Input:
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
    # 任务二特有
    car_position_xy_m: tuple[float, float] | None = None
    car_velocity_xy_m_s: tuple[float, float] | None = None
    offset_ready: bool = False
    # 动态降落子状态机反馈（由飞行适配层提取）
    landing_gate_passed: bool = False
    touchdown_confirmed: bool = False
    deck_ride_complete: bool = False
    landing_aborted: bool = False


@dataclass(frozen=True)
class Task2Command:
    phase: Task2Phase
    target_xy_m: tuple[float, float]
    target_world_height_m: float
    target_deck_height_m: float | None
    vx_m_s: float
    vy_m_s: float
    takeoff_requested: bool
    land_requested: bool
    tracker_active: bool
    landing_active: bool
    keep_armed: bool
    fixed_heading: bool
    mission_success: bool
    reason: str


class Task2MissionDirector:
    """任务二决策源。

    C 点之前复用任务一的路径跟随与视觉对中逻辑；C 点之后不投放，而是
    激活 T265 坐标系下的精确伴飞与动态降落。本类不直接持有控制模块，
    通过 tracker_active/landing_active 标志通知飞行适配层切换控制源。
    """

    def __init__(self, config: Task2Config | None = None) -> None:
        self.config = config or Task2Config()
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
        if self.config.stationary_retakeoff_test:
            self.sync_c = (
                self.config.stationary_test_point_x_m,
                self.config.stationary_test_point_y_m,
            )
        else:
            self.sync_c = (
                C[0] + (
                    self.config.safe_c_offset_x_m
                    if self.config.safe_open_loop_cd_test
                    else 0.0
                ),
                C[1] + (
                    self.config.safe_c_offset_y_m
                    if self.config.safe_open_loop_cd_test
                    else 0.0
                ),
            )
        self.cd_follower = Task1PathFollower(
            PolylinePath((self.sync_c, D)), path_cfg
        )
        self.phase = Task2Phase.WAIT_START
        self._phase_since = 0.0
        self._stable_since: float | None = None
        self._last_vision_seq: int | None = None
        self._vision_confirm_count = 0
        self.target_acquired = False
        self.mission_success = False
        self.reason = "waiting_for_car_start"
        self._climb_anchor = H
        self._safe_hover_anchor = D
        self._retakeoff_anchor = H
        self._retakeoff_forward_last_time: float | None = None
        self._return_h_target = H
        self._retakeoff_local_frame_pending = False
        self._transit_c_waypoints = (
            (C_TRANSIT_1, C_TRANSIT_2, self.sync_c)
            if self.config.vision_landing_test
            and not self.config.stationary_retakeoff_test
            else (self.sync_c,)
        )
        self._transit_c_index = 0
        self._sync_vision_errors: deque[tuple[float, int, float, float]] = deque()
        self._sync_vision_last_seq: int | None = None
        self._sync_vision_last_valid_time: float | None = None
        self._sync_vision_control_time: float | None = None
        self._sync_vision_xy_m_s = (0.0, 0.0)
        self._sync_vision_active = False
        self._sync_hold_anchor = self.sync_c
        self._visual_return_a_stopped_since: float | None = None
        self._return_h_waypoints = (H,)
        self._return_h_index = 0

    def begin_platform_retakeoff(
        self,
        *,
        now: float,
        anchor_xy_m: tuple[float, float],
    ) -> None:
        """Enter the existing retakeoff/return-H path after a locked stop."""
        if not self.config.platform_retakeoff_enabled:
            raise RuntimeError("platform retakeoff is disabled")
        anchor = (float(anchor_xy_m[0]), float(anchor_xy_m[1]))
        if not all(math.isfinite(value) for value in anchor):
            raise ValueError("retakeoff anchor must contain finite values")
        self._retakeoff_anchor = anchor
        self._retakeoff_local_frame_pending = True
        self.mission_success = True
        self._transition(
            Task2Phase.RETAKEOFF,
            float(now),
            "platform_lock_hold_complete_retakeoff",
        )

    def tick(self, data: Task2Input) -> Task2Command:
        cfg = self.config
        x, y, z_world = data.position_xyz_m
        position_xy = (x, y)
        raw_car_speed = (
            cfg.car_speed_m_s
            if cfg.safe_open_loop_cd_test or data.car_speed_m_s is None
            else max(0.0, float(data.car_speed_m_s))
        )
        car_speed = raw_car_speed * max(0.0, cfg.car_speed_scale)
        curve_speed = (
            car_speed
            if cfg.curve_speed_m_s is None
            else max(0.0, cfg.curve_speed_m_s) * max(0.0, cfg.car_speed_scale)
        )
        target_xy = position_xy
        target_world_height = cfg.cruise_height_m
        target_deck_height: float | None = None
        vx = vy = 0.0
        takeoff_requested = False
        land_requested = False
        tracker_active = False
        landing_active = False

        if self.phase == Task2Phase.WAIT_START:
            target_xy = H
            if data.car_start and data.t265_confidence >= cfg.t265_min_confidence:
                self._transition(Task2Phase.TAKEOFF, data.now, "car_start_accepted")

        elif self.phase == Task2Phase.TAKEOFF:
            target_xy = H
            vx, vy = self._hold_velocity(
                position_xy,
                H,
                data.velocity_xy_m_s,
                cfg.hold_position_max_speed_m_s,
            )
            takeoff_requested = True
            if z_world >= cfg.cruise_height_m - 0.10:
                self._transition(
                    Task2Phase.HOLD_3S, data.now, "takeoff_height_reached"
                )
                self._stable_since = None

        elif self.phase == Task2Phase.HOLD_3S:
            target_xy = H
            vx, vy = self._hold_velocity(
                position_xy,
                H,
                data.velocity_xy_m_s,
                cfg.hold_position_max_speed_m_s,
            )
            hold_elapsed = data.now - self._phase_since
            height_safe = abs(z_world - cfg.cruise_height_m) <= 0.15
            if hold_elapsed >= cfg.hold_duration_s and height_safe:
                if (
                    cfg.safe_open_loop_cd_test
                    or cfg.vision_landing_test
                    or cfg.stationary_retakeoff_test
                ):
                    self._transition(
                        Task2Phase.TRANSIT_C,
                        data.now,
                        (
                            "stationary_test_takeoff_height_reached"
                            if cfg.stationary_retakeoff_test
                            else (
                                "vision_landing_test_takeoff_height_reached"
                                if cfg.vision_landing_test
                                else "safe_test_takeoff_height_reached"
                            )
                        ),
                    )
                else:
                    self._transition(
                        Task2Phase.INTERCEPT_B_PRE,
                        data.now,
                        "hover_3s_complete",
                    )

        elif self.phase == Task2Phase.TRANSIT_C:
            target_xy = self._transit_c_waypoints[self._transit_c_index]
            target_world_height = cfg.cruise_height_m
            vx, vy = self._point_velocity(
                position_xy, target_xy, cfg.intercept_speed_m_s
            )
            is_intermediate = self._transit_c_index < (
                len(self._transit_c_waypoints) - 1
            )
            arrival_radius = (
                min(cfg.point_arrival_radius_m, 0.10)
                if is_intermediate
                else cfg.point_arrival_radius_m
            )
            stable_speed = (
                min(cfg.hold_stable_speed_m_s, 0.08)
                if is_intermediate
                else cfg.hold_stable_speed_m_s
            )
            arrived = (
                math.dist(position_xy, target_xy) <= arrival_radius
                and math.hypot(*data.velocity_xy_m_s) <= stable_speed
            )
            if arrived:
                if is_intermediate:
                    self._transit_c_index += 1
                    self.reason = (
                        f"vision_landing_transit_waypoint_"
                        f"{self._transit_c_index}_reached"
                    )
                else:
                    self._transition(
                        Task2Phase.SYNC_TARGET_AT_C,
                        data.now,
                        (
                            "stationary_test_point_reached"
                            if cfg.stationary_retakeoff_test
                            else (
                                "vision_landing_test_c_reached"
                                if cfg.vision_landing_test
                                else "safe_test_c_reached"
                            )
                        ),
                    )

        elif self.phase == Task2Phase.INTERCEPT_B_PRE:
            target_xy = B_PRE
            vx, vy = self._point_velocity(
                position_xy, B_PRE, cfg.intercept_speed_m_s
            )
            if self._near(position_xy, B_PRE) and math.hypot(
                *data.velocity_xy_m_s
            ) <= cfg.hold_stable_speed_m_s:
                self._transition(
                    Task2Phase.ACQUIRE_TARGET, data.now, "b_pre_reached"
                )

        elif self.phase == Task2Phase.ACQUIRE_TARGET:
            target_xy = B_PRE
            vx, vy = self._point_velocity(
                position_xy, B_PRE, cfg.hold_position_max_speed_m_s
            )
            if cfg.path_only_b_pre_descent:
                target_world_height = cfg.follow_height_m
                if abs(z_world - cfg.follow_height_m) <= 0.10:
                    self.target_acquired = True
                    self.bc_follower.reset(
                        timestamp=data.now,
                        velocity_xy_m_s=data.velocity_xy_m_s,
                    )
                    self._transition(
                        Task2Phase.FOLLOW_B_C,
                        data.now,
                        "path_test_b_pre_descent_complete",
                    )
            elif self._confirm_vision(data, require_centered=True):
                self.target_acquired = True
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task2Phase.FOLLOW_B_C, data.now, "target_acquired_at_b_pre"
                )
            elif data.now - self._phase_since >= cfg.acquire_timeout_s:
                self.bc_follower.reset(
                    timestamp=data.now, velocity_xy_m_s=data.velocity_xy_m_s
                )
                self._transition(
                    Task2Phase.FOLLOW_B_C,
                    data.now,
                    "b_pre_timeout_path_fallback",
                )

        elif self.phase == Task2Phase.FOLLOW_B_C:
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
            target_xy, vx, vy = self._from_path(path)
            if path.completed:
                if cfg.c_sync_vision_enabled:
                    self._transition(
                        Task2Phase.SYNC_TARGET_AT_C,
                        data.now,
                        "c_reached_waiting_for_centered_target",
                    )
                else:
                    self._transition(
                        Task2Phase.SYNC_TARGET_AT_C, data.now, "c_reached"
                    )

        elif self.phase == Task2Phase.SYNC_TARGET_AT_C:
            target_world_height = cfg.follow_height_m
            active_visual_centering = (
                cfg.vision_landing_test
                and cfg.c_sync_vision_enabled
                and not cfg.stationary_retakeoff_test
            )
            if active_visual_centering:
                visual_command, target_recent = self._sync_vision_command(data)
                if target_recent:
                    self._sync_vision_active = True
                    self._sync_hold_anchor = position_xy
                    target_xy = (
                        position_xy
                        if data.vision_error_xy_m is None
                        else (
                            position_xy[0] + data.vision_error_xy_m[0],
                            position_xy[1] + data.vision_error_xy_m[1],
                        )
                    )
                    vx, vy = visual_command
                    self.reason = "c_sync_visual_centering"
                else:
                    if self._sync_vision_active:
                        self._sync_hold_anchor = position_xy
                        self._sync_vision_active = False
                    target_xy = self._sync_hold_anchor
                    vx, vy = self._hold_velocity(
                        position_xy,
                        self._sync_hold_anchor,
                        data.velocity_xy_m_s,
                        cfg.hold_position_max_speed_m_s,
                    )
                    self.reason = "c_sync_waiting_for_visual"
            else:
                target_xy = self.sync_c
                vx, vy = self._point_velocity(
                    position_xy, self.sync_c, cfg.hold_position_max_speed_m_s
                )
            ready = True
            if (
                cfg.c_sync_vision_enabled
                and not (
                    cfg.stationary_retakeoff_test
                    and cfg.stationary_skip_vision
                )
                and not self._confirm_vision(data, require_centered=True)
            ):
                ready = False
            if cfg.require_car_position_at_c and not (
                data.offset_ready and data.car_position_xy_m is not None
            ):
                ready = False
            if ready:
                if cfg.safe_open_loop_cd_test:
                    self.cd_follower.reset(
                        timestamp=data.now,
                        velocity_xy_m_s=data.velocity_xy_m_s,
                    )
                    self._transition(
                        Task2Phase.OPEN_LOOP_C_D,
                        data.now,
                        "safe_test_c_vision_centered",
                    )
                elif cfg.direct_descent_after_trigger:
                    # 视觉连续居中已经是唯一降落门槛；同一周期直接把控制权
                    # 交给持续下降链路，避免再经过ACTIVATE_TRACKER二次等待。
                    tracker_active = True
                    landing_active = True
                    target_xy = position_xy
                    vx = vy = 0.0
                    self._transition(
                        Task2Phase.DYNAMIC_LANDING,
                        data.now,
                        "c_sync_ready_direct_descent",
                    )
                else:
                    self._transition(
                        Task2Phase.ACTIVATE_TRACKER,
                        data.now,
                        (
                            "stationary_test_fixed_point_ready_no_vision"
                            if cfg.stationary_skip_vision
                            else "c_sync_ready"
                        ),
                    )

        elif self.phase == Task2Phase.OPEN_LOOP_C_D:
            path = self.cd_follower.command(
                position_xy,
                nominal_speed_m_s=car_speed,
                timestamp=data.now,
            )
            target_xy, vx, vy = self._from_path(path)
            # 下降目标仍由实际路径进度驱动，避免水平受阻时按时间盲降；
            # 但下降速率与整段长度解耦，可在到D前提前到达0.5m。
            height_drop = path.progress_m * (
                max(0.0, cfg.safe_descent_rate_m_s)
                / max(car_speed, 1e-3)
            )
            target_world_height = max(
                cfg.safe_hover_height_m,
                cfg.cruise_height_m - height_drop,
            )
            if (
                target_world_height <= cfg.safe_hover_height_m + 1e-6
                and z_world <= cfg.safe_hover_height_m + 0.08
            ):
                # 达到0.5m测试高度后立即终止水平伴随，保持当前位置悬停。
                self._safe_hover_anchor = position_xy
                target_xy = position_xy
                vx = vy = 0.0
                self._transition(
                    Task2Phase.SAFE_HOVER_D,
                    data.now,
                    "safe_test_50cm_reached_stop_forward_motion",
                )
            elif data.now - self._phase_since >= cfg.safe_follow_cutoff_s:
                # 小车即将到D转弯；安全测试在固定世界坐标悬停，绝不跟随转弯。
                self._safe_hover_anchor = position_xy
                target_xy = position_xy
                vx = vy = 0.0
                self._transition(
                    Task2Phase.SAFE_HOVER_D,
                    data.now,
                    "safe_test_cutoff_before_car_reaches_d",
                )
            elif path.completed:
                target_xy = D
                vx, vy = self._point_velocity(
                    position_xy, D, cfg.hold_position_max_speed_m_s
                )
                if (
                    z_world <= cfg.safe_hover_height_m + 0.08
                    and math.hypot(*data.velocity_xy_m_s)
                    <= cfg.hold_stable_speed_m_s
                ):
                    self._safe_hover_anchor = D
                    self._transition(
                        Task2Phase.SAFE_HOVER_D,
                        data.now,
                        "safe_test_d_reached_at_50cm",
                    )

        elif self.phase == Task2Phase.SAFE_HOVER_D:
            target_xy = self._safe_hover_anchor
            target_world_height = cfg.safe_hover_height_m
            vx, vy = self._point_velocity(
                position_xy,
                self._safe_hover_anchor,
                cfg.hold_position_max_speed_m_s,
            )
            self.reason = "safe_test_hovering_before_d_no_landing"

        elif self.phase == Task2Phase.ACTIVATE_TRACKER:
            # 激活 PlatformTracker 和 DynamicLandingController；水平控制仍由
            # 点位控制保持悬停在 C 点（vz 不使用），待 LANDING_GATE 通过后
            # 切换到 FormationController
            tracker_active = True
            landing_active = True
            target_xy = self.sync_c
            target_world_height = cfg.follow_height_m
            vx, vy = self._hold_velocity(
                position_xy,
                self.sync_c,
                data.velocity_xy_m_s,
                cfg.hold_position_max_speed_m_s,
            )
            if data.landing_aborted:
                self._begin_climb(data.now, position_xy, "landing_aborted_at_gate")
            elif data.landing_gate_passed:
                self._transition(
                    Task2Phase.DYNAMIC_LANDING, data.now, "landing_gate_passed"
                )
            elif data.now - self._phase_since >= cfg.activate_tracker_timeout_s:
                self._begin_climb(data.now, position_xy, "activate_tracker_timeout")

        elif self.phase == Task2Phase.DYNAMIC_LANDING:
            # 委托给 DynamicLandingController，水平控制交给 FormationController
            tracker_active = True
            landing_active = True
            target_xy = position_xy
            target_world_height = cfg.follow_height_m
            if data.landing_aborted:
                self._begin_climb(data.now, position_xy, "landing_aborted")
            elif data.deck_ride_complete and cfg.fc_direct_lock_enabled:
                # DynamicLandingController reaching RETAKEOFF_GATE only proves
                # that its touchdown sequence completed.  In direct-lock mode
                # the flight adapter must still send 102, confirm unlock_sta=0
                # and zero motor PWM, hold five seconds, then reset the FC task.
                # Never let this controller-local flag bypass that contract.
                self.reason = "awaiting_fc_direct_lock_confirmation"
            elif data.deck_ride_complete:
                self.mission_success = True
                self._retakeoff_anchor = position_xy
                self._retakeoff_local_frame_pending = True
                if cfg.land_only_after_touchdown:
                    self._safe_hover_anchor = position_xy
                    self._transition(
                        Task2Phase.LANDED_ON_PLATFORM,
                        data.now,
                        "deck_ride_complete_land_only",
                    )
                else:
                    self._transition(
                        Task2Phase.RETAKEOFF, data.now, "deck_ride_complete"
                    )

        elif self.phase == Task2Phase.LANDED_ON_PLATFORM:
            tracker_active = False
            landing_active = False
            target_xy = self._safe_hover_anchor
            target_world_height = 0.05
            vx = vy = 0.0
            self.reason = "landed_on_platform_waiting_for_manual_shutdown"

        elif self.phase == Task2Phase.RETAKEOFF:
            tracker_active = False
            landing_active = False
            target_world_height = cfg.retakeoff_height_m
            target_xy, vx, vy = self._retakeoff_forward_command(
                data,
                position_xy,
            )
            if z_world >= cfg.retakeoff_height_m - 0.10:
                if cfg.stationary_retakeoff_test:
                    self._safe_hover_anchor = self._retakeoff_anchor
                    self._transition(
                        Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF,
                        data.now,
                        "stationary_test_retakeoff_height_reached",
                    )
                elif cfg.platform_retakeoff_enabled:
                    self._safe_hover_anchor = self._retakeoff_anchor
                    self._transition(
                        Task2Phase.STABILIZE_AFTER_RETAKEOFF,
                        data.now,
                        "retakeoff_height_reached_stabilize",
                    )
                else:
                    self._begin_return_h(
                        data.now,
                        position_xy,
                        "retakeoff_height_reached",
                    )

        elif self.phase == Task2Phase.STABILIZE_AFTER_RETAKEOFF:
            target_world_height = cfg.retakeoff_height_m
            target_xy, vx, vy = self._retakeoff_forward_command(
                data,
                position_xy,
            )
            self._safe_hover_anchor = target_xy
            stable = (
                data.now - self._phase_since >= cfg.retakeoff_stabilize_s
                and abs(z_world - cfg.retakeoff_height_m) <= 0.15
                and math.hypot(*data.velocity_xy_m_s)
                <= cfg.hold_stable_speed_m_s
            )
            if stable:
                if cfg.visual_return_a_enabled:
                    self._begin_visual_return_a(data.now, position_xy)
                else:
                    self._begin_return_h(
                        data.now,
                        position_xy,
                        "retakeoff_stabilized_return_h",
                    )

        elif self.phase == Task2Phase.VISUAL_RETURN_A:
            target_world_height = cfg.retakeoff_height_m
            (vx, vy), target_recent = self._sync_vision_command(data)
            if target_recent and data.vision_error_xy_m is not None:
                target_xy = (
                    position_xy[0] + data.vision_error_xy_m[0],
                    position_xy[1] + data.vision_error_xy_m[1],
                )
            else:
                target_xy = position_xy

            follow_timed_out = (
                cfg.visual_return_a_timeout_s > 0.0
                and data.now - self._phase_since
                >= cfg.visual_return_a_timeout_s
            )
            if follow_timed_out:
                self._begin_return_h(
                    data.now,
                    position_xy,
                    "visual_return_a_30s_timeout",
                )
                target_xy = self._return_h_waypoints[0]
                vx, vy = self._point_velocity(
                    position_xy, target_xy, cfg.return_speed_m_s
                )
            else:
                centered = self._visual_return_a_centered(target_recent)
                if data.car_speed_m_s is not None:
                    stopped = (
                        abs(float(data.car_speed_m_s))
                        <= cfg.visual_return_a_stop_speed_m_s
                    )
                else:
                    stopped = (
                        math.hypot(*data.velocity_xy_m_s)
                        <= cfg.visual_return_a_stop_speed_m_s
                    )
                ready_to_confirm = (
                    data.now - self._phase_since
                    >= cfg.visual_return_a_min_follow_s
                    and centered
                    and stopped
                )
                if ready_to_confirm:
                    if self._visual_return_a_stopped_since is None:
                        self._visual_return_a_stopped_since = data.now
                    elif (
                        data.now - self._visual_return_a_stopped_since
                        >= cfg.visual_return_a_stop_hold_s
                    ):
                        self._begin_return_h(
                            data.now,
                            position_xy,
                            "car_a_visual_centered_and_stopped",
                        )
                        target_xy = self._return_h_waypoints[0]
                        vx, vy = self._point_velocity(
                            position_xy, target_xy, cfg.return_speed_m_s
                        )
                else:
                    self._visual_return_a_stopped_since = None
            if self.phase == Task2Phase.VISUAL_RETURN_A:
                self.reason = (
                    "visual_follow_car_to_a"
                    if target_recent
                    else "visual_return_a_target_lost_hover"
                )

        elif self.phase == Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF:
            target_xy = self._safe_hover_anchor
            target_world_height = cfg.retakeoff_height_m
            vx, vy = self._hold_velocity(
                position_xy,
                self._safe_hover_anchor,
                data.velocity_xy_m_s,
                cfg.hold_position_max_speed_m_s,
            )
            self.reason = "stationary_test_hovering_after_retakeoff_manual_land"

        elif self.phase == Task2Phase.CLIMB_150CM:
            target_xy = self._climb_anchor
            vx, vy = self._point_velocity(
                position_xy,
                self._climb_anchor,
                cfg.hold_position_max_speed_m_s,
            )
            if z_world >= cfg.abort_climb_height_m - 0.10:
                self._begin_return_h(data.now, position_xy, self.reason)

        elif self.phase == Task2Phase.RETURN_H:
            target_xy = self._return_h_waypoints[self._return_h_index]
            vx, vy = self._point_velocity(
                position_xy, target_xy, cfg.return_speed_m_s
            )
            is_intermediate = self._return_h_index < (
                len(self._return_h_waypoints) - 1
            )
            arrival_radius = (
                min(cfg.point_arrival_radius_m, 0.10)
                if is_intermediate
                else cfg.point_arrival_radius_m
            )
            stable_speed = (
                min(cfg.hold_stable_speed_m_s, 0.08)
                if is_intermediate
                else min(cfg.hold_stable_speed_m_s, 0.10)
            )
            arrived = (
                math.dist(position_xy, target_xy) <= arrival_radius
                and math.hypot(*data.velocity_xy_m_s) <= stable_speed
            )
            return_timed_out = (
                cfg.return_h_auto_land_timeout_s > 0.0
                and data.now - self._phase_since
                >= cfg.return_h_auto_land_timeout_s
            )
            if return_timed_out:
                self._return_h_target = position_xy
                self._return_h_waypoints = (position_xy,)
                self._return_h_index = 0
                target_xy = position_xy
                target_world_height = cfg.final_height_m
                vx = vy = 0.0
                self._transition(
                    Task2Phase.LAND_H,
                    data.now,
                    "return_h_timeout_land_current_position",
                )
            elif arrived:
                if is_intermediate:
                    self._return_h_index += 1
                    self.reason = (
                        f"return_h_waypoint_{self._return_h_index}_reached"
                    )
                else:
                    self._transition(Task2Phase.LAND_H, data.now, "h_reached")

        elif self.phase == Task2Phase.LAND_H:
            target_xy = self._return_h_target
            target_world_height = cfg.final_height_m
            vx, vy = self._point_velocity(
                position_xy, self._return_h_target, 0.10
            )
            land_requested = z_world <= cfg.final_height_m + 0.05
            if (
                cfg.land_h_auto_descend_timeout_s > 0.0
                and data.now - self._phase_since
                >= cfg.land_h_auto_descend_timeout_s
            ):
                self._return_h_target = position_xy
                target_xy = position_xy
                vx = vy = 0.0
                land_requested = True
                self.reason = "land_h_timeout_force_descend"
            if data.landed:
                self._transition(Task2Phase.COMPLETE, data.now, "landed_at_h")

        elif self.phase == Task2Phase.COMPLETE:
            target_xy = self._return_h_target
            target_world_height = cfg.final_height_m

        return Task2Command(
            phase=self.phase,
            target_xy_m=target_xy,
            target_world_height_m=target_world_height,
            target_deck_height_m=target_deck_height,
            vx_m_s=vx,
            vy_m_s=vy,
            takeoff_requested=takeoff_requested,
            land_requested=land_requested,
            tracker_active=tracker_active,
            landing_active=landing_active,
            # 飞行阶段必须保持解锁。正式平台复飞模式的锁桨、5秒停留和
            # 第二次起飞边沿由飞行适配层在进入 RETAKEOFF 之前完成；
            # RETAKEOFF 之后若仍未解锁，适配层会拒绝继续。
            keep_armed=self.phase not in (
                Task2Phase.WAIT_START,
                Task2Phase.COMPLETE,
            ),
            fixed_heading=True,
            mission_success=self.mission_success,
            reason=self.reason,
        )

    def _confirm_vision(
        self, data: Task2Input, *, require_centered: bool
    ) -> bool:
        valid = (
            data.vision_seq is not None
            and data.vision_found
            and not data.vision_ambiguous
            and data.vision_quality >= self.config.vision_min_quality
        )
        if require_centered:
            valid = (
                valid
                and data.vision_error_xy_m is not None
                and math.hypot(*data.vision_error_xy_m)
                <= self.config.drop_max_error_m
            )
        if not valid:
            self._vision_confirm_count = 0
            self._last_vision_seq = None
            return False
        if data.vision_seq != self._last_vision_seq:
            self._last_vision_seq = data.vision_seq
            self._vision_confirm_count += 1
        return self._vision_confirm_count >= self.config.vision_confirm_frames

    def _sync_vision_command(
        self, data: Task2Input
    ) -> tuple[tuple[float, float], bool]:
        """Generate a filtered, rate-limited visual centering command.

        Detection continues at camera rate, while the velocity target updates at
        a lower fixed cadence.  A short loss grace bridges isolated missed frames;
        a longer loss smoothly brakes and holds the current T265 position.  The
        return-to-A phase uses its own faster gains so it can follow the moving
        car without making C-point landing alignment more aggressive.
        """
        cfg = self.config
        now = float(data.now)
        visual_return_a = self.phase == Task2Phase.VISUAL_RETURN_A
        vision_kp = (
            cfg.visual_return_a_kp
            if visual_return_a
            else cfg.c_sync_vision_kp
        )
        vision_max_speed = (
            cfg.visual_return_a_max_speed_m_s
            if visual_return_a
            else cfg.c_sync_vision_max_speed_m_s
        )
        vision_max_accel = (
            cfg.visual_return_a_max_accel_m_s2
            if visual_return_a
            else cfg.c_sync_vision_max_accel_m_s2
        )
        valid = (
            data.vision_seq is not None
            and data.vision_found
            and not data.vision_ambiguous
            and data.vision_quality >= cfg.vision_min_quality
            and data.vision_error_xy_m is not None
        )
        if valid:
            assert data.vision_seq is not None
            assert data.vision_error_xy_m is not None
            if data.vision_seq != self._sync_vision_last_seq:
                error_x, error_y = data.vision_error_xy_m
                self._sync_vision_errors.append(
                    (now, data.vision_seq, error_x, error_y)
                )
                self._sync_vision_last_seq = data.vision_seq
            self._sync_vision_last_valid_time = now

        cutoff = now - max(0.05, cfg.c_sync_vision_filter_window_s)
        while (
            self._sync_vision_errors
            and self._sync_vision_errors[0][0] < cutoff
        ):
            self._sync_vision_errors.popleft()

        target_recent = (
            self._sync_vision_last_valid_time is not None
            and now - self._sync_vision_last_valid_time
            <= cfg.c_sync_vision_loss_grace_s
            and bool(self._sync_vision_errors)
        )
        if (
            self._sync_vision_control_time is not None
            and now - self._sync_vision_control_time
            < cfg.c_sync_vision_control_period_s
        ):
            return self._sync_vision_xy_m_s, target_recent

        desired_x = desired_y = 0.0
        if target_recent:
            error_x = median(item[2] for item in self._sync_vision_errors)
            error_y = median(item[3] for item in self._sync_vision_errors)
            error_norm = math.hypot(error_x, error_y)
            if error_norm > cfg.c_sync_vision_deadband_m:
                active_scale = (
                    error_norm - cfg.c_sync_vision_deadband_m
                ) / error_norm
                desired_x = vision_kp * error_x * active_scale
                desired_y = vision_kp * error_y * active_scale
                desired_x, desired_y = _limit_norm(
                    desired_x,
                    desired_y,
                    vision_max_speed,
                )

        previous_x, previous_y = self._sync_vision_xy_m_s
        if self._sync_vision_control_time is None:
            dt = 0.05
        else:
            dt = min(
                max(0.0, now - self._sync_vision_control_time),
                max(0.20, cfg.c_sync_vision_control_period_s * 2.0),
            )
        max_delta = vision_max_accel * dt
        delta_x, delta_y = _limit_norm(
            desired_x - previous_x,
            desired_y - previous_y,
            max_delta,
        )
        command = (previous_x + delta_x, previous_y + delta_y)
        self._sync_vision_xy_m_s = command
        self._sync_vision_control_time = now
        return command, target_recent

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

    def _hold_velocity(
        self,
        position_xy: tuple[float, float],
        target_xy: tuple[float, float],
        velocity_xy_m_s: tuple[float, float],
        speed_limit: float,
    ) -> tuple[float, float]:
        """Position hold with optional velocity damping.

        The damping gain defaults to zero, so the formal task keeps its
        established P-only behavior.  The stationary retakeoff test enables
        damping to arrest takeoff/relaunch drift before it reaches the safety
        geofence.
        """
        dx = target_xy[0] - position_xy[0]
        dy = target_xy[1] - position_xy[1]
        kd = max(0.0, self.config.hold_velocity_kd)
        vx = self.config.point_kp * dx - kd * velocity_xy_m_s[0]
        vy = self.config.point_kp * dy - kd * velocity_xy_m_s[1]
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

    def _begin_climb(
        self,
        now: float,
        position_xy: tuple[float, float],
        reason: str,
    ) -> None:
        self._climb_anchor = position_xy
        self._transition(Task2Phase.CLIMB_150CM, now, reason)

    def _begin_visual_return_a(
        self,
        now: float,
        position_xy: tuple[float, float],
    ) -> None:
        self._sync_vision_errors.clear()
        self._sync_vision_last_seq = None
        self._sync_vision_last_valid_time = None
        self._sync_vision_control_time = None
        self._sync_vision_xy_m_s = (0.0, 0.0)
        self._sync_hold_anchor = position_xy
        self._visual_return_a_stopped_since = None
        self._transition(
            Task2Phase.VISUAL_RETURN_A,
            now,
            "retakeoff_stabilized_visual_follow_to_a",
        )

    def _retakeoff_forward_command(
        self,
        data: Task2Input,
        position_xy: tuple[float, float],
    ) -> tuple[tuple[float, float], float, float]:
        """Hold T265 stability while advancing along -Y during retakeoff.

        The moving anchor prevents the position-hold P term from cancelling the
        requested forward velocity.  This is deliberately independent of car
        telemetry and is disabled for the stationary retakeoff bench mode.
        """
        now = float(data.now)
        forward_y = (
            float(self.config.retakeoff_forward_y_speed_m_s)
            if self.config.platform_retakeoff_enabled
            and not self.config.stationary_retakeoff_test
            else 0.0
        )
        if self._retakeoff_forward_last_time is not None:
            dt = min(max(0.0, now - self._retakeoff_forward_last_time), 0.20)
            self._retakeoff_anchor = (
                self._retakeoff_anchor[0],
                self._retakeoff_anchor[1] + forward_y * dt,
            )
        self._retakeoff_forward_last_time = now
        hold_vx, hold_vy = self._hold_velocity(
            position_xy,
            self._retakeoff_anchor,
            data.velocity_xy_m_s,
            self.config.hold_position_max_speed_m_s,
        )
        vx, vy = _limit_norm(
            hold_vx,
            hold_vy + forward_y,
            self.config.hold_position_max_speed_m_s,
        )
        return self._retakeoff_anchor, vx, vy

    def _visual_return_a_centered(self, target_recent: bool) -> bool:
        if not target_recent or not self._sync_vision_errors:
            return False
        error_x = median(item[2] for item in self._sync_vision_errors)
        error_y = median(item[3] for item in self._sync_vision_errors)
        return math.hypot(error_x, error_y) <= (
            self.config.visual_return_a_center_radius_m
        )

    def _begin_return_h(
        self,
        now: float,
        position_xy: tuple[float, float],
        reason: str,
    ) -> None:
        """Build the path to the active H target.

        Normal abort/legacy returns keep the original global ``H``.  After a
        platform retakeoff, ``_return_h_target`` is expressed relative to the
        visually confirmed A hover point and is flown as one direct leg, so
        T265's possibly drifted global origin is no longer used as the
        physical H reference.
        """
        start_x, start_y = float(position_xy[0]), float(position_xy[1])
        local_retakeoff_return = self._retakeoff_local_frame_pending
        if local_retakeoff_return:
            self._return_h_target = (
                start_x + self.config.retakeoff_h_offset_x_m,
                start_y + self.config.retakeoff_h_offset_y_m,
            )
            self._retakeoff_local_frame_pending = False
        target_x, target_y = self._return_h_target
        if local_retakeoff_return:
            self._return_h_waypoints = (self._return_h_target,)
        else:
            self._return_h_waypoints = tuple(
                (
                    target_x + (start_x - target_x) * fraction,
                    target_y + (start_y - target_y) * fraction,
                )
                for fraction in RETURN_H_FRACTIONS
            ) + (self._return_h_target,)
        self._return_h_index = 0
        self._transition(Task2Phase.RETURN_H, now, reason)

    def _transition(
        self, phase: Task2Phase, now: float, reason: str
    ) -> None:
        if phase == Task2Phase.RETAKEOFF:
            self._retakeoff_forward_last_time = float(now)
        if phase == Task2Phase.TRANSIT_C:
            self._transit_c_index = 0
        if phase == Task2Phase.SYNC_TARGET_AT_C:
            self._sync_vision_errors.clear()
            self._sync_vision_last_seq = None
            self._sync_vision_last_valid_time = None
            self._sync_vision_control_time = None
            self._sync_vision_xy_m_s = (0.0, 0.0)
            self._sync_vision_active = False
            self._sync_hold_anchor = self.sync_c
        self.phase = phase
        self._phase_since = float(now)
        self.reason = reason
        self._stable_since = None
        self._vision_confirm_count = 0
        self._last_vision_seq = None


def _limit_norm(x: float, y: float, limit: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= limit or norm <= 1e-9:
        return x, y
    scale = limit / norm
    return x * scale, y * scale
