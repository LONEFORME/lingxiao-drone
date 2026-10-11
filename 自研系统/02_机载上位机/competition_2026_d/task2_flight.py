"""任务二核心与 basic 飞控安全状态机的飞行适配类。

C 点之前复用任务一的路径跟随+视觉对中+WorldDeckHeightController；
C 点之后切换到 PlatformTracker（T265 坐标系伴飞估计）+
FormationController（水平速度）+ DynamicLandingController（垂直速度）。
"""

from __future__ import annotations

import json
import math
import sys
import time
from collections import deque
from dataclasses import replace
from pathlib import Path
from typing import Callable

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

from Lcode.Logger import logger  # noqa: E402
from Lcode.global_variable import lock  # noqa: E402
from Mission_GPT import DRY_RUN, laser_height_valid, mission  # noqa: E402

from .control.formation_controller import (  # noqa: E402
    FormationConfig,
    FormationController,
)
from .dynamic_landing import (  # noqa: E402
    DynamicLandingController,
    LandingConfig,
    LandingInput,
    LandingState,
)
from .payload_actuator import PayloadActuator  # noqa: E402
from .task1_runtime import (  # noqa: E402
    HeightReference,
    Task1T265SafetyMonitor,
    WorldDeckHeightController,
    observation_to_gate_sample,
)
from .task2_mission import (  # noqa: E402
    B_PRE,
    H,
    Task2Config,
    Task2Input,
    Task2MissionDirector,
    Task2Phase,
)
from .task2_runtime import (  # noqa: E402
    LaserContactConfig,
    LaserContactDetector,
    OffsetCalibrator,
)
from .vision.cybercam_reader import CyberCamReader  # noqa: E402
from .vision.platform_observation import FeatureFlag  # noqa: E402
from .vision.platform_tracker import (  # noqa: E402
    PlatformEstimate,
    PlatformTracker,
    TrackerConfig,
)


FC_DIRECT_LOCK_SIGN = 102
FC_DIRECT_LOCK_CONFIRM_COUNT = 5
FC_DIRECT_LOCK_WARN_TIMEOUT_S = 3.0
FC_TASK_RESET_CONFIRM_COUNT = 3
T265_RETAKEOFF_JUMP_TRIGGER_M = 0.30
T265_RETAKEOFF_RECOVERY_INNOVATION_M = 0.05
T265_RETAKEOFF_RECOVERY_WINDOW_S = 0.80
T265_RETAKEOFF_MAX_NET_CORRECTION_M = 2.00


class Task2FlightMission(mission):
    """飞控循环内唯一写入 XY 速度的任务二适配器。"""

    def __init__(
        self,
        re_fc,
        se_fc,
        realsense_obj,
        serial_fc_ref,
        *,
        vision_reader: CyberCamReader,
        payload_actuator: PayloadActuator,
        task_config: Task2Config | None = None,
        tracker_config: TrackerConfig | None = None,
        formation_config: FormationConfig | None = None,
        landing_config: LandingConfig | None = None,
        vision_config: dict | None = None,
        car_speed_provider: Callable[[], float | None] | None = None,
        car_position_provider: Callable[[], tuple[float, float] | None] | None = None,
        car_velocity_provider: Callable[[], tuple[float, float] | None] | None = None,
        height_source: str = "t265",
        preflight_warning_completed: bool = False,
        preflight_warning_started_at: float | None = None,
    ) -> None:
        super().__init__(
            re_fc,
            se_fc,
            realsense_obj,
            serial_fc_ref,
            interactive_preflight=False,
            preflight_warning_completed=preflight_warning_completed,
            preflight_warning_started_at=preflight_warning_started_at,
        )
        self.director = Task2MissionDirector(task_config)
        self.vision_reader = vision_reader
        self.payload_actuator = payload_actuator
        self.height_controller = WorldDeckHeightController()
        self.t265_safety = Task1T265SafetyMonitor()
        effective_landing_config = landing_config or LandingConfig()
        if self.director.config.direct_descent_after_trigger:
            effective_landing_config = replace(
                effective_landing_config,
                direct_descent_after_gate=True,
            )
        if tracker_config is None:
            tracker_config = TrackerConfig(
                max_predict_s=max(
                    0.15, effective_landing_config.terminal_max_s
                )
            )
        self.platform_tracker = PlatformTracker(tracker_config)
        self.formation_controller = FormationController(formation_config)
        self.dynamic_landing = DynamicLandingController(
            effective_landing_config
        )
        self.offset_calibrator = OffsetCalibrator()
        self.laser_contact = LaserContactDetector(
            LaserContactConfig(
                contact_height_m=self.dynamic_landing.config.contact_height_m
            )
        )
        self.car_speed_provider = car_speed_provider
        self.car_position_provider = car_position_provider
        self.car_velocity_provider = car_velocity_provider
        if height_source not in ("t265", "laser"):
            raise ValueError("height_source must be 't265' or 'laser'")
        self.height_source = height_source
        self.vision_config = {
            "max_age_s": 0.15,
            "min_quality": self.director.config.vision_min_quality,
            "image_center_px": (320.0, 240.0),
            "focal_px": (570.0, 570.0),
            "target_offset_xy_m": (
                self.director.config.vision_target_offset_x_m,
                self.director.config.vision_target_offset_y_m,
            ),
            **(vision_config or {}),
        }
        self._last_task2_phase = self.director.phase
        self._last_runtime_log = 0.0
        self._last_nav_time: float | None = None
        self._tracker_active_prev = False
        # 供 telemetry 提取
        self.last_landing_state = None
        self.last_world_position = (0.0, 0.0, 0.0)
        # 上一帧的降落反馈（给下一帧的 director 用）
        self._landing_gate_passed = False
        self._touchdown_confirmed = False
        self._deck_ride_complete = False
        self._landing_aborted = False
        self._dry_run = DRY_RUN
        self._stationary_platform_measurement: tuple[float, float] | None = None
        self._last_platform_vision_seq: int | None = None
        self._platform_measurement_source = "none"
        self._landing_vz_applied_m_s = 0.0
        self._last_platform_estimate: PlatformEstimate | None = None
        self._direct_lock_active = False
        self._direct_lock_started_at: float | None = None
        self._direct_lock_confirm_count = 0
        self._direct_lock_timeout_logged = False
        self._direct_lock_confirmed_at: float | None = None
        self._direct_lock_reset_started_at: float | None = None
        self._direct_lock_reset_confirm_count = 0
        self._direct_lock_retakeoff_blocked = False
        self._direct_lock_retakeoff_block_logged = False
        self._direct_lock_laser_samples: deque[tuple[float, float]] = deque()
        self._direct_lock_zero_setpoint_since: float | None = None
        self._t265_continuity_last_time: float | None = None
        self._t265_continuity_last_position: tuple[float, float, float] | None = None
        self._t265_recovery_until: float | None = None
        self._t265_recovery_used = False
        self._t265_continuity_net_correction = (0.0, 0.0, 0.0)
        self._task2_hover_wait_started_at: float | None = None
        self._final_realtime_landing_active = False

    def navigate(self, _laser_position, yaw):
        now = time.monotonic()
        try:
            world_position = tuple(
                float(v) for v in self.realsense.get_position()
            )
            velocity = tuple(
                float(v) for v in self.realsense.get_velocity()
            )
            confidence = int(self.realsense.get_tracking_confidence())
        except Exception as exc:
            logger.error(f"任务二读取T265失败，本周期悬停: {exc}")
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            return

        laser_height = self._laser_height()
        if confidence == 0:
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            logger.warning("任务二T265置信度为0，冻结状态机并原地悬停")
            return
        if (
            (
                self.director.config.safe_open_loop_cd_test
                or self.director.config.stationary_retakeoff_test
            )
            and confidence < self.director.config.t265_min_confidence
        ):
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            logger.error(
                "任务二安全联调T265置信度不足；停止运动并转HOVER_WAIT"
            )
            self.state = "HOVER_WAIT"
            return

        world_position = self._preserve_retakeoff_t265_continuity(
            now=now,
            world_position=world_position,
            velocity=velocity,
        )

        phase_before_tick = self.director.phase
        hold_anchor = self._hold_anchor_for_phase(phase_before_tick)
        ground_reference_expected = phase_before_tick in (
            Task2Phase.WAIT_START,
            Task2Phase.TAKEOFF,
            Task2Phase.HOLD_3S,
            Task2Phase.INTERCEPT_B_PRE,
            Task2Phase.ACQUIRE_TARGET,
            Task2Phase.TRANSIT_C,
            Task2Phase.SYNC_TARGET_AT_C,
            Task2Phase.OPEN_LOOP_C_D,
            Task2Phase.SAFE_HOVER_D,
            Task2Phase.ACTIVATE_TRACKER,
            Task2Phase.LAND_H,
        )
        safety_fault = self.t265_safety.update(
            timestamp=now,
            world_position_xyz_m=world_position,
            laser_height_m=laser_height,
            ground_reference_expected=ground_reference_expected,
            hold_anchor_xy_m=hold_anchor,
        )
        if safety_fault is not None:
            if laser_height is not None:
                self._ramp_z_cm = laser_height * 100.0
            self.set_speed(0, 0, 0, int(round(self._ramp_z_cm)))
            logger.error(
                f"任务二T265安全保护触发: {safety_fault}；转HOVER_WAIT"
            )
            self.state = "HOVER_WAIT"
            return

        observation = self.vision_reader.latest(
            now, float(self.vision_config["max_age_s"])
        )
        relative_height = (
            laser_height
            if laser_height is not None
            else max(0.01, world_position[2])
        )
        gate = observation_to_gate_sample(
            observation,
            now=now,
            relative_height_m=relative_height,
            max_age_s=float(self.vision_config["max_age_s"]),
            min_quality=int(self.vision_config["min_quality"]),
            image_center_px=tuple(self.vision_config["image_center_px"]),
            focal_px=tuple(self.vision_config["focal_px"]),
            target_offset_xy_m=tuple(
                self.vision_config["target_offset_xy_m"]
            ),
        )

        car_speed = (
            None if self.car_speed_provider is None else self.car_speed_provider()
        )
        car_position = (
            None
            if self.car_position_provider is None
            else self.car_position_provider()
        )
        car_velocity = (
            None
            if self.car_velocity_provider is None
            else self.car_velocity_provider()
        )

        # offset 标定：C 点前持续收集样本
        offset_ready = self.offset_calibrator.ready
        offset = self.offset_calibrator.offset() if offset_ready else (0.0, 0.0)
        if phase_before_tick in (
            Task2Phase.TAKEOFF,
            Task2Phase.HOLD_3S,
            Task2Phase.INTERCEPT_B_PRE,
            Task2Phase.ACQUIRE_TARGET,
            Task2Phase.FOLLOW_B_C,
        ):
            if car_position is not None and confidence >= 2:
                self.offset_calibrator.record_sample(
                    car_position,
                    (world_position[0], world_position[1]),
                )
                offset_ready = self.offset_calibrator.ready
                offset = (
                    self.offset_calibrator.offset() if offset_ready else (0.0, 0.0)
                )

        contact_evidence = self.laser_contact.update(laser_height)

        self.last_world_position = world_position

        control_height = (
            laser_height
            if self.height_source == "laser" and laser_height is not None
            else world_position[2]
        )
        command = self.director.tick(
            Task2Input(
                now=now,
                position_xyz_m=(
                    world_position[0],
                    world_position[1],
                    control_height,
                ),
                velocity_xy_m_s=(velocity[0], velocity[1]),
                t265_confidence=confidence,
                car_start=True,
                car_speed_m_s=car_speed,
                vision_seq=gate.seq,
                vision_found=gate.found,
                vision_quality=gate.quality,
                vision_ambiguous=gate.ambiguous,
                vision_error_xy_m=gate.error_xy_m,
                deck_relative_height_m=laser_height,
                car_position_xy_m=car_position,
                car_velocity_xy_m_s=car_velocity,
                offset_ready=offset_ready,
                landing_gate_passed=self._landing_gate_passed,
                touchdown_confirmed=self._touchdown_confirmed,
                deck_ride_complete=self._deck_ride_complete,
                landing_aborted=self._landing_aborted,
            )
        )

        if not self._verify_continuous_arm_contract(command):
            return

        vx = command.vx_m_s
        vy = command.vy_m_s
        horizontal_control_source = "task2_director"
        if (
            command.phase == Task2Phase.SYNC_TARGET_AT_C
            and command.reason == "c_sync_visual_centering"
        ):
            horizontal_control_source = "c_sync_visual_centering"
        landing_vz = 0.0
        estimate: PlatformEstimate | None = None

        if command.tracker_active:
            if not self._tracker_active_prev:
                self.platform_tracker.reset()
                self._stationary_platform_measurement = None
                self._last_platform_vision_seq = None
                self._platform_measurement_source = "none"
                self._landing_vz_applied_m_s = 0.0
                self._last_platform_estimate = None
                self.formation_controller.reset(
                    timestamp=now,
                    velocity=(velocity[0], velocity[1]),
                )
                self.dynamic_landing.reset()
                self._landing_gate_passed = False
                self._touchdown_confirmed = False
                self._deck_ride_complete = False
                self._landing_aborted = False
                logger.info(
                    "任务二激活 PlatformTracker + FormationController + "
                    "DynamicLandingController"
                )

            landing_car_velocity = car_velocity
            if self.director.config.stationary_retakeoff_test:
                estimate = self._update_stationary_platform_estimate(
                    gate=gate,
                    world_position=world_position,
                    now=now,
                )
                landing_car_velocity = (0.0, 0.0)
                self._platform_measurement_source = "stationary_visual"
            # 正式任务优先使用视觉相对误差；同一视觉帧只更新一次滤波器。
            elif gate.found and gate.error_xy_m is not None:
                estimate = self._update_visual_platform_estimate(
                    gate=gate,
                    world_position=world_position,
                    now=now,
                )
                if estimate is not None:
                    landing_car_velocity = (
                        estimate.vx_m_s,
                        estimate.vy_m_s,
                    )
            # 视觉暂时不可用时，小车坐标 + offset 作为后备测量。
            elif car_position is not None and offset_ready:
                car_in_uav = (
                    car_position[0] - offset[0],
                    car_position[1] - offset[1],
                )
                estimate = self.platform_tracker.update(
                    car_in_uav[0], car_in_uav[1], now
                )
                self._platform_measurement_source = "car_t265"
            else:
                estimate = self.platform_tracker.predict(now)
                self._platform_measurement_source = (
                    "visual_prediction" if estimate is not None else "lost"
                )
                if estimate is not None:
                    landing_car_velocity = (
                        estimate.vx_m_s,
                        estimate.vy_m_s,
                    )

            if estimate is not None:
                self._last_platform_estimate = estimate
            landing_estimate = estimate
            if (
                landing_estimate is None
                and self.director.config.direct_descent_after_trigger
                and self._landing_gate_passed
            ):
                # 触发后的垂直下降不依赖持续视觉。保留最后一次平台估计只用于
                # 驱动降落状态机；FormationController 会自行拒绝过期估计，
                # 因此视觉长期丢失时水平命令自然回到零而不会追逐陈旧目标。
                landing_estimate = self._last_platform_estimate

            if command.landing_active and landing_estimate is not None:
                landing_cmd = self._run_landing(
                    landing_estimate,
                    world_position,
                    velocity,
                    laser_height,
                    gate,
                    observation,
                    contact_evidence,
                    safety_fault,
                    landing_car_velocity,
                )
                if landing_cmd is not None:
                    self._landing_gate_passed = (
                        landing_cmd.state != LandingState.LANDING_GATE
                    )
                    self._touchdown_confirmed = landing_cmd.touchdown_confirmed
                    self._deck_ride_complete = (
                        landing_cmd.state == LandingState.RETAKEOFF_GATE
                        and not self.director.config.fc_direct_lock_enabled
                    )
                    self._landing_aborted = (
                        landing_cmd.state == LandingState.CONTROLLED_ABORT
                    )
                    if command.phase == Task2Phase.DYNAMIC_LANDING:
                        landing_vz = landing_cmd.vertical_speed_m_s
                        formation_cmd = self.formation_controller.command(
                            landing_estimate,
                            (world_position[0], world_position[1]),
                            (velocity[0], velocity[1]),
                            now,
                        )
                        if formation_cmd.valid:
                            vx = formation_cmd.vx_m_s
                            vy = formation_cmd.vy_m_s
                            vx, vy = self._limit_xy_speed(
                                vx,
                                vy,
                                self._landing_xy_speed_limit(relative_height),
                            )
                            horizontal_control_source = "visual_formation"
                        elif self.director.config.direct_descent_after_trigger:
                            # 视觉预测也失效后不瞬间把水平速度砍到零；复用
                            # FormationController 的限加速度/限jerk刹车输出，
                            # 在持续开环下降的同时平滑收住水平运动。
                            vx = formation_cmd.vx_m_s
                            vy = formation_cmd.vy_m_s
                            vx, vy = self._limit_xy_speed(
                                vx,
                                vy,
                                self._landing_xy_speed_limit(relative_height),
                            )
                            horizontal_control_source = "open_loop_smooth_brake"
        else:
            if self._tracker_active_prev:
                self._landing_gate_passed = False
                self._touchdown_confirmed = False
                self._deck_ride_complete = False
                self._landing_aborted = False

        self._tracker_active_prev = command.tracker_active

        self.last_landing_state = (
            self.dynamic_landing.state
            if command.landing_active
            else None
        )

        if self._should_begin_fc_direct_lock(
            command, laser_height, contact_evidence, now
        ):
            self._begin_fc_direct_lock(laser_height)
            return

        if self._should_handoff_to_fc_one_key_land(command, laser_height):
            self._begin_fc_one_key_land(laser_height)
            return

        # 高度控制
        if (
            command.phase == Task2Phase.DYNAMIC_LANDING
            and command.landing_active
        ):
            direct_ground_contact = (
                self.director.config.direct_descent_after_trigger
                and self.dynamic_landing.state
                in (
                    LandingState.TOUCHDOWN_CANDIDATE,
                    LandingState.DECK_RIDE,
                    LandingState.RETAKEOFF_GATE,
                )
            )
            if direct_ground_contact:
                # A 5 cm setpoint can create a real hover equilibrium.  Keep
                # commanding zero until the platform physically supports the
                # aircraft; only then may the stable-laser lock gate fire.
                self._landing_vz_applied_m_s = 0.0
                self._ramp_z_cm = 0.0
                if self._direct_lock_zero_setpoint_since is None:
                    self._direct_lock_zero_setpoint_since = now
                height = HeightReference(
                    0.0, True, "direct_touchdown_press", "ok"
                )
            else:
                self._direct_lock_zero_setpoint_since = None
                if self._last_nav_time is not None:
                    dt = min(max(now - self._last_nav_time, 0.001), 0.05)
                else:
                    dt = 0.02
                self._landing_vz_applied_m_s = self._smooth_landing_vz(
                    landing_vz, dt
                )
                self._step_landing_height(self._landing_vz_applied_m_s, dt)
                height = HeightReference(
                    self._ramp_z_cm / 100.0, True, "landing_vz", "ok"
                )
        elif command.phase == Task2Phase.LANDED_ON_PLATFORM:
            self._landing_vz_applied_m_s = 0.0
            # Keep the FC armed but command a height below the physical laser
            # rest height so collective thrust remains near its ground minimum.
            self._ramp_z_cm = 5.0
            height = HeightReference(0.05, True, "platform_ground_rest", "ok")
        elif self.height_source == "laser":
            self._landing_vz_applied_m_s = 0.0
            target_laser_height = (
                command.target_world_height_m
                if command.target_deck_height_m is None
                else command.target_deck_height_m
            )
            if laser_height is None:
                height = HeightReference(
                    self._ramp_z_cm / 100.0,
                    False,
                    "laser_height",
                    "laser_height_unavailable",
                )
            else:
                self._step_ramp_z(target_laser_height * 100.0)
                height = HeightReference(
                    self._ramp_z_cm / 100.0, True, "laser_height", "ok"
                )
        else:
            self._landing_vz_applied_m_s = 0.0
            height = self.height_controller.command(
                timestamp=now,
                current_world_height_m=world_position[2],
                current_laser_height_m=laser_height,
                target_world_height_m=command.target_world_height_m,
                target_deck_height_m=command.target_deck_height_m,
            )
            if height.valid:
                self._ramp_z_cm = height.laser_setpoint_m * 100.0

        self._last_nav_time = now

        self._heading_status = self._update_heading_hold(yaw, confidence)
        yaw_cmd = self._heading_status.command_dps
        vx_cms = int(round(vx * 100.0))
        vy_cms = int(round(vy * 100.0))
        self.set_speed(vx_cms, vy_cms, yaw_cmd, int(round(self._ramp_z_cm)))

        if command.phase != self._last_task2_phase:
            if command.phase == Task2Phase.RETAKEOFF:
                logger.info(
                    "任务二第二次起飞成功：保持复飞锚点并继续爬升至"
                    f"{self.director.config.retakeoff_height_m:.2f}m"
                )
            elif command.phase == Task2Phase.STABILIZE_AFTER_RETAKEOFF:
                logger.info(
                    "任务二达到复飞高度：原地稳定"
                    f"{self.director.config.retakeoff_stabilize_s:.1f}秒后返航"
                )
            elif command.phase == Task2Phase.RETURN_H:
                target = self.director._return_h_waypoints[0]
                logger.warning(
                    "任务二复飞局部坐标系已建立：当前位置=(0,0)，"
                    f"局部H=({self.director.config.retakeoff_h_offset_x_m:+.2f},"
                    f"{self.director.config.retakeoff_h_offset_y_m:+.2f})m；"
                    "A点上空直接飞往T265世界系H目标="
                    f"({target[0]:+.2f},{target[1]:+.2f})"
                )
            elif command.phase == Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF:
                logger.warning(
                    "静止平台复升联调通过：已到达复升高度并保持T265定点；"
                    "请用遥控器安全接管、落地并确认电机停稳后再终止程序"
                )
            elif command.phase == Task2Phase.LANDED_ON_PLATFORM:
                logger.warning(
                    "任务二静止平台降落完成：不执行复升，保持5cm低高度目标；"
                    "确认飞行器稳定后使用遥控器停止电机，再终止程序"
                )
            logger.info(
                f"任务二状态: {self._last_task2_phase.value} -> "
                f"{command.phase.value} ({command.reason})"
            )
            self._last_task2_phase = command.phase

        self._log_runtime(
            now,
            command,
            world_position,
            velocity,
            confidence,
            laser_height,
            height,
            gate,
            car_position,
            car_velocity,
            offset_ready,
            offset,
            estimate,
            landing_vz,
            (vx, vy),
            horizontal_control_source,
        )

        if (
            command.phase == Task2Phase.LAND_H
            and command.land_requested
            and math.dist(
                (world_position[0], world_position[1]),
                self.director._return_h_target,
            )
            <= self.director.config.point_arrival_radius_m
        ):
            logger.info(
                "任务二到达最终降落点，转入实时帧两级下降；"
                "近地后使用102直接锁桨，不调用一键降落"
            )
            self._final_realtime_landing_active = True
            self.state = "DESCEND"

    def _preserve_retakeoff_t265_continuity(
        self,
        *,
        now: float,
        world_position: tuple[float, float, float],
        velocity: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Absorb T265 relocalization during platform retakeoff and return.

        Landing vibration can make T265 relocalize while still reporting high
        confidence.  A 30+ cm step in one control period is physically
        impossible here, so shift the software calibration offset to keep the
        local return frame continuous.  The guarded window covers retakeoff,
        post-retakeoff stabilization and RETURN_H.  With the jump hard-stop
        enabled, only one bounded recovery episode is accepted.  With it
        disabled for task two, every physically impossible step is converted
        into a software-frame correction so it cannot force HOVER_WAIT or feed
        a multi-metre discontinuity into flight control.
        """
        position = tuple(float(v) for v in world_position)
        last_time = self._t265_continuity_last_time
        last_position = self._t265_continuity_last_position
        self._t265_continuity_last_time = float(now)

        if last_time is None or last_position is None:
            self._t265_continuity_last_position = position
            return position

        dt = float(now) - last_time
        if not (0.0 < dt <= 0.30):
            self._t265_continuity_last_position = position
            return position

        predicted = tuple(
            last_position[index] + float(velocity[index]) * dt
            for index in range(3)
        )
        innovation = tuple(
            position[index] - predicted[index] for index in range(3)
        )
        step = math.dist(position, last_position)
        innovation_norm = math.sqrt(sum(value * value for value in innovation))
        active_phase = self.director.phase in (
            Task2Phase.RETAKEOFF,
            Task2Phase.STABILIZE_AFTER_RETAKEOFF,
            Task2Phase.VISUAL_RETURN_A,
            Task2Phase.RETURN_H,
        )
        recovering = (
            self._t265_recovery_until is not None
            and now <= self._t265_recovery_until
        )
        jump_hard_stop = (
            self.director.config.retakeoff_t265_jump_protection_enabled
        )
        should_correct = active_phase and (
            (
                not jump_hard_stop
                and step > T265_RETAKEOFF_JUMP_TRIGGER_M
            )
            or (
                not self._t265_recovery_used
                and step > T265_RETAKEOFF_JUMP_TRIGGER_M
            )
            or (
                recovering
                and innovation_norm > T265_RETAKEOFF_RECOVERY_INNOVATION_M
            )
        )
        if not should_correct:
            self._t265_continuity_last_position = position
            return position

        if not self._t265_recovery_used:
            self._t265_recovery_used = True
            self._t265_recovery_until = now + T265_RETAKEOFF_RECOVERY_WINDOW_S

        proposed_net = tuple(
            self._t265_continuity_net_correction[index] + innovation[index]
            for index in range(3)
        )
        if jump_hard_stop and math.sqrt(
            sum(value * value for value in proposed_net)
        ) > T265_RETAKEOFF_MAX_NET_CORRECTION_M:
            # Leave this sample unmodified so Task1T265SafetyMonitor fails
            # closed instead of hiding an unbounded coordinate failure.
            self._t265_recovery_until = None
            self._t265_continuity_last_position = position
            return position

        apply_correction = getattr(
            self.realsense, "apply_position_continuity_correction", None
        )
        if not callable(apply_correction):
            self._t265_recovery_until = None
            self._t265_continuity_last_position = position
            return position

        apply_correction(innovation)
        self._t265_continuity_net_correction = proposed_net
        self._t265_continuity_last_position = predicted
        logger.warning(
            "任务二复飞T265坐标连续性补偿："
            f"step={step:.3f}m/{dt:.3f}s, "
            f"delta=({innovation[0]:+.3f},{innovation[1]:+.3f},"
            f"{innovation[2]:+.3f})m；保留局部返航坐标连续性，"
            f"jump_hard_stop={'on' if jump_hard_stop else 'off'}"
        )
        return predicted

    def _step_landing_height(self, vertical_speed_m_s: float, dt: float) -> None:
        """Integrate signed vertical speed into the laser height setpoint."""
        # Positive means climb and negative means descend in
        # DynamicLandingController, while the laser setpoint is height above
        # the surface.  Therefore the signed velocity must be added.
        self._ramp_z_cm += float(vertical_speed_m_s) * 100.0 * float(dt)
        self._ramp_z_cm = max(0.0, min(self._ramp_z_cm, 200.0))

    def _should_begin_fc_direct_lock(
        self,
        command,
        laser_height: float | None,
        contact_evidence: bool,
        now: float,
    ) -> bool:
        """Lock only after a zero-height command proves physical support."""
        cfg = self.director.config
        if (
            not cfg.fc_direct_lock_enabled
            or command.phase != Task2Phase.DYNAMIC_LANDING
            or not command.landing_active
        ):
            self._direct_lock_laser_samples.clear()
            self._direct_lock_zero_setpoint_since = None
            return False
        if laser_height is None or not math.isfinite(float(laser_height)):
            samples = self._direct_lock_laser_samples
            if (
                not samples
                or now - samples[-1][0]
                > cfg.fc_direct_lock_laser_dropout_grace_s
            ):
                samples.clear()
            return False
        relative_height = float(laser_height)
        zero_target_active = (
            self._direct_lock_zero_setpoint_since is not None
            and self._ramp_z_cm <= 0.5
        )
        if not zero_target_active:
            self._direct_lock_laser_samples.clear()
            return False
        stable_below_ten_cm = self._laser_stable_for_direct_lock(
            now=now,
            laser_height_m=relative_height,
        )
        zero_setpoint_sustained = (
            now - self._direct_lock_zero_setpoint_since
            >= cfg.fc_direct_lock_stable_hold_s
        )
        return contact_evidence and zero_setpoint_sustained and stable_below_ten_cm

    def _laser_stable_for_direct_lock(
        self,
        *,
        now: float,
        laser_height_m: float,
    ) -> bool:
        """Confirm that a sub-10cm laser reading has stopped changing."""
        cfg = self.director.config
        height = float(laser_height_m)
        if not (0.02 <= height < cfg.fc_direct_lock_stable_height_m):
            self._direct_lock_laser_samples.clear()
            return False

        samples = self._direct_lock_laser_samples
        samples.append((float(now), height))
        cutoff = float(now) - cfg.fc_direct_lock_stable_hold_s
        # Keep one sample immediately before the window boundary so duration
        # can be proven without depending on exact camera/control tick timing.
        while len(samples) >= 2 and samples[1][0] <= cutoff:
            samples.popleft()
        if (
            len(samples) < cfg.fc_direct_lock_stable_min_samples
            or float(now) - samples[0][0]
            < cfg.fc_direct_lock_stable_hold_s
        ):
            return False
        heights = [sample[1] for sample in samples]
        return max(heights) - min(heights) <= (
            cfg.fc_direct_lock_stable_tolerance_m
        )

    def _begin_fc_direct_lock(self, laser_height: float | None) -> None:
        """Keep task_sta asserted and request the firmware's guarded direct lock."""
        relative_height = (
            f"{laser_height:.2f}m" if laser_height is not None else "setpoint"
        )
        logger.warning(
            "任务二实时帧降落进入近地直接锁桨："
            f"relative_height={relative_height}, task_sta保持1, "
            f"next_task_sign={FC_DIRECT_LOCK_SIGN}；不调用OneKey_Land"
        )
        self._direct_lock_active = True
        self._direct_lock_started_at = time.monotonic()
        self._direct_lock_confirm_count = 0
        self._direct_lock_timeout_logged = False
        self._direct_lock_confirmed_at = None
        self._direct_lock_reset_started_at = None
        self._direct_lock_reset_confirm_count = 0
        self._direct_lock_retakeoff_blocked = False
        self._direct_lock_retakeoff_block_logged = False
        self._direct_lock_laser_samples.clear()
        self._direct_lock_zero_setpoint_since = None
        self.heading_hold.disarm("task2_fc_direct_lock")
        self._landing_vz_applied_m_s = 0.0
        self._ramp_z_cm = 0.0
        self.set_speed(0, 0, 0, 0)
        with lock:
            # Do not clear task_sta: its 1->0 edge is the legacy OneKey_Land path.
            self.se_fc[7] = FC_DIRECT_LOCK_SIGN
        self.state = "LAND"

    def land(self):
        """Confirm direct lock, hold five seconds, then safely re-arm."""
        if not self._direct_lock_active:
            return super().land()

        now = time.monotonic()
        resetting_task = self._direct_lock_reset_started_at is not None
        self.set_speed(0, 0, 0, 0)
        with lock:
            if resetting_task:
                # 102锁桨确认后才允许制造新的0->1任务边沿。固件在该专用
                # 情况下只复位任务状态，不会调用OneKey_Land。
                self.se_fc[2] = 0
                self.se_fc[7] = 0
            else:
                self.se_fc[7] = FC_DIRECT_LOCK_SIGN
            unlock_sta = int(self.re_fc[5]) if len(self.re_fc) > 5 else 1
            mission_stage = int(self.re_fc[0]) if self.re_fc else -1

        motor_pwm_mask = self._motor_pwm_mask()
        # 缺少PWM反馈不能当作“电机已停”；必须同时收到锁定状态和
        # 四路PWM为零的明确遥测，才允许开始平台停留计时。
        motor_stopped = (
            motor_pwm_mask is not None and int(motor_pwm_mask) == 0
        )

        if resetting_task:
            self._tick_platform_task_reset(
                now=now,
                unlock_sta=unlock_sta,
                motor_stopped=motor_stopped,
                mission_stage=mission_stage,
            )
            return

        if unlock_sta == 0 and motor_stopped:
            self._direct_lock_confirm_count += 1
        else:
            self._direct_lock_confirm_count = 0

        cfg = self.director.config
        if (
            self._direct_lock_confirm_count >= FC_DIRECT_LOCK_CONFIRM_COUNT
            and self._direct_lock_confirmed_at is None
        ):
            logger.info(
                "任务二近地直接锁桨确认成功：unlock_sta=0, "
                f"motor_pwm_mask={motor_pwm_mask}"
            )
            self.director.mission_success = True
            if (
                self._final_realtime_landing_active
                or not cfg.platform_retakeoff_enabled
            ):
                self.stop_all()
                return
            self._direct_lock_confirmed_at = now
            logger.warning(
                "任务二平台锁桨停留开始：保持锁桨"
                f"{cfg.platform_locked_hold_s:.1f}秒，T265持续运行且不重新校零"
            )

        if self._direct_lock_confirmed_at is not None:
            if unlock_sta != 0 or not motor_stopped:
                self._block_platform_retakeoff(
                    "平台停留期间锁桨或PWM零值反馈丢失"
                )
                return
            if self._direct_lock_retakeoff_blocked:
                return
            hold_elapsed = now - self._direct_lock_confirmed_at
            if hold_elapsed >= cfg.platform_locked_hold_s:
                if not self._t265_ready_for_platform_retakeoff():
                    self._block_platform_retakeoff(
                        "平台复飞前T265未运行、数据过期或置信度不足"
                    )
                    return
                self._direct_lock_reset_started_at = now
                self._direct_lock_reset_confirm_count = 0
                with lock:
                    self.se_fc[2] = 0
                    self.se_fc[7] = 0
                logger.warning(
                    "任务二平台停留5秒完成：拉低task_sta并等待飞控"
                    "mission_stage=0，随后产生第二次起飞边沿"
                )
                return

        elapsed = now - (self._direct_lock_started_at or now)
        if (
            self._direct_lock_confirmed_at is None
            and elapsed >= FC_DIRECT_LOCK_WARN_TIMEOUT_S
            and not self._direct_lock_timeout_logged
        ):
            logger.error(
                "任务二近地直接锁桨3秒内未确认；继续保持0cm实时高度目标并重试102，"
                "不会回退到OneKey_Land，请准备遥控器接管"
            )
            self._direct_lock_timeout_logged = True

    def _motor_pwm_mask(self) -> int | None:
        if self.serial_fc_ref is None:
            return None
        with lock:
            value = self.serial_fc_ref.debug_data.get("motor_pwm_mask")
        return None if value is None else int(value)

    def _t265_ready_for_platform_retakeoff(self) -> bool:
        if self.realsense is None or not self.realsense.is_running():
            return False
        try:
            if (
                int(self.realsense.get_tracking_confidence())
                < self.director.config.t265_min_confidence
            ):
                return False
            get_age = getattr(self.realsense, "get_pose_age_s", None)
            return get_age is None or float(get_age()) <= 0.20
        except Exception:
            return False

    def _block_platform_retakeoff(self, reason: str) -> None:
        self._direct_lock_retakeoff_blocked = True
        if not self._direct_lock_retakeoff_block_logged:
            logger.error(
                f"任务二平台复飞已禁止：{reason}；保持锁桨等待人工处理"
            )
            self._direct_lock_retakeoff_block_logged = True

    def _tick_platform_task_reset(
        self,
        *,
        now: float,
        unlock_sta: int,
        motor_stopped: bool,
        mission_stage: int,
    ) -> None:
        """Hold task_sta low until firmware confirms a clean reset."""
        if self._direct_lock_retakeoff_blocked:
            return
        if unlock_sta != 0 or not motor_stopped:
            self._block_platform_retakeoff(
                "任务位复位期间锁桨或PWM零值反馈异常"
            )
            return
        if mission_stage == 0:
            self._direct_lock_reset_confirm_count += 1
        else:
            self._direct_lock_reset_confirm_count = 0

        cfg = self.director.config
        reset_elapsed = now - (self._direct_lock_reset_started_at or now)
        if (
            reset_elapsed < cfg.platform_task_reset_hold_s
            or self._direct_lock_reset_confirm_count
            < FC_TASK_RESET_CONFIRM_COUNT
        ):
            return
        if not self._t265_ready_for_platform_retakeoff():
            self._block_platform_retakeoff(
                "第二次起飞边沿发送前T265状态不满足门槛"
            )
            return

        try:
            position = tuple(float(v) for v in self.realsense.get_position())
            anchor_xy = (position[0], position[1])
        except Exception as exc:
            self._block_platform_retakeoff(f"无法读取复飞锚点: {exc}")
            return

        self.director.begin_platform_retakeoff(
            now=now,
            anchor_xy_m=anchor_xy,
        )
        self._landing_gate_passed = False
        self._touchdown_confirmed = False
        self._deck_ride_complete = False
        self._landing_aborted = False
        self._tracker_active_prev = False
        self._landing_vz_applied_m_s = 0.0
        self._last_nav_time = None
        self.laser_contact.reset()
        self._direct_lock_laser_samples.clear()
        self._ramp_z_cm = 5.0
        self._direct_lock_active = False
        self._direct_lock_reset_started_at = None
        with lock:
            self.se_fc[2] = 0
            self.se_fc[7] = 0
        logger.warning(
            "任务二飞控任务复位确认完成：T265坐标保持连续，"
            f"复飞锚点=({anchor_xy[0]:+.2f},{anchor_xy[1]:+.2f})m；"
            "局部H偏移="
            f"({self.director.config.retakeoff_h_offset_x_m:+.2f},"
            f"{self.director.config.retakeoff_h_offset_y_m:+.2f})m；"
            "将在复飞爬升稳定后以当时位置建立局部原点；"
            "开始第二次起飞"
        )
        # 下一控制周期复用成熟的基础起飞链。takeoff()会产生task_sta 0->1，
        # 到达30cm后自动回到NAVIGATE并进入RETAKEOFF/RETURN_H。
        self.state = "TAKEOFF"

    def stop_all(self):
        """After direct lock, stop software without emitting legacy command 101."""
        if not self._direct_lock_active:
            return super().stop_all()

        logger.info("任务二结束：保持近地直接锁桨指令，不发送OneKey_Land")
        self.set_speed(0, 0, 0, 5)
        with lock:
            self.se_fc[7] = FC_DIRECT_LOCK_SIGN
        self.heading_hold.disarm("task2_direct_lock_complete")
        self._resource_monitor.stop()
        try:
            if self._log_file:
                self._log_file.close()
        except Exception:
            pass
        if self.realsense:
            self.realsense.stop()
        self.task_running = False

    def _should_handoff_to_fc_one_key_land(
        self, command, laser_height: float | None
    ) -> bool:
        """达到末段高度后只触发一次飞控一键降落接管。"""
        cfg = self.director.config
        if (
            not cfg.fc_one_key_land_enabled
            or command.phase != Task2Phase.DYNAMIC_LANDING
            or not command.landing_active
        ):
            return False
        relative_height = (
            float(laser_height)
            if laser_height is not None
            else float(self._ramp_z_cm) / 100.0
        )
        return relative_height <= cfg.fc_one_key_land_height_m

    def _begin_fc_one_key_land(self, laser_height: float | None) -> None:
        """发送飞控既有一键降落格式，并切入基础LAND确认流程。"""
        relative_height = (
            f"{laser_height:.2f}m" if laser_height is not None else "setpoint"
        )
        logger.warning(
            "任务二末段交给飞控一键降落："
            f"relative_height={relative_height}, task_sta=0, "
            "next_task_sign=101"
        )
        self.heading_hold.disarm("task2_fc_one_key_land")
        self.set_speed(0, 0, 0, 0)
        with lock:
            self.se_fc[2] = 0
            self.se_fc[7] = 101
        self._ramp_z_cm = 0.0
        self.state = "LAND"

    def _smooth_landing_vz(self, target_vz_m_s: float, dt: float) -> float:
        """Slew only downward commands; stop/climb commands take effect now."""
        target = float(target_vz_m_s)
        if target >= 0.0:
            return target
        current = float(self._landing_vz_applied_m_s)
        max_delta = self.dynamic_landing.config.descend_slew_m_s2 * max(
            0.0, float(dt)
        )
        delta = target - current
        return current + max(-max_delta, min(max_delta, delta))

    def _verify_continuous_arm_contract(self, command) -> bool:
        """Verify that every airborne task-two phase is actually unlocked."""
        if not command.keep_armed:
            return True

        with lock:
            task_sta = int(self.se_fc[2])
            next_task_sign = int(self.se_fc[7])
            unlock_sta = int(self.re_fc[5]) if len(self.re_fc) > 5 else 0

        expected_task_sta = 0 if self._dry_run else 1
        if task_sta != expected_task_sta or next_task_sign != 0:
            logger.error(
                "任务二持续解锁契约被破坏："
                f"phase={command.phase.value}, task_sta={task_sta}, "
                f"expected_task_sta={expected_task_sta}, "
                f"next_task_sign={next_task_sign}；禁止继续飞行并转HOVER_WAIT"
            )
            self.set_speed(0, 0, 0, int(round(self._ramp_z_cm)))
            self.state = "HOVER_WAIT"
            return False

        # 正常的第二次起飞边沿应已在平台锁桨处理链中完成。进入
        # RETAKEOFF 导航后若仍是锁桨状态，禁止再次盲目制造起飞边沿。
        if (
            not self._dry_run
            and command.phase
            in (
                Task2Phase.RETAKEOFF,
                Task2Phase.STABILIZE_AFTER_RETAKEOFF,
            )
            and unlock_sta == 0
        ):
            logger.error(
                "任务二复升门禁拒绝：飞控反馈已锁桨；"
                "不会发送第二次解锁/起飞指令，转HOVER_WAIT等待人工处理"
            )
            self.set_speed(0, 0, 0, int(round(self._ramp_z_cm)))
            self.state = "HOVER_WAIT"
            return False

        return True

    def hover_wait(self):
        """Task-two hover fallback with a bounded automatic landing delay."""
        super().hover_wait()
        self._tick_post_retakeoff_hover_auto_land(time.monotonic())

    def descend(self, pos):
        """Use realtime-frame descent plus guarded 102 lock for final landing."""
        if not self._final_realtime_landing_active:
            return super().descend(pos)

        super().descend(pos)
        if self.state == "HOVER_WAIT":
            # A timeout must not switch the final fallback back to an indefinite
            # hover.  Restart the realtime descent ramp from the current height.
            for attr in ("_descend_start", "_hover_wait_start"):
                if hasattr(self, attr):
                    delattr(self, attr)
            self.state = "DESCEND"
            logger.warning(
                "任务二最终实时帧下降超时；重新从当前高度继续下降，"
                "不调用一键降落"
            )
            return
        if self.state != "LAND":
            return

        with lock:
            unlock_sta = int(self.re_fc[5]) if len(self.re_fc) > 5 else 0
            laser_height = (
                self.serial_fc_ref._last_laser_height_cm
                if self.serial_fc_ref is not None
                else None
            )
        if unlock_sta == 0:
            logger.info("任务二最终实时帧下降后飞控已锁桨，结束任务")
            self.director.mission_success = True
            self.director._transition(
                Task2Phase.COMPLETE,
                time.monotonic(),
                "final_realtime_descent_locked",
            )
            self.state = "END"
            return

        self._begin_fc_direct_lock(
            float(laser_height)
            if laser_height is not None and math.isfinite(laser_height)
            else None
        )
        logger.warning(
            "任务二最终实时帧下降已近地：发送102直接锁桨；"
            "不发送101，不调用一键降落"
        )

    def _tick_post_retakeoff_hover_auto_land(self, now: float) -> bool:
        post_retakeoff_phases = (
            Task2Phase.RETAKEOFF,
            Task2Phase.STABILIZE_AFTER_RETAKEOFF,
            Task2Phase.VISUAL_RETURN_A,
            Task2Phase.RETURN_H,
            Task2Phase.LAND_H,
        )
        if self.director.phase not in post_retakeoff_phases:
            self._task2_hover_wait_started_at = None
            return False
        if self._task2_hover_wait_started_at is None:
            self._task2_hover_wait_started_at = float(now)
            return False
        if (
            float(now) - self._task2_hover_wait_started_at
            < self.director.config.hover_wait_auto_land_delay_s
        ):
            return False

        with lock:
            unlock_sta = int(self.re_fc[5]) if len(self.re_fc) > 5 else 0
        if unlock_sta == 0:
            logger.warning(
                "任务二复飞后HOVER_WAIT超时，但飞控反馈已锁桨；"
                "判定飞行器已经落地并结束任务"
            )
            self.director.mission_success = True
            self.director._transition(
                Task2Phase.COMPLETE,
                float(now),
                "hover_wait_timeout_already_locked",
            )
            self.state = "END"
            return True

        current_xy = (
            float(self.last_world_position[0]),
            float(self.last_world_position[1]),
        )
        self.director._return_h_target = current_xy
        self.director._return_h_waypoints = (current_xy,)
        self.director._return_h_index = 0
        self.director._transition(
            Task2Phase.LAND_H,
            float(now),
            "hover_wait_timeout_force_descend",
        )
        for attr in ("_descend_start", "_hover_wait_start"):
            if hasattr(self, attr):
                delattr(self, attr)
        self._task2_hover_wait_started_at = None
        self._final_realtime_landing_active = True
        self.state = "DESCEND"
        logger.warning(
            "任务二复飞后HOVER_WAIT持续超过"
            f"{self.director.config.hover_wait_auto_land_delay_s:.1f}秒；"
            "停止水平运动并在当前位置强制执行basic两级下降"
        )
        return True

    def _update_stationary_platform_estimate(
        self, *, gate, world_position, now: float
    ) -> PlatformEstimate | None:
        """视觉更新固定平台世界位置，近地丢失后保持最后一次测量。"""
        if gate.found and gate.error_xy_m is not None:
            self._stationary_platform_measurement = (
                world_position[0] + gate.error_xy_m[0],
                world_position[1] + gate.error_xy_m[1],
            )
        if self._stationary_platform_measurement is None:
            self._stationary_platform_measurement = self.director.sync_c
        return self.platform_tracker.update(
            self._stationary_platform_measurement[0],
            self._stationary_platform_measurement[1],
            now,
            quality=gate.quality if gate.found else 80,
        )

    def _update_visual_platform_estimate(
        self, *, gate, world_position, now: float
    ) -> PlatformEstimate | None:
        """Fuse each new visual frame into the moving-platform world estimate."""
        if (
            gate.found
            and gate.error_xy_m is not None
            and gate.seq != self._last_platform_vision_seq
        ):
            self._last_platform_vision_seq = gate.seq
            self._platform_measurement_source = "visual"
            return self.platform_tracker.update(
                world_position[0] + gate.error_xy_m[0],
                world_position[1] + gate.error_xy_m[1],
                now,
                quality=gate.quality,
            )
        estimate = self.platform_tracker.predict(now)
        self._platform_measurement_source = (
            "visual_prediction" if estimate is not None else "lost"
        )
        return estimate

    def _landing_xy_speed_limit(self, relative_height_m: float) -> float:
        cfg = self.director.config
        if relative_height_m > self.dynamic_landing.config.mid_height_m:
            return cfg.landing_xy_speed_high_m_s
        if relative_height_m > self.dynamic_landing.config.low_height_m:
            return cfg.landing_xy_speed_mid_m_s
        return cfg.landing_xy_speed_low_m_s

    @staticmethod
    def _limit_xy_speed(vx: float, vy: float, limit: float):
        norm = math.hypot(vx, vy)
        safe_limit = max(0.0, float(limit))
        if norm <= safe_limit or norm <= 1e-9:
            return vx, vy
        scale = safe_limit / norm
        return vx * scale, vy * scale

    def _hold_anchor_for_phase(self, phase: Task2Phase):
        if phase in (Task2Phase.WAIT_START, Task2Phase.TAKEOFF, Task2Phase.HOLD_3S):
            return H
        if phase == Task2Phase.ACQUIRE_TARGET:
            return B_PRE
        if phase in (Task2Phase.SYNC_TARGET_AT_C, Task2Phase.ACTIVATE_TRACKER):
            return self.director.sync_c
        if phase == Task2Phase.RETAKEOFF:
            return self.director._retakeoff_anchor
        if phase == Task2Phase.STABILIZE_AFTER_RETAKEOFF:
            return self.director._safe_hover_anchor
        if phase == Task2Phase.LAND_H:
            return self.director._return_h_target
        if phase in (
            Task2Phase.SAFE_HOVER_D,
            Task2Phase.SAFE_HOVER_AFTER_RETAKEOFF,
        ):
            return self.director._safe_hover_anchor
        return None

    def _run_landing(
        self,
        estimate: PlatformEstimate,
        world_position,
        velocity,
        laser_height,
        gate,
        observation,
        contact_evidence: bool,
        safety_fault,
        car_velocity,
    ):
        """组装 LandingInput 并调 DynamicLandingController.tick()。"""
        relative_height = (
            laser_height
            if laser_height is not None
            else max(0.01, world_position[2])
        )
        relative_vel = (
            estimate.vx_m_s - velocity[0],
            estimate.vy_m_s - velocity[1],
        )
        position_error = (
            estimate.x_m - world_position[0],
            estimate.y_m - world_position[1],
        )
        visual_too_close = False
        if observation is not None:
            flags = FeatureFlag(observation.flags)
            visual_too_close = bool(flags & FeatureFlag.TOO_CLOSE)
        fixed_point_no_vision = (
            self.director.config.stationary_retakeoff_test
            and self.director.config.stationary_skip_vision
        )
        visual_usable = fixed_point_no_vision or (
            gate.found and not gate.ambiguous
        )
        # In the fixed-platform no-vision test, the repeatedly refreshed T265
        # platform coordinate replaces both visual and car-motion freshness.
        car_motion_fresh = fixed_point_no_vision or car_velocity is not None
        # roll/pitch: 后续接入飞控遥测
        roll_deg = 0.0
        pitch_deg = 0.0
        landing_input = LandingInput(
            relative_height_m=relative_height,
            vertical_speed_m_s=float(velocity[2]),
            relative_velocity_xy_m_s=relative_vel,
            position_error_xy_m=position_error,
            estimate_uncertainty_m=estimate.uncertainty_m,
            visual_usable=visual_usable,
            visual_too_close=visual_too_close,
            car_motion_fresh=car_motion_fresh,
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            contact_evidence=contact_evidence,
            t265_healthy=safety_fault is None,
        )
        return self.dynamic_landing.tick(landing_input)

    def _laser_height(self) -> float | None:
        if self.serial_fc_ref is None:
            return None
        value = float(self.serial_fc_ref._last_laser_height_cm)
        return value if laser_height_valid(value) else None

    def _log_runtime(
        self,
        now,
        command,
        world_position,
        velocity,
        confidence,
        laser_height,
        height,
        gate,
        car_position,
        car_velocity,
        offset_ready,
        offset,
        estimate,
        landing_vz,
        actual_command_xy,
        horizontal_control_source,
    ) -> None:
        if self._log_file is None or now - self._last_runtime_log < 0.10:
            return
        self._last_runtime_log = now
        try:
            with self._log_lock:
                self._log_file.write(
                    json.dumps(
                        {
                            "event": "task2_runtime",
                            "t": round(time.time(), 3),
                            "phase": command.phase.value,
                            "reason": command.reason,
                            "world_pos": [round(v, 4) for v in world_position],
                            "t265_velocity": [round(v, 4) for v in velocity[:2]],
                            "t265_confidence": confidence,
                            "laser_height_m": laser_height,
                            "laser_setpoint_m": round(
                                height.laser_setpoint_m, 4
                            ),
                            "height_mode": height.mode,
                            "target_xy_m": [
                                round(command.target_xy_m[0], 4),
                                round(command.target_xy_m[1], 4),
                            ],
                            "command_xy_m_s": [
                                round(actual_command_xy[0], 4),
                                round(actual_command_xy[1], 4),
                            ],
                            "director_command_xy_m_s": [
                                round(command.vx_m_s, 4),
                                round(command.vy_m_s, 4),
                            ],
                            "vision_seq": gate.seq,
                            "vision_found": gate.found,
                            "vision_quality": gate.quality,
                            "vision_error_xy_m": gate.error_xy_m,
                            "vision_reason": gate.reason,
                            "car_position_xy_m": (
                                [round(car_position[0], 4), round(car_position[1], 4)]
                                if car_position
                                else None
                            ),
                            "car_velocity_xy_m_s": (
                                [round(car_velocity[0], 4), round(car_velocity[1], 4)]
                                if car_velocity
                                else None
                            ),
                            "offset_ready": offset_ready,
                            "offset_xy_m": [
                                round(offset[0], 4),
                                round(offset[1], 4),
                            ],
                            "tracker_initialized": self.platform_tracker.initialized,
                            "platform_estimate": (
                                {
                                    "x_m": round(estimate.x_m, 4),
                                    "y_m": round(estimate.y_m, 4),
                                    "vx_m_s": round(estimate.vx_m_s, 4),
                                    "vy_m_s": round(estimate.vy_m_s, 4),
                                    "uncertainty_m": round(
                                        estimate.uncertainty_m, 4
                                    ),
                                    "predicted": estimate.predicted,
                                }
                                if estimate
                                else None
                            ),
                            "landing_state": (
                                self.dynamic_landing.state.value
                                if command.landing_active
                                else None
                            ),
                            "landing_vz_m_s": round(landing_vz, 4),
                            "landing_vz_applied_m_s": round(
                                self._landing_vz_applied_m_s, 4
                            ),
                            "landing_gate_passed": self._landing_gate_passed,
                            "touchdown_confirmed": self._touchdown_confirmed,
                            "deck_ride_complete": self._deck_ride_complete,
                            "landing_aborted": self._landing_aborted,
                            "tracker_active": command.tracker_active,
                            "landing_active": command.landing_active,
                            "mission_success": command.mission_success,
                            "platform_measurement_source": self._platform_measurement_source,
                            "horizontal_control_source": horizontal_control_source,
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                self._log_file.flush()
        except Exception:
            pass
