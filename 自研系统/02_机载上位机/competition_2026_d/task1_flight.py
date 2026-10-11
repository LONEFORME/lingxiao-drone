"""任务一核心与 basic 飞控安全状态机的飞行适配类。

正式任务一由无人机自身 T265 和固定场地路径生成主水平速度。小车 T265/
``CAR_POSITION`` 不参与控制；Cyber Camera 只提供受限微调、目标确认与投放门禁。
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Callable

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

from Lcode.Logger import logger  # noqa: E402
from Mission_GPT import laser_height_valid, mission  # noqa: E402

from .payload_actuator import PayloadActuator  # noqa: E402
from .task1_mission import (  # noqa: E402
    B_PRE,
    H,
    Task1Config,
    Task1Input,
    Task1MissionDirector,
    Task1Phase,
)
from .task1_runtime import (  # noqa: E402
    HeightReference,
    HeightReferenceConfig,
    Task1T265SafetyMonitor,
    WorldDeckHeightController,
    observation_to_gate_sample,
)
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


class Task1FlightMission(mission):
    """飞控循环内唯一写入 XY 速度的任务一适配器。"""

    def __init__(
        self,
        re_fc,
        se_fc,
        realsense_obj,
        serial_fc_ref,
        *,
        vision_reader: CyberCamReader,
        payload_actuator: PayloadActuator,
        task_config: Task1Config | None = None,
        vision_config: dict | None = None,
        car_speed_provider: Callable[[], float | None] | None = None,
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
        self.director = Task1MissionDirector(task_config)
        self.vision_reader = vision_reader
        self.payload_actuator = payload_actuator
        self.height_controller = WorldDeckHeightController(
            HeightReferenceConfig(
                normal_ascent_slew_m_s=(
                    self.director.config.takeoff_ascent_slew_m_s
                )
            )
        )
        self.t265_safety = Task1T265SafetyMonitor()
        self.car_speed_provider = car_speed_provider
        if height_source not in ("t265", "laser"):
            raise ValueError("height_source must be 't265' or 'laser'")
        self.height_source = height_source
        self.vision_config = {
            "max_age_s": 0.15,
            "min_quality": min(
                self.director.config.vision_min_quality,
                self.director.config.acquire_vision_min_quality,
            ),
            "image_center_px": (320.0, 240.0),
            "focal_px": (570.0, 570.0),
            "target_offset_xy_m": (0.0, 0.0),
            **(vision_config or {}),
        }
        self._last_task1_phase = self.director.phase
        self._last_runtime_log = 0.0

    def navigate(self, _laser_position, yaw):
        now = time.monotonic()
        try:
            world_position = tuple(float(v) for v in self.realsense.get_position())
            velocity = tuple(float(v) for v in self.realsense.get_velocity())
            confidence = int(self.realsense.get_tracking_confidence())
        except Exception as exc:
            logger.error(f"任务一读取T265失败，本周期悬停: {exc}")
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            return

        laser_height = self._laser_height()
        if confidence == 0:
            self.set_speed(0, 0, 0, int(self._ramp_z_cm))
            logger.warning("任务一T265置信度为0，冻结状态机并原地悬停")
            return

        phase_before_tick = self.director.phase
        hold_anchor = None
        if phase_before_tick in (
            Task1Phase.WAIT_START,
            Task1Phase.TAKEOFF,
            Task1Phase.HOLD_3S,
            Task1Phase.LAND_H,
        ):
            hold_anchor = H
        elif (
            phase_before_tick == Task1Phase.ACQUIRE_TARGET
            and not self.director.config.vision_track_only
            and not self.director.config.intercept_vision_early_stop_enabled
        ):
            # 固定路径联合模式必须在B_PRE定点等待；纯视觉模式仍允许离开B_PRE。
            hold_anchor = B_PRE
        ground_reference_expected = phase_before_tick in (
            Task1Phase.WAIT_START,
            Task1Phase.TAKEOFF,
            Task1Phase.HOLD_3S,
            Task1Phase.INTERCEPT_B_PRE,
            Task1Phase.ACQUIRE_TARGET,
            Task1Phase.LAND_H,
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
                f"任务一T265安全保护触发: {safety_fault}；"
                "冻结当前激光高度并转HOVER_WAIT，等待人工接管"
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
        payload_state = self.payload_actuator.poll()
        car_speed = (
            None if self.car_speed_provider is None else self.car_speed_provider()
        )
        control_height = (
            laser_height
            if self.height_source == "laser" and laser_height is not None
            else world_position[2]
        )
        command = self.director.tick(
            Task1Input(
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
                payload_state=payload_state,
            )
        )
        if command.release_requested:
            self.payload_actuator.release_once()

        if self.height_source == "laser":
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
                    self._ramp_z_cm / 100.0,
                    True,
                    "laser_height",
                    "ok",
                )
        else:
            height = self.height_controller.command(
                timestamp=now,
                current_world_height_m=world_position[2],
                current_laser_height_m=laser_height,
                target_world_height_m=command.target_world_height_m,
                target_deck_height_m=command.target_deck_height_m,
            )
        if height.valid:
            self._ramp_z_cm = height.laser_setpoint_m * 100.0

        self._heading_status = self._update_heading_hold(yaw, confidence)
        yaw_cmd = self._heading_status.command_dps
        vx_cms = int(round(command.vx_m_s * 100.0))
        vy_cms = int(round(command.vy_m_s * 100.0))
        self.set_speed(vx_cms, vy_cms, yaw_cmd, int(round(self._ramp_z_cm)))

        if command.phase != self._last_task1_phase:
            logger.info(
                f"任务一状态: {self._last_task1_phase.value} -> "
                f"{command.phase.value} ({command.reason})"
            )
            self._last_task1_phase = command.phase

        self._log_runtime(
            now,
            command,
            world_position,
            velocity,
            confidence,
            laser_height,
            height,
            gate,
            observation,
            payload_state,
        )

        if (
            command.phase == Task1Phase.LAND_H
            and command.land_requested
            and math.hypot(world_position[0], world_position[1])
            <= self.director.config.final_landing_radius_m
        ):
            logger.info(
                "任务一在(0,0,0.15)稳定满足最终门槛，"
                "转入带T265水平闭环的两级降落"
            )
            self.state = "DESCEND"

    def _descend_horizontal_command(self, pos) -> tuple[int, int]:
        """任务一末段下降继续低速闭环到H，避免开环零速度随气流漂移。"""
        try:
            confidence = int(self.realsense.get_tracking_confidence())
        except Exception:
            return 0, 0
        if confidence < self.director.config.t265_min_confidence:
            return 0, 0
        vx, vy = self.director._point_velocity(
            (float(pos[0]), float(pos[1])),
            H,
            self.director.config.final_descend_horizontal_max_speed_m_s,
        )
        return int(round(vx * 100.0)), int(round(vy * 100.0))

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
        observation,
        payload_state,
    ) -> None:
        if self._log_file is None or now - self._last_runtime_log < 0.10:
            return
        self._last_runtime_log = now
        try:
            with self._log_lock:
                self._log_file.write(
                    json.dumps(
                        {
                            "event": "task1_runtime",
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
                                round(command.vx_m_s, 4),
                                round(command.vy_m_s, 4),
                            ],
                            "base_path_command_xy_m_s": [
                                round(command.base_vx_m_s, 4),
                                round(command.base_vy_m_s, 4),
                            ],
                            "vision_trim_command_xy_m_s": [
                                round(command.vision_trim_vx_m_s, 4),
                                round(command.vision_trim_vy_m_s, 4),
                            ],
                            "vision_seq": gate.seq,
                            "vision_stream_id": (
                                None
                                if observation is None
                                else observation.stream_id
                            ),
                            "vision_capture_ms": (
                                None
                                if observation is None
                                else observation.capture_ms
                            ),
                            "vision_age_s": (
                                None
                                if observation is None
                                else round(observation.age_s(now), 4)
                            ),
                            "vision_center_px": (
                                None
                                if observation is None
                                else [observation.cx, observation.cy]
                            ),
                            "vision_outer_px": (
                                None
                                if observation is None
                                else observation.outer_px
                            ),
                            "vision_flags": (
                                None
                                if observation is None
                                else observation.flags
                            ),
                            "vision_found": gate.found,
                            "vision_quality": gate.quality,
                            "vision_error_xy_m": gate.error_xy_m,
                            "vision_error_norm_m": (
                                None
                                if gate.error_xy_m is None
                                else round(math.hypot(*gate.error_xy_m), 4)
                            ),
                            "vision_reason": gate.reason,
                            "payload_state": payload_state.value,
                            "drop_committed": command.drop_committed,
                            "drop_released": command.drop_released,
                            "horizontal_control_source": (
                                "task1_braking_before_pure_vision"
                                if command.reason
                                == "pure_vision_takeover_braking"
                                else (
                                    "task1_pure_vision_centering"
                                    if command.phase
                                    == Task1Phase.ACQUIRE_TARGET
                                    else "task1_fixed_path_uav_t265_plus_limited_vision_trim"
                                )
                            ),
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                self._log_file.flush()
        except Exception:
            pass
