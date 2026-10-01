"""任务二正式一键启动入口。

启动顺序与任务一一致，区别仅在于：
1. CAR_START 的 task_mode 必须为 2；
2. 构造的是 Task2FlightMission，C 点后激活 T265 坐标伴飞与动态降落。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import zlib
from dataclasses import replace
from pathlib import Path

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.gpio_led import set_rgb_led  # noqa: E402
from Lcode.global_variable import fc_last_rx_time, sp_side  # noqa: E402
from main import wait_for_start_button  # noqa: E402
from t265 import t265_class  # noqa: E402

from shared.competition_2026_d_protocol import (  # noqa: E402
    Device,
    Flag,
    MessageType,
    pack_payload,
    unpack_payload,
)

from .Lcode.air_ground_link import AirGroundLink, LinkConfig  # noqa: E402
from .control.formation_controller import FormationConfig  # noqa: E402
from .dynamic_landing import LandingConfig  # noqa: E402
from .payload_servo import build_payload_actuator  # noqa: E402
from .task1_start import (  # noqa: E402
    READY_BT,
    READY_FC_LINK,
    READY_PAYLOAD_LOCKED,
    READY_VISION,
    Task1StartGate,
    _stop_mission_safely,
    _wait_for_fc,
    _wait_for_vision,
)
from .task2_flight import Task2FlightMission  # noqa: E402
from .task2_mission import Task2Config  # noqa: E402
from .task2_telemetry import (  # noqa: E402
    Task2TelemetryPublisher,
    Task2TelemetrySample,
)
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


TASK2_MODE = 2
TASK2_MASK = 1 << 1


class NullVisionReader:
    """No-op reader used only by the fixed-point no-vision bench mode."""

    def start(self) -> bool:
        return True

    def latest(self, _now=None, _max_age_s=0.15):
        return None

    def close(self) -> None:
        return None


class Task2StartGate(Task1StartGate):
    """只接受来自小车、任务模式为2、session非0的CAR_START。"""

    def wait(self, timeout_s=None):
        started = self.clock()
        next_ready = 0.0
        next_diagnostic = started + 3.0
        while not self._start_event.is_set():
            now = self.clock()
            if timeout_s is not None and now - started >= timeout_s:
                return None
            if now >= next_ready:
                next_ready = now + self.ready_interval_s
                self.link.publish(
                    MessageType.UAV_READY,
                    pack_payload(
                        MessageType.UAV_READY,
                        (
                            TASK2_MASK,
                            self.ready_bits,
                            self.config_hash,
                        ),
                    ),
                    session_id=0,
                    dest=Device.CAR,
                )
            if now >= next_diagnostic:
                next_diagnostic = now + 3.0
                stats = self.link.stats
                logger.info(
                    "蓝牙接收诊断: "
                    f"bytes={stats.rx_bytes}, "
                    f"valid_frames={stats.rx_frames}, "
                    f"parser_rejected={stats.rx_rejected}, "
                    f"wrong_dest={stats.rx_wrong_dest}, "
                    f"start_rejected={self.rejected_start_frames}, "
                    f"state_rejected={self.rejected_state_frames}, "
                    f"position_rejected={self.rejected_position_frames}, "
                    f"last_type={stats.last_frame_type}, "
                    f"last_src={stats.last_frame_source}, "
                    f"last_dst={stats.last_frame_dest}, "
                    f"last_hex={stats.last_rx_hex or '-'}"
                )
            self._start_event.wait(0.05)
        return self.session_id

    def _handle_start(self, frame):
        required = int(Flag.ACK_REQUIRED | Flag.EVENT)
        if int(frame.flags) & required != required:
            self.rejected_start_frames += 1
            if int(frame.flags) & int(Flag.ACK_REQUIRED):
                self.link.acknowledge(frame, result=1)
            logger.warning(
                "拒绝CAR_START: flags=0x%02X，必须包含0x%02X",
                int(frame.flags),
                required,
            )
            return
        try:
            task_mode, car_config_hash = unpack_payload(
                MessageType.CAR_START, frame.payload
            )
        except ValueError:
            self.rejected_start_frames += 1
            self.link.acknowledge(frame, result=1)
            logger.warning(
                "拒绝CAR_START: payload长度或格式错误，实际%d字节",
                len(frame.payload),
            )
            return
        if task_mode != TASK2_MODE or int(frame.session_id) == 0:
            self.rejected_start_frames += 1
            self.link.acknowledge(frame, result=1)
            logger.warning(
                "拒绝CAR_START: task_mode=%d, session_id=%d",
                task_mode,
                int(frame.session_id),
            )
            return
        with self._lock:
            if self._session_id is not None:
                result = 0 if self._session_id == int(frame.session_id) else 1
                self.link.acknowledge(frame, result=result)
                return
            self._session_id = int(frame.session_id)
            self._car_config_hash = int(car_config_hash)
            self._car_position = None
            self._last_car_position_seq = None
        self.link.acknowledge(frame, result=0)
        self._start_event.set()


def _load_task_config(data: dict) -> Task2Config:
    task = data["task2"]
    return Task2Config(
        cruise_height_m=float(task["cruise_height_m"]),
        follow_height_m=float(task["follow_height_m"]),
        final_height_m=float(task["final_height_m"]),
        hold_duration_s=float(task["hover_duration_s"]),
        intercept_speed_m_s=float(task["intercept_speed_m_s"]),
        car_speed_m_s=float(task["car_speed_m_s"]),
        car_speed_scale=float(task.get("car_speed_scale", 1.0)),
        return_speed_m_s=float(task["return_speed_m_s"]),
        point_arrival_radius_m=float(task["path_capture_radius_m"]),
        hold_position_max_speed_m_s=float(
            task.get("hold_position_max_speed_m_s", 0.15)
        ),
        hold_stable_speed_m_s=float(
            task.get("hold_stable_speed_m_s", 0.12)
        ),
        path_lookahead_m=float(task.get("path_lookahead_m", 0.20)),
        path_cross_track_kp=float(
            task.get("path_cross_track_kp", 0.80)
        ),
        path_max_correction_m_s=float(
            task.get("path_max_correction_m_s", 0.08)
        ),
        path_max_speed_m_s=float(task.get("path_max_speed_m_s", 0.20)),
        path_max_accel_m_s2=float(
            task.get("path_max_accel_m_s2", 0.30)
        ),
        vision_min_quality=int(task["vision_min_quality"]),
        vision_confirm_frames=int(task["vision_confirm_frames"]),
        drop_max_error_m=float(task["drop_max_error_m"]),
        t265_min_confidence=int(task.get("t265_min_confidence", 2)),
        retakeoff_height_m=float(task.get("retakeoff_height_m", 1.30)),
        retakeoff_forward_y_speed_m_s=float(
            task.get("retakeoff_forward_y_speed_m_s", -0.05)
        ),
        retakeoff_t265_jump_protection_enabled=bool(
            task.get("retakeoff_t265_jump_protection_enabled", False)
        ),
        retakeoff_h_offset_x_m=float(
            task.get("retakeoff_h_offset_x_m", -0.50)
        ),
        retakeoff_h_offset_y_m=float(
            task.get("retakeoff_h_offset_y_m", -0.60)
        ),
        visual_return_a_enabled=bool(
            task.get("visual_return_a_enabled", False)
        ),
        visual_return_a_center_radius_m=float(
            task.get("visual_return_a_center_radius_m", 0.15)
        ),
        visual_return_a_stop_speed_m_s=float(
            task.get("visual_return_a_stop_speed_m_s", 0.015)
        ),
        visual_return_a_stop_hold_s=float(
            task.get("visual_return_a_stop_hold_s", 1.0)
        ),
        visual_return_a_min_follow_s=float(
            task.get("visual_return_a_min_follow_s", 2.0)
        ),
        visual_return_a_timeout_s=float(
            task.get("visual_return_a_timeout_s", 30.0)
        ),
        visual_return_a_kp=float(task.get("visual_return_a_kp", 0.65)),
        visual_return_a_max_speed_m_s=float(
            task.get("visual_return_a_max_speed_m_s", 0.15)
        ),
        visual_return_a_max_accel_m_s2=float(
            task.get("visual_return_a_max_accel_m_s2", 0.45)
        ),
        return_h_auto_land_timeout_s=float(
            task.get("return_h_auto_land_timeout_s", 15.0)
        ),
        land_h_auto_descend_timeout_s=float(
            task.get("land_h_auto_descend_timeout_s", 8.0)
        ),
        hover_wait_auto_land_delay_s=float(
            task.get("hover_wait_auto_land_delay_s", 3.0)
        ),
        abort_climb_height_m=float(task.get("abort_climb_height_m", 1.50)),
        activate_tracker_timeout_s=float(
            task.get("activate_tracker_timeout_s", 3.0)
        ),
        vision_target_offset_x_m=float(
            task.get("vision_target_offset_x_m", 0.0)
        ),
        vision_target_offset_y_m=float(
            task.get("vision_target_offset_y_m", 0.0)
        ),
        c_sync_vision_kp=float(task.get("c_sync_vision_kp", 0.45)),
        c_sync_vision_deadband_m=float(
            task.get("c_sync_vision_deadband_m", 0.03)
        ),
        c_sync_vision_max_speed_m_s=float(
            task.get("c_sync_vision_max_speed_m_s", 0.10)
        ),
        c_sync_vision_max_accel_m_s2=float(
            task.get("c_sync_vision_max_accel_m_s2", 0.25)
        ),
        c_sync_vision_filter_window_s=float(
            task.get("c_sync_vision_filter_window_s", 0.20)
        ),
        c_sync_vision_control_period_s=float(
            task.get("c_sync_vision_control_period_s", 0.08)
        ),
        c_sync_vision_loss_grace_s=float(
            task.get("c_sync_vision_loss_grace_s", 0.20)
        ),
        require_car_position_at_c=bool(
            task.get("require_car_position_at_c", True)
        ),
        landing_xy_speed_high_m_s=float(
            task.get("landing_xy_speed_high_m_s", 0.15)
        ),
        landing_xy_speed_mid_m_s=float(
            task.get("landing_xy_speed_mid_m_s", 0.10)
        ),
        landing_xy_speed_low_m_s=float(
            task.get("landing_xy_speed_low_m_s", 0.07)
        ),
    )


def build_open_loop_cd_test_config(
    base: Task2Config,
    *,
    cd_speed_m_s: float = 0.06,
) -> Task2Config:
    """1.3m直飞C、视觉放行、C-D匀速下降至0.5m的安全测试配置。"""
    speed = float(cd_speed_m_s)
    if not 0.02 <= speed <= 0.20:
        raise ValueError("C-D固定速度必须在0.02~0.20m/s之间")
    return replace(
        base,
        cruise_height_m=1.30,
        follow_height_m=1.30,
        hold_duration_s=0.0,
        car_speed_m_s=speed,
        car_speed_scale=1.0,
        vision_min_quality=40,
        vision_confirm_frames=5,
        drop_max_error_m=0.15,
        require_car_position_at_c=False,
        safe_open_loop_cd_test=True,
        safe_hover_height_m=0.50,
        safe_c_offset_x_m=-0.15,
        safe_c_offset_y_m=0.25,
        safe_descent_rate_m_s=0.12,
        safe_follow_cutoff_s=25.0,
    )


def build_vision_landing_test_config(base: Task2Config) -> Task2Config:
    """Direct-to-C moving-platform landing with continuous visual correction."""
    return replace(
        base,
        cruise_height_m=1.30,
        follow_height_m=1.30,
        retakeoff_height_m=1.30,
        hold_duration_s=0.0,
        hold_position_max_speed_m_s=0.22,
        hold_velocity_kd=0.45,
        vision_min_quality=40,
        vision_confirm_frames=5,
        drop_max_error_m=0.15,
        require_car_position_at_c=False,
        safe_open_loop_cd_test=False,
        vision_landing_test=True,
        direct_descent_after_trigger=True,
        fc_one_key_land_enabled=False,
        fc_one_key_land_height_m=0.10,
        fc_direct_lock_enabled=True,
        fc_direct_lock_height_m=0.05,
        fc_direct_lock_stable_height_m=0.10,
        fc_direct_lock_stable_hold_s=0.35,
        fc_direct_lock_stable_tolerance_m=0.01,
        fc_direct_lock_stable_min_samples=4,
        fc_direct_lock_laser_dropout_grace_s=0.25,
        platform_retakeoff_enabled=True,
        visual_return_a_enabled=True,
        platform_locked_hold_s=5.0,
        platform_task_reset_hold_s=0.30,
        retakeoff_stabilize_s=0.80,
        land_only_after_touchdown=False,
        landing_xy_speed_high_m_s=0.12,
        landing_xy_speed_mid_m_s=0.09,
        landing_xy_speed_low_m_s=0.06,
        c_sync_vision_kp=0.45,
        c_sync_vision_deadband_m=0.03,
        c_sync_vision_max_speed_m_s=0.10,
        c_sync_vision_max_accel_m_s2=0.25,
        c_sync_vision_filter_window_s=0.20,
        c_sync_vision_control_period_s=0.08,
        c_sync_vision_loss_grace_s=0.20,
    )


def build_stationary_platform_retakeoff_test_config(
    base: Task2Config,
    *,
    point_x_m: float = 0.0,
    point_y_m: float = 0.50,
    retakeoff_height_m: float = 0.80,
    skip_vision: bool = False,
    land_only: bool = False,
) -> Task2Config:
    """正式控制链的低高度静止平台降落/直接复升联调配置。"""
    point = (float(point_x_m), float(point_y_m))
    retakeoff_height = float(retakeoff_height_m)
    if not all(math.isfinite(value) for value in (*point, retakeoff_height)):
        raise ValueError("静止平台测试坐标和复升高度必须为有限数值")
    distance = math.hypot(*point)
    if not 0.30 <= distance <= 1.00:
        raise ValueError("静止平台测试点距H点必须在0.30~1.00m之间")
    if not 0.60 <= retakeoff_height <= 1.00:
        raise ValueError("静止平台测试复升高度必须在0.60~1.00m之间")
    return replace(
        base,
        cruise_height_m=1.00,
        follow_height_m=1.00,
        hold_duration_s=2.0,
        intercept_speed_m_s=0.10,
        hold_position_max_speed_m_s=0.22,
        hold_velocity_kd=0.45,
        vision_min_quality=40,
        vision_confirm_frames=5,
        drop_max_error_m=0.15,
        retakeoff_height_m=retakeoff_height,
        abort_climb_height_m=1.00,
        require_car_position_at_c=False,
        safe_open_loop_cd_test=False,
        stationary_retakeoff_test=True,
        stationary_skip_vision=bool(skip_vision),
        land_only_after_touchdown=bool(land_only),
        stationary_test_point_x_m=point[0],
        stationary_test_point_y_m=point[1],
    )


def wait_for_task2_start(
    *,
    local_button_start: bool,
    start_gate: Task2StartGate | None,
    start_timeout: float | None,
) -> tuple[bool, int | None]:
    """等待本地按键或小车CAR_START；失败时一律关闭门禁。"""
    if local_button_start:
        return bool(wait_for_start_button()), None
    if start_gate is None:
        return False, None
    session_id = start_gate.wait(start_timeout)
    return session_id is not None, session_id


def _load_landing_config(data: dict) -> LandingConfig:
    landing = data.get("landing", {})
    terminal = data.get("terminal_landing", {})
    touchdown = data.get("touchdown", {})
    return LandingConfig(
        mid_height_m=float(landing.get("mid_height_m", 0.80)),
        low_height_m=float(landing.get("low_height_m", 0.35)),
        visual_min_height_m=float(landing.get("visual_min_height_m", 0.16)),
        contact_height_m=float(landing.get("contact_height_m", 0.10)),
        descend_high_m_s=float(landing.get("descend_high_m_s", 0.30)),
        descend_mid_m_s=float(landing.get("descend_mid_m_s", 0.20)),
        descend_low_m_s=float(landing.get("descend_low_m_s", 0.08)),
        descend_slew_m_s2=float(
            landing.get("descend_slew_m_s2", 0.40)
        ),
        reacquire_climb_m_s=float(landing.get("reacquire_climb_m_s", 0.12)),
        terminal_max_s=float(terminal.get("max_duration_s", 0.50)),
        terminal_max_drop_m=float(terminal.get("max_drop_m", 0.10)),
        terminal_max_error_m=float(
            terminal.get("max_position_error_m", 0.10)
        ),
        terminal_max_relative_speed_m_s=float(
            terminal.get("max_relative_speed_m_s", 0.10)
        ),
        terminal_max_uncertainty_m=float(
            terminal.get("max_uncertainty_m", 0.08)
        ),
        touchdown_height_margin_m=float(
            touchdown.get("height_margin_m", 0.03)
        ),
        touchdown_max_vz_m_s=float(
            touchdown.get("max_vertical_speed_m_s", 0.08)
        ),
        touchdown_max_relative_speed_m_s=float(
            touchdown.get("max_relative_speed_m_s", 0.10)
        ),
        touchdown_max_tilt_deg=float(touchdown.get("max_tilt_deg", 8.0)),
        touchdown_hold_s=float(touchdown.get("hold_s", 0.40)),
        deck_ride_s=float(landing.get("deck_ride_s", 5.0)),
        max_terminal_retries=int(terminal.get("max_retries", 1)),
    )


def _load_formation_config(data: dict) -> FormationConfig:
    form = data.get("formation", {})
    return FormationConfig(
        kp=float(form.get("kp", 0.75)),
        kd=float(form.get("kd", 0.22)),
        max_speed_m_s=float(form.get("max_speed_m_s", 0.40)),
        max_accel_m_s2=float(form.get("max_accel_m_s2", 0.45)),
        max_jerk_m_s3=float(form.get("max_jerk_m_s3", 1.2)),
        position_deadband_m=float(form.get("position_deadband_m", 0.03)),
    )


def _build_telemetry_sample(mission_obj) -> Task2TelemetrySample | None:
    return Task2TelemetrySample(
        phase=mission_obj.director.phase,
        base_state=str(mission_obj.state),
        position_xyz_m=tuple(
            float(v) for v in mission_obj.last_world_position
        ),
        landing_state=mission_obj.last_landing_state,
        mission_success=mission_obj.director.mission_success,
    )


def validate_task2_start_modes(
    *,
    open_loop_cd_test: bool,
    vision_landing_test: bool = False,
    stationary_retakeoff_test: bool,
    local_button_start: bool,
    stationary_skip_vision: bool = False,
    stationary_land_only: bool = False,
) -> None:
    if sum(
        bool(value)
        for value in (
            open_loop_cd_test,
            vision_landing_test,
            stationary_retakeoff_test,
        )
    ) > 1:
        raise ValueError(
            "--open-loop-cd-test、--vision-landing-test 与 "
            "--stationary-platform-retakeoff-test 只能选择一个"
        )
    if local_button_start and not stationary_retakeoff_test:
        raise ValueError(
            "--local-button-start 只能与 --stationary-platform-retakeoff-test 同时使用"
        )
    if stationary_skip_vision and not stationary_retakeoff_test:
        raise ValueError(
            "--stationary-skip-vision 只能与 --stationary-platform-retakeoff-test 同时使用"
        )
    if stationary_land_only and not stationary_retakeoff_test:
        raise ValueError(
            "--stationary-land-only 只能与 --stationary-platform-retakeoff-test 同时使用"
        )


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    parser.add_argument(
        "--start-timeout",
        type=float,
        default=None,
        help="等待CAR_START的秒数；默认无限等待",
    )
    parser.add_argument(
        "--open-loop-cd-test",
        action="store_true",
        help=(
            "任务二安全联调：1.3m直飞C，视觉居中后恒速飞向D并下降，"
            "最低0.5m悬停，不触地、不锁桨"
        ),
    )
    parser.add_argument(
        "--vision-landing-test",
        action="store_true",
        help=(
            "任务二联合降落测试：直接飞往C点等待小车，视觉连续居中后动态纠偏下降，"
            "触地停留且不复升"
        ),
    )
    parser.add_argument(
        "--cd-speed",
        type=float,
        default=0.06,
        help="安全联调中C-D固定速度，单位m/s，默认0.06",
    )
    parser.add_argument(
        "--stationary-platform-retakeoff-test",
        action="store_true",
        help=(
            "正式控制链静止平台联调：1.0m起飞后直达近距离平台，"
            "动态降落、保持5秒、直接复升至0.8m并定点悬停"
        ),
    )
    parser.add_argument(
        "--test-point-x",
        type=float,
        default=0.0,
        help="静止平台测试点T265 X坐标，默认0.0m",
    )
    parser.add_argument(
        "--test-point-y",
        type=float,
        default=0.50,
        help="静止平台测试点T265 Y坐标，默认0.50m",
    )
    parser.add_argument(
        "--test-retakeoff-height",
        type=float,
        default=0.80,
        help="静止平台测试直接复升高度，默认0.80m，范围0.60~1.00m",
    )
    parser.add_argument(
        "--local-button-start",
        action="store_true",
        help=(
            "仅静止平台复升联调：使用无人机BCM5一键起飞按钮，"
            "跳过蓝牙和小车CAR_START"
        ),
    )
    parser.add_argument(
        "--stationary-skip-vision",
        action="store_true",
        help=(
            "仅静止平台复升联调：跳过视觉门禁，使用固定T265测试点执行降落和复升"
        ),
    )
    parser.add_argument(
        "--stationary-land-only",
        action="store_true",
        help=(
            "仅静止平台联调：触地稳定并停留5秒后保持低高度目标，不执行复升"
        ),
    )
    args = parser.parse_args(argv)

    try:
        validate_task2_start_modes(
            open_loop_cd_test=args.open_loop_cd_test,
            vision_landing_test=args.vision_landing_test,
            stationary_retakeoff_test=args.stationary_platform_retakeoff_test,
            local_button_start=args.local_button_start,
            stationary_skip_vision=args.stationary_skip_vision,
            stationary_land_only=args.stationary_land_only,
        )
    except ValueError as exc:
        parser.error(str(exc))

    raw_config = args.config.read_bytes()
    data = json.loads(raw_config.decode("utf-8"))
    config_hash = zlib.crc32(raw_config) & 0xFFFFFFFF
    task_config = _load_task_config(data)
    if args.open_loop_cd_test:
        try:
            task_config = build_open_loop_cd_test_config(
                task_config,
                cd_speed_m_s=args.cd_speed,
            )
        except ValueError as exc:
            parser.error(str(exc))
    elif args.vision_landing_test:
        task_config = build_vision_landing_test_config(task_config)
    elif args.stationary_platform_retakeoff_test:
        try:
            task_config = build_stationary_platform_retakeoff_test_config(
                task_config,
                point_x_m=args.test_point_x,
                point_y_m=args.test_point_y,
                retakeoff_height_m=args.test_retakeoff_height,
                skip_vision=args.stationary_skip_vision,
                land_only=args.stationary_land_only,
            )
        except ValueError as exc:
            parser.error(str(exc))
    landing_config = _load_landing_config(data)
    formation_config = _load_formation_config(data)
    bluetooth = data["bluetooth"]
    cybercam = data["cybercam"]
    square = data["static_square_test"]
    payload_cfg = data.get("payload_servo", {})
    dry_run = os.getenv("DRONE_DRY_RUN", "0").strip().lower() in {
        "1",
        "true",
    }
    if args.open_loop_cd_test:
        logger.warning(
            "任务二0.5m安全联调：起飞高度=1.30m，跳过B_PRE/B-C，直飞C；"
            "C点等待坐标X向小值方向修正0.15m；"
            "C点等待坐标Y向大值方向修正0.25m；"
            "C点视觉中心误差<=0.15m连续5帧后，"
            f"以{task_config.car_speed_m_s:.3f}m/s沿C-D飞行并按路径进度下降；"
            f"目标下降速率约{task_config.safe_descent_rate_m_s:.3f}m/s；"
            f"C点放行后{task_config.safe_follow_cutoff_s:.1f}s强制停止伴随，"
            "到达0.50m附近立即停止水平前进并定点悬停；"
            "绝不自动降落、锁桨或重新起飞"
        )
        logger.warning(
            "安全提示：悬空时不要按Ctrl+C；请先用遥控器接管并安全落地，"
            "确认电机停稳后再终止程序"
        )
    elif args.vision_landing_test:
        logger.warning(
            "任务二视觉动态降落联合测试：起飞至1.30m后经两个中间点飞往C点等待小车；"
            "检测到目标后以10cm/s限速、12.5Hz控制频率主动视觉居中；"
            "目标误差不超过15cm连续5帧后，以0.30m/s持续纠偏下降；"
            "视觉失效后以相同速度开环下降，不暂停、不爬升且不使用失效观测追偏；"
            "触地候选后持续下压0cm高度目标；激光低于0.10m且稳定0.8秒后，"
            "保持task_sta=1并发送"
            "next_task_sign=102，由飞控近地门禁直接锁桨；锁桨保持5秒后重新起飞，"
            "保持T265连续运行；以复飞锚点为局部(0,0)，"
            "复飞爬升及稳定阶段以5cm/s沿-Y前进；"
            "复飞后T265跳变不触发悬停，改为连续坐标补偿；"
            f"局部H=({task_config.retakeoff_h_offset_x_m:+.2f},"
            f"{task_config.retakeoff_h_offset_y_m:+.2f})m，"
            "视觉跟随小车回A最长30秒，超时后从当时位置直接飞往局部H并完成最终降落；"
            "返H超过15秒或最终接近超过8秒则在当前位置使用实时帧强制下降，"
            "复飞后HOVER_WAIT超过3秒同样使用实时帧下降；"
            "所有最终降落均在近地后使用102直接锁桨，不调用一键降落；"
            "复飞若发生一次物理不可能的重定位跳变则做坐标连续性补偿；"
            "平台降落阶段不调用OneKey_Land"
        )
        logger.warning(
            "安全提示：该模式会真实降落到移动平台；测试前确认小车平台平整、"
            "C-D速度较慢、遥控器可随时接管"
        )
    elif args.stationary_platform_retakeoff_test:
        vision_gate_text = (
            "跳过视觉检测，抵达固定T265测试点后直接进入动态降落"
            if task_config.stationary_skip_vision
            else "视觉居中连续5帧后动态降落"
        )
        touchdown_action_text = (
            "触地稳定并保持5秒后维持5cm低高度目标，不执行复升"
            if task_config.land_only_after_touchdown
            else (
                "触地保持解锁5秒，不二次解锁直接复升至"
                f"{task_config.retakeoff_height_m:.2f}m并定点悬停"
            )
        )
        logger.warning(
            "任务二静止平台复升联调：复用正式起飞/T265/动态降落控制链；"
            f"请将固定平台中心放在H点相对坐标"
            f"({task_config.stationary_test_point_x_m:+.2f},"
            f"{task_config.stationary_test_point_y_m:+.2f})m；"
            "起飞至1.00m并悬停2秒后，以0.10m/s直达平台；"
            f"{vision_gate_text}，{touchdown_action_text}"
        )
        if task_config.land_only_after_touchdown:
            logger.warning(
                "静止平台只降落模式：触地稳定并保持5秒后不执行复升；"
                "程序维持5cm低高度目标，等待人工确认后安全停止"
            )
        if task_config.land_only_after_touchdown:
            logger.warning(
                "安全提示：平台必须固定、平整且周围无人；触地后先用遥控器停止电机，"
                "确认电机停稳后才能按Ctrl+C"
            )
        else:
            logger.warning(
                "安全提示：平台必须固定、平整且周围无人；复升悬停后必须先用遥控器"
                "接管并安全落地，确认电机停稳后才能按Ctrl+C"
            )
        if task_config.stationary_skip_vision:
            logger.warning(
                "无视觉静止平台测试：下降过程中不做图像纠偏；"
                "平台中心必须准确固定在上述T265坐标，位置不符会导致落空"
            )

    actuator, payload_hardware = build_payload_actuator(
        release_hold_s=float(payload_cfg.get("release_hold_s", 1.0))
    )
    if not payload_hardware.ready:
        raise RuntimeError("投放舵机未能锁定到180°，禁止进入等待启动状态")

    link = None
    if not args.local_button_start:
        link = AirGroundLink(
            LinkConfig(
                port=str(bluetooth["port"]),
                baudrate=int(bluetooth["baudrate"]),
                write_timeout_s=float(
                    bluetooth.get("write_timeout_s", 0.20)
                ),
                ack_timeout_s=float(bluetooth["ack_timeout_s"]),
                max_retries=int(bluetooth["max_retries"]),
                max_consecutive_tx_errors=int(
                    bluetooth.get("max_consecutive_tx_errors", 3)
                ),
            )
        )
        if not link.start():
            raise RuntimeError("蓝牙链路启动失败")

    reader = None
    serial_fc = None
    mission_obj = None
    telemetry = None
    telemetry_finished = False
    warning_led_active = False
    try:
        if task_config.stationary_skip_vision:
            reader = NullVisionReader()
            reader.start()
            vision_ready = True
            logger.warning(
                "静止平台无视觉测试：跳过Cyber Camera启动和双向通信预检"
            )
        else:
            reader = CyberCamReader(
                port=str(cybercam["port"]),
                baudrate=int(cybercam["baudrate"]),
            )
            vision_started = reader.start()
            vision_ready = (
                vision_started and _wait_for_vision(reader)
                if not dry_run
                else False
            )
            if not dry_run and not vision_ready:
                raise RuntimeError("Cyber Camera双向通信预检失败，禁止起飞")
            if vision_ready:
                logger.info("Cyber Camera VS1与PING/PONG预检通过")
            else:
                logger.warning(
                    "DRONE_DRY_RUN桌面通信测试：跳过Cyber Camera双向预检"
                )

        realsense = t265_class()
        re_fc = [0] * 14
        se_fc = [
            170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255
        ]
        port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(
            se_fc, realsense, vel_freq=100, cmd_freq=50
        )
        if not _wait_for_fc():
            raise RuntimeError("飞控串口3秒内未收到遥测，禁止进入等待启动状态")

        start_gate = None
        if link is not None:
            start_gate = Task2StartGate(
                link,
                config_hash=config_hash,
                ready_bits=(
                    READY_BT
                    | READY_PAYLOAD_LOCKED
                    | READY_VISION
                    | READY_FC_LINK
                ),
                state_max_age_s=float(
                    bluetooth.get("car_state_max_age_s", 0.30)
                ),
                state_max_speed_m_s=float(
                    bluetooth.get("car_speed_max_m_s", 0.30)
                ),
                state_max_component_m_s=float(
                    bluetooth.get("car_component_max_m_s", 0.40)
                ),
            )

        def _car_position_provider():
            if start_gate is None:
                return None
            pos = start_gate.latest_car_position()
            return pos.position_xy_m if pos else None

        mission_obj = Task2FlightMission(
            re_fc,
            se_fc,
            realsense,
            serial_fc,
            vision_reader=reader,
            payload_actuator=actuator,
            task_config=task_config,
            formation_config=formation_config,
            landing_config=landing_config,
            vision_config={
                "max_age_s": float(cybercam["max_age_s"]),
                "min_quality": int(task_config.vision_min_quality),
                "image_center_px": tuple(square["image_center_px"]),
                "focal_px": tuple(square["focal_px"]),
            },
            car_speed_provider=(
                start_gate.car_speed if start_gate is not None else None
            ),
            car_position_provider=_car_position_provider,
            car_velocity_provider=(
                start_gate.car_velocity if start_gate is not None else None
            ),
        )

        if args.local_button_start:
            logger.info(
                "静止平台复升联调本地按键模式：不等待蓝牙或小车CAR_START"
            )
        else:
            if not set_rgb_led("B"):
                raise RuntimeError("等待启动蓝色状态灯点亮失败")
            warning_led_active = True
            if args.stationary_platform_retakeoff_test:
                logger.info(
                    "蓝灯：静止平台复升联调已就绪；确认平台固定且车辆保持静止后，"
                    "按小车任务二按键发送CAR_START。T265尚未初始化"
                )
            else:
                logger.info(
                    "蓝灯：蓝牙、视觉、飞控和舵机均已就绪；"
                    "等待小车按键发送CAR_START。T265尚未初始化；"
                    "任务二将在C点激活T265坐标伴飞与动态降落"
                )
        start_accepted, session_id = wait_for_task2_start(
            local_button_start=args.local_button_start,
            start_gate=start_gate,
            start_timeout=args.start_timeout,
        )
        if not start_accepted:
            logger.error(
                "本地一键起飞门禁失败，任务取消"
                if args.local_button_start
                else "等待CAR_START超时，任务取消"
            )
            return
        if args.local_button_start:
            logger.info("无人机一键起飞按钮已确认；开始初始化T265并准备起飞")
        else:
            logger.info(
                f"收到任务二CAR_START，session={session_id}，"
                f"car_config_hash=0x{start_gate.car_config_hash:08X}；"
                "解除T265拔插阻塞并开始初始化"
            )
        if not set_rgb_led("R"):
            raise RuntimeError("启动门禁通过后红色起飞警示灯点亮失败")
        warning_led_active = True
        mission_obj.preflight_warning_started_at = time.monotonic()
        if link is not None and session_id is not None:
            telemetry_hz = max(
                1.0, float(bluetooth.get("uav_state_hz", 10.0))
            )
            telemetry = Task2TelemetryPublisher(
                link,
                session_id=session_id,
                state_interval_s=1.0 / telemetry_hz,
            )
            if not link.is_running:
                raise RuntimeError("蓝牙链路已在起飞预检前中断，取消任务")
        mission_obj.start()
        if not mission_obj.task_running:
            raise RuntimeError("T265或起飞预检失败，任务未启动")
        if link is not None and not link.is_running:
            raise RuntimeError("蓝牙链路在起飞预检期间中断，紧急停止任务")
        warning_led_active = False
        while mission_obj.task_running:
            sample = _build_telemetry_sample(mission_obj)
            if sample is not None and telemetry is not None:
                telemetry.update(sample)
            time.sleep(0.03)
        if telemetry is not None:
            telemetry.finish(
                mission_success=mission_obj.director.mission_success,
                faulted=bool(mission_obj.emergency_stop),
            )
            telemetry_finished = True
            link.wait_pending(0.65)
    except KeyboardInterrupt:
        logger.warning("用户中断任务二启动程序")
        _stop_mission_safely(mission_obj)
    except Exception:
        _stop_mission_safely(mission_obj)
        raise
    finally:
        _stop_mission_safely(mission_obj)
        if telemetry is not None and not telemetry_finished:
            mission_success = bool(
                mission_obj is not None
                and mission_obj.director.mission_success
            )
            telemetry.finish(
                mission_success=mission_success,
                faulted=True,
            )
            if link is not None:
                link.wait_pending(0.65)
        if warning_led_active:
            try:
                set_rgb_led("OFF")
            except Exception:
                pass
        if reader is not None:
            reader.close()
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()
