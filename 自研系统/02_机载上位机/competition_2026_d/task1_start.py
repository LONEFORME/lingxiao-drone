"""任务一正式一键启动入口。

启动顺序：
1. 锁定投放舵机并打开蓝牙；
2. 周期广播UAV_READY，等待小车按键产生的CAR_START；
3. 收到有效CAR_START后才启动Cyber Camera、构造T265并打开飞控串口；
4. T265严格预检通过后自动起飞，全程不等待终端人工确认。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import zlib
from dataclasses import dataclass, replace
from pathlib import Path

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.gpio_led import set_rgb_led  # noqa: E402
from Lcode.global_variable import fc_last_rx_time, sp_side  # noqa: E402
from t265 import t265_class  # noqa: E402

from shared.competition_2026_d_protocol import (  # noqa: E402
    CarSegment,
    CarStateFlag,
    Device,
    Flag,
    MessageType,
    PositionFlag,
    pack_payload,
    seq_is_newer,
    unpack_car_state,
    unpack_payload,
)

from .Lcode.air_ground_link import AirGroundLink, LinkConfig  # noqa: E402
from .payload_servo import build_payload_actuator  # noqa: E402
from .task1_flight import Task1FlightMission  # noqa: E402
from .task1_mission import Task1Config  # noqa: E402
from .task1_telemetry import (  # noqa: E402
    Task1TelemetryPublisher,
    Task1TelemetrySample,
)
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


TASK1_MODE = 1
TASK1_MASK = 1 << 0
READY_BT = 1 << 0
READY_PAYLOAD_LOCKED = 1 << 1
READY_VISION = 1 << 2
READY_FC_LINK = 1 << 3


@dataclass(frozen=True)
class CarStateSnapshot:
    session_id: int
    seq: int
    sender_ms: int
    received_monotonic: float
    segment: int
    track_s_mm: int
    speed_m_s: float
    heading_deg: float
    state_flags: int
    world_velocity_m_s: tuple[float, float] | None
    legacy_payload: bool

    @property
    def encoder_speed_valid(self) -> bool:
        return bool(
            self.state_flags & int(CarStateFlag.ENCODER_SPEED_VALID)
        )


@dataclass(frozen=True)
class CarPositionSnapshot:
    session_id: int
    seq: int
    sender_ms: int
    received_monotonic: float
    position_xy_m: tuple[float, float]
    pose_age_s: float
    flags: int

    @property
    def pose_valid(self) -> bool:
        return bool(self.flags & int(PositionFlag.CAR_POSE_VALID))

    @property
    def pose_fresh(self) -> bool:
        return bool(self.flags & int(PositionFlag.CAR_POSE_FRESH))


class Task1StartGate:
    """只接受来自小车、任务模式为1、session非0的CAR_START。"""

    def __init__(
        self,
        link: AirGroundLink,
        *,
        config_hash: int,
        ready_bits: int = READY_BT | READY_PAYLOAD_LOCKED,
        ready_interval_s: float = 0.5,
        state_max_age_s: float = 0.30,
        state_max_speed_m_s: float = 0.30,
        state_max_component_m_s: float = 0.40,
        clock=time.monotonic,
    ) -> None:
        self.link = link
        self.config_hash = int(config_hash) & 0xFFFFFFFF
        self.ready_bits = int(ready_bits) & 0xFFFF
        self.ready_interval_s = max(0.1, float(ready_interval_s))
        self.state_max_age_s = max(0.1, float(state_max_age_s))
        self.state_max_speed_m_s = max(0.01, float(state_max_speed_m_s))
        self.state_max_component_m_s = max(
            self.state_max_speed_m_s,
            float(state_max_component_m_s),
        )
        self.clock = clock
        self._start_event = threading.Event()
        self._lock = threading.Lock()
        self._session_id: int | None = None
        self._car_config_hash: int | None = None
        self._car_state: CarStateSnapshot | None = None
        self._car_position: CarPositionSnapshot | None = None
        self._last_car_state_seq: int | None = None
        self._last_car_position_seq: int | None = None
        self.rejected_start_frames = 0
        self.rejected_state_frames = 0
        self.duplicate_or_old_state_frames = 0
        self.legacy_state_frames = 0
        self.rejected_position_frames = 0
        self.duplicate_or_old_position_frames = 0
        self.link.add_callback(self._on_frame)

    @property
    def session_id(self) -> int | None:
        with self._lock:
            return self._session_id

    @property
    def car_config_hash(self) -> int | None:
        with self._lock:
            return self._car_config_hash

    def wait(self, timeout_s: float | None = None) -> int | None:
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
                            TASK1_MASK,
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

    def car_speed(self) -> float | None:
        state = self.latest_car_state()
        if state is None or not state.encoder_speed_valid:
            return None
        if not 0.0 <= state.speed_m_s <= self.state_max_speed_m_s:
            return None
        return state.speed_m_s

    def car_velocity(self) -> tuple[float, float] | None:
        state = self.latest_car_state()
        if state is None or not state.encoder_speed_valid:
            return None
        return state.world_velocity_m_s

    def latest_car_state(self) -> CarStateSnapshot | None:
        with self._lock:
            state = self._car_state
        if state is None:
            return None
        age = self.clock() - state.received_monotonic
        if age < 0.0 or age > self.state_max_age_s:
            return None
        return state

    def latest_car_position(self) -> CarPositionSnapshot | None:
        with self._lock:
            position = self._car_position
        if position is None:
            return None
        age = self.clock() - position.received_monotonic
        if age < 0.0 or age > self.state_max_age_s:
            return None
        if not position.pose_valid or not position.pose_fresh:
            return None
        return position

    def _on_frame(self, frame) -> None:
        if (
            int(frame.source) != int(Device.CAR)
            or int(frame.dest) not in (int(Device.UAV), int(Device.BROADCAST))
        ):
            return
        if int(frame.message_type) == int(MessageType.CAR_START):
            self._handle_start(frame)
        elif int(frame.message_type) == int(MessageType.CAR_STATE):
            self._handle_car_state(frame)
        elif int(frame.message_type) == int(MessageType.CAR_POSITION):
            self._handle_car_position(frame)

    def _handle_start(self, frame) -> None:
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
        if task_mode != TASK1_MODE or int(frame.session_id) == 0:
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
                ack_frame_seq = self.link.acknowledge(frame, result=result)
                logger.info(
                    "重复CAR_START已幂等回复ACK: "
                    f"ack_frame_seq={ack_frame_seq}, "
                    f"acked_seq={int(frame.seq)}, "
                    f"session={int(frame.session_id)}, result={result}"
                )
                return
            self._session_id = int(frame.session_id)
            self._car_config_hash = int(car_config_hash)
            self._car_position = None
            self._last_car_position_seq = None
        ack_frame_seq = self.link.acknowledge(frame, result=0)
        logger.info(
            "CAR_START ACK已入发送队列: "
            f"ack_frame_seq={ack_frame_seq}, "
            f"acked_type=0x{int(frame.message_type):02X}, "
            f"acked_seq={int(frame.seq)}, "
            f"session={int(frame.session_id)}, result=0"
        )
        self._start_event.set()

    def _handle_car_state(self, frame) -> None:
        with self._lock:
            session_id = self._session_id
            previous_seq = self._last_car_state_seq
        if session_id is None or int(frame.session_id) != session_id:
            self.rejected_state_frames += 1
            return
        if previous_seq is not None and not seq_is_newer(
            int(frame.seq), previous_seq
        ):
            self.duplicate_or_old_state_frames += 1
            return
        try:
            payload = unpack_car_state(frame.payload)
        except ValueError:
            self.rejected_state_frames += 1
            return
        try:
            CarSegment(payload.segment)
        except ValueError:
            self.rejected_state_frames += 1
            return
        if int(payload.state_flags) & ~0x001F:
            self.rejected_state_frames += 1
            return

        speed_m_s = float(payload.speed_mm_s) / 1000.0
        if abs(speed_m_s) > self.state_max_speed_m_s:
            self.rejected_state_frames += 1
            return

        car_velocity = None
        if payload.has_world_velocity:
            car_velocity = (
                float(payload.vx_mm_s) / 1000.0,
                float(payload.vy_mm_s) / 1000.0,
            )
            if any(
                abs(component) > self.state_max_component_m_s
                for component in car_velocity
            ):
                self.rejected_state_frames += 1
                return
        else:
            self.legacy_state_frames += 1

        received = self.clock()
        snapshot = CarStateSnapshot(
            session_id=int(frame.session_id),
            seq=int(frame.seq),
            sender_ms=int(frame.sender_ms),
            received_monotonic=received,
            segment=int(payload.segment),
            track_s_mm=int(payload.track_s_mm),
            speed_m_s=speed_m_s,
            heading_deg=float(payload.heading_cdeg) / 100.0,
            state_flags=int(payload.state_flags),
            world_velocity_m_s=car_velocity,
            legacy_payload=not payload.has_world_velocity,
        )
        with self._lock:
            self._car_state = snapshot
            self._last_car_state_seq = int(frame.seq)

    def _handle_car_position(self, frame) -> None:
        with self._lock:
            session_id = self._session_id
            previous_seq = self._last_car_position_seq
        expected_session = 0 if session_id is None else session_id
        if int(frame.session_id) != expected_session:
            self.rejected_position_frames += 1
            return
        if previous_seq is not None and not seq_is_newer(
            int(frame.seq), previous_seq
        ):
            self.duplicate_or_old_position_frames += 1
            return
        try:
            x_mm, y_mm, pose_age_ms, flags = unpack_payload(
                MessageType.CAR_POSITION, frame.payload
            )
        except ValueError:
            self.rejected_position_frames += 1
            return
        if int(flags) & ~0x000F:
            self.rejected_position_frames += 1
            return
        session_flag = bool(int(flags) & int(PositionFlag.SESSION_VALID))
        if session_flag != (int(frame.session_id) != 0):
            self.rejected_position_frames += 1
            return
        snapshot = CarPositionSnapshot(
            session_id=int(frame.session_id),
            seq=int(frame.seq),
            sender_ms=int(frame.sender_ms),
            received_monotonic=self.clock(),
            position_xy_m=(float(x_mm) / 1000.0, float(y_mm) / 1000.0),
            pose_age_s=float(pose_age_ms) / 1000.0,
            flags=int(flags),
        )
        with self._lock:
            self._car_position = snapshot
            self._last_car_position_seq = int(frame.seq)


def _load_task_config(data: dict) -> Task1Config:
    task = data["task1"]
    return Task1Config(
        cruise_height_m=float(task["cruise_height_m"]),
        follow_height_m=float(task["follow_height_m"]),
        drop_height_m=float(task["drop_height_m"]),
        final_height_m=float(task["final_height_m"]),
        hold_duration_s=float(task["hover_duration_s"]),
        takeoff_ascent_slew_m_s=float(
            task.get("takeoff_ascent_slew_m_s", 0.40)
        ),
        intercept_speed_m_s=float(task["intercept_speed_m_s"]),
        car_speed_m_s=float(task["car_speed_m_s"]),
        curve_speed_m_s=(
            None
            if task.get("curve_speed_m_s") is None
            else float(task["curve_speed_m_s"])
        ),
        car_speed_scale=float(task.get("car_speed_scale", 1.0)),
        return_speed_m_s=float(task["return_speed_m_s"]),
        point_arrival_radius_m=float(task["path_capture_radius_m"]),
        hold_position_max_speed_m_s=float(
            task.get("hold_position_max_speed_m_s", 0.15)
        ),
        hold_stable_speed_m_s=float(
            task.get("hold_stable_speed_m_s", 0.12)
        ),
        final_landing_radius_m=float(
            task.get("final_landing_radius_m", 0.08)
        ),
        final_landing_max_speed_m_s=float(
            task.get("final_landing_max_speed_m_s", 0.05)
        ),
        final_landing_stable_s=float(
            task.get("final_landing_stable_s", 0.50)
        ),
        final_descend_horizontal_max_speed_m_s=float(
            task.get("final_descend_horizontal_max_speed_m_s", 0.06)
        ),
        acquire_timeout_s=float(task.get("acquire_timeout_s", 0.0)),
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
        intercept_vision_confirm_frames=int(
            task.get("intercept_vision_confirm_frames", 5)
        ),
        intercept_vision_max_error_m=float(
            task.get("intercept_vision_max_error_m", 0.35)
        ),
        acquire_max_error_m=float(
            task.get("acquire_max_error_m", task["drop_max_error_m"])
        ),
        drop_max_error_m=float(task["drop_max_error_m"]),
        drop_confirm_duration_s=float(
            task.get("drop_confirm_duration_s", 0.0)
        ),
        min_follow_before_drop_s=float(
            task.get("min_follow_before_drop_s", 0.0)
        ),
        vision_trim_kp=float(task.get("vision_trim_kp", 0.0)),
        vision_trim_deadband_m=float(
            task.get("vision_trim_deadband_m", 0.0)
        ),
        vision_trim_max_speed_m_s=float(
            task.get("vision_trim_max_speed_m_s", 0.0)
        ),
        vision_trim_max_accel_m_s2=float(
            task.get("vision_trim_max_accel_m_s2", 0.0)
        ),
        acquire_vision_min_quality=int(
            task.get("acquire_vision_min_quality", 40)
        ),
        acquire_vision_kp=float(task.get("acquire_vision_kp", 0.45)),
        acquire_vision_deadband_m=float(
            task.get("acquire_vision_deadband_m", 0.04)
        ),
        acquire_vision_max_speed_m_s=float(
            task.get("acquire_vision_max_speed_m_s", 0.12)
        ),
        acquire_vision_max_accel_m_s2=float(
            task.get("acquire_vision_max_accel_m_s2", 0.25)
        ),
        acquire_vision_control_period_s=float(
            task.get("acquire_vision_control_period_s", 0.20)
        ),
        acquire_vision_filter_window_s=float(
            task.get("acquire_vision_filter_window_s", 0.60)
        ),
        acquire_vision_loss_grace_s=float(
            task.get("acquire_vision_loss_grace_s", 0.30)
        ),
        vision_takeover_max_speed_m_s=float(
            task.get("vision_takeover_max_speed_m_s", 0.08)
        ),
        intercept_vision_early_stop_enabled=bool(
            task.get("intercept_vision_early_stop_enabled", True)
        ),
        drop_during_bc_enabled=bool(
            task.get("drop_during_bc_enabled", False)
        ),
        drop_at_follow_height=bool(
            task.get("drop_at_follow_height", False)
        ),
        drop_descent_speed_m_s=float(task["drop_descent_speed_m_s"]),
        drop_time_margin_s=float(task["drop_time_margin_s"]),
        release_timeout_s=float(task["release_timeout_s"]),
    )


def build_joint_drop_test_config(base: Task1Config) -> Task1Config:
    """Pure-vision joint test: hover 3 s, then track and release at 1.2 m."""
    return replace(
        base,
        cruise_height_m=1.20,
        follow_height_m=1.20,
        hold_duration_s=3.0,
        intercept_speed_m_s=0.20,
        car_speed_m_s=0.05,
        curve_speed_m_s=0.05,
        drop_max_error_m=0.20,
        drop_confirm_duration_s=3.0,
        payload_drop_enabled=True,
        drop_during_bc_enabled=True,
        drop_at_follow_height=True,
        vision_track_only=True,
    )


def build_joint_path_drop_test_config(
    base: Task1Config,
    *,
    follow_speed_m_s: float = 0.060,
    follow_duration_s: float = 3.0,
) -> Task1Config:
    """Stable 1.2 m joint test using a constant T265 path speed."""
    speed = float(follow_speed_m_s)
    duration = float(follow_duration_s)
    if not 0.02 <= speed <= 0.20:
        raise ValueError("固定路径伴飞速度必须在0.02~0.20m/s之间")
    if not 0.0 <= duration <= 20.0:
        raise ValueError("投放前伴飞时间必须在0~20s之间")
    return replace(
        base,
        cruise_height_m=1.20,
        follow_height_m=1.20,
        car_speed_m_s=speed,
        curve_speed_m_s=speed,
        vision_min_quality=40,
        vision_confirm_frames=3,
        acquire_max_error_m=0.30,
        drop_max_error_m=0.30,
        drop_confirm_duration_s=0.20,
        min_follow_before_drop_s=duration,
        vision_trim_kp=0.0,
        vision_trim_max_speed_m_s=0.0,
        vision_trim_max_accel_m_s2=0.0,
        payload_drop_enabled=True,
        drop_during_bc_enabled=True,
        drop_at_follow_height=True,
        vision_track_only=False,
        intercept_vision_early_stop_enabled=False,
    )


def _wait_for_vision(reader: CyberCamReader, timeout_s: float = 45.0) -> bool:
    """Wait through a simultaneous-board boot without weakening preflight."""
    started = time.monotonic()
    deadline = started + max(0.0, float(timeout_s))
    next_diagnostic = started + 5.0
    while time.monotonic() < deadline:
        stats = reader.stats()
        if stats["accepted_frames"] >= 3 and stats["pongs_received"] >= 1:
            return True
        if not stats["running"]:
            logger.warning("Cyber Camera串口接收线程已停止，结束启动等待")
            return False
        now = time.monotonic()
        if now >= next_diagnostic:
            next_diagnostic = now + 5.0
            logger.info(
                "等待Cyber Camera自启动: "
                f"elapsed={now - started:.1f}s, "
                f"accepted_frames={stats['accepted_frames']}, "
                f"pongs_received={stats['pongs_received']}, "
                f"ping_write_errors={stats['ping_write_errors']}"
            )
        time.sleep(0.05)
    return False


def _wait_for_fc(timeout_s: float = 3.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if fc_last_rx_time.value > 0:
            return True
        time.sleep(0.05)
    return False


def _stop_mission_safely(mission_obj, timeout_s: float = 0.8) -> None:
    if mission_obj is None or not mission_obj.task_running:
        return
    mission_obj.emergency()
    deadline = time.monotonic() + timeout_s
    while mission_obj.task_running and time.monotonic() < deadline:
        time.sleep(0.03)
    if mission_obj.task_running:
        mission_obj.stop_all()


def _build_telemetry_sample(
    mission_obj,
    realsense,
) -> Task1TelemetrySample | None:
    try:
        position = tuple(float(v) for v in realsense.get_position())
    except Exception:
        return None
    return Task1TelemetrySample(
        phase=mission_obj.director.phase,
        base_state=str(mission_obj.state),
        position_xyz_m=(position[0], position[1], position[2]),
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
    test_mode = parser.add_mutually_exclusive_group()
    test_mode.add_argument(
        "--joint-drop-test",
        action="store_true",
        help=(
            "联合投放测试：等待小车CAR_START，起飞后悬停3s，1.2m慢速伴飞，"
            "视觉门槛连续满足3s后直接投放"
        ),
    )
    test_mode.add_argument(
        "--joint-path-drop-test",
        action="store_true",
        help=(
            "联合固定路径投放测试：到达B_PRE并确认目标后，"
            "按T265固定路径恒速伴飞一段时间再投放"
        ),
    )
    parser.add_argument(
        "--path-follow-speed",
        type=float,
        default=0.060,
        help="固定路径联合测试的B-C伴飞速度，单位m/s，默认0.060",
    )
    parser.add_argument(
        "--path-follow-duration",
        type=float,
        default=3.0,
        help="固定路径联合测试投放前的最短伴飞时间，默认3.0s",
    )
    args = parser.parse_args(argv)

    raw_config = args.config.read_bytes()
    data = json.loads(raw_config.decode("utf-8"))
    config_hash = zlib.crc32(raw_config) & 0xFFFFFFFF
    task_config = _load_task_config(data)
    if args.joint_drop_test:
        task_config = build_joint_drop_test_config(task_config)
    elif args.joint_path_drop_test:
        try:
            task_config = build_joint_path_drop_test_config(
                task_config,
                follow_speed_m_s=args.path_follow_speed,
                follow_duration_s=args.path_follow_duration,
            )
        except ValueError as exc:
            parser.error(str(exc))
    bluetooth = data["bluetooth"]
    cybercam = data["cybercam"]
    square = data["static_square_test"]
    payload_cfg = data.get("payload_servo", {})
    vision_target_offset_xy_m = (
        float(data["task1"].get("vision_target_offset_x_m", 0.0)),
        float(data["task1"].get("vision_target_offset_y_m", 0.0)),
    )
    if args.joint_drop_test:
        logger.warning(
            "联合投放测试模式：等待小车CAR_START；高度=1.20m，起飞后悬停=3.0s；"
            "中央门槛后由纯视觉持续追踪，不执行B-C固定路径；"
            "视觉偏差<=0.20m连续3.0s后直接投放，禁止下降到0.65m"
        )
    elif args.joint_path_drop_test:
        logger.warning(
            "联合固定路径投放测试：高度=1.20m；必须到达B_PRE后才等待目标；"
            f"目标确认后以{task_config.curve_speed_m_s:.3f}m/s沿B-C恒速伴飞；"
            f"至少伴飞{task_config.min_follow_before_drop_s:.1f}s后，"
            "目标在0.30m投放区域内稳定0.20s即投放；视觉不参与速度控制，"
            "C点保留等待目标的兜底"
        )
    else:
        logger.info(
            "任务一策略：B-C固定路径+限幅视觉微调，1.0m直接投放，"
            "C点等待居中作为兜底；不执行0.65m投放下降"
        )
    logger.info(
        "任务一视觉/T265/控制数据按10Hz写入: "
        f"{Path(__file__).with_name('flight_data.jsonl')}"
    )
    logger.info(
        "任务一视觉投放对准偏置: "
        f"X={vision_target_offset_xy_m[0]:+.2f}m, "
        f"Y={vision_target_offset_xy_m[1]:+.2f}m"
    )
    dry_run = os.getenv("DRONE_DRY_RUN", "0").strip().lower() in {
        "1",
        "true",
    }

    actuator, payload_hardware = build_payload_actuator(
        release_hold_s=float(payload_cfg.get("release_hold_s", 1.0))
    )
    if not payload_hardware.ready:
        raise RuntimeError("投放舵机未能锁定到180°，禁止进入等待启动状态")

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
        # 除T265外的硬件全部在等待小车按键之前完成初始化。
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

        # 这里只构造Python对象，不访问T265硬件。pipeline.start严格放在
        # CAR_START解除阻塞后的mission.start()中。
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
        mission_obj = Task1FlightMission(
            re_fc,
            se_fc,
            realsense,
            serial_fc,
            vision_reader=reader,
            payload_actuator=actuator,
            task_config=task_config,
            vision_config={
                "max_age_s": float(cybercam["max_age_s"]),
                "min_quality": min(
                    int(task_config.vision_min_quality),
                    int(task_config.acquire_vision_min_quality),
                ),
                "image_center_px": tuple(square["image_center_px"]),
                "focal_px": tuple(square["focal_px"]),
                "target_offset_xy_m": vision_target_offset_xy_m,
            },
            car_speed_provider=None,
        )

        if not set_rgb_led("G"):
            raise RuntimeError("等待启动绿色状态灯点亮失败")
        warning_led_active = True
        start_gate = Task1StartGate(
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
        logger.info(
            "绿灯：蓝牙、视觉、飞控和舵机均已就绪；等待小车按键发送CAR_START。"
            "无人机T265尚未初始化；CAR_STATE与CAR_POSITION仅记录诊断，"
            "小车T265不参与任务一飞行控制"
        )
        session_id = start_gate.wait(args.start_timeout)
        if session_id is None:
            logger.error("等待CAR_START超时，任务取消")
            return
        logger.info(
            f"收到任务一CAR_START，session={session_id}，"
            f"car_config_hash=0x{start_gate.car_config_hash:08X}；"
            "解除T265拔插阻塞并开始初始化"
        )
        ack_tx_deadline = time.monotonic() + 0.20
        while (
            link.stats.tx_ack_frames < 1
            and time.monotonic() < ack_tx_deadline
        ):
            time.sleep(0.005)
        if link.stats.tx_ack_frames < 1:
            raise RuntimeError("CAR_START ACK未能写入蓝牙串口，取消任务")
        logger.info(
            "CAR_START ACK已写入蓝牙串口: "
            f"tx_ack_frames={link.stats.tx_ack_frames}, "
            f"raw={link.stats.last_ack_tx_hex}"
        )
        if not set_rgb_led("R"):
            raise RuntimeError("收到CAR_START后红色起飞警示灯点亮失败")
        mission_obj.preflight_warning_started_at = time.monotonic()
        telemetry_hz = max(
            1.0, float(bluetooth.get("uav_state_hz", 10.0))
        )
        telemetry = Task1TelemetryPublisher(
            link,
            session_id=session_id,
            state_interval_s=1.0 / telemetry_hz,
        )

        # T265初始化与红灯5秒安全提示并行；两者都完成后mission.start才解锁。
        if not link.is_running:
            raise RuntimeError("蓝牙链路已在起飞预检前中断，取消任务")
        mission_obj.start()
        if not mission_obj.task_running:
            raise RuntimeError("T265或起飞预检失败，任务未启动")
        if not link.is_running:
            raise RuntimeError("蓝牙链路在起飞预检期间中断，紧急停止任务")
        warning_led_active = False
        while mission_obj.task_running:
            sample = _build_telemetry_sample(
                mission_obj,
                realsense,
            )
            if sample is not None:
                telemetry.update(sample)
            time.sleep(0.03)
        telemetry.finish(
            mission_success=mission_obj.director.mission_success,
            faulted=bool(mission_obj.emergency_stop),
        )
        telemetry_finished = True
        link.wait_pending(0.65)
    except KeyboardInterrupt:
        logger.warning("用户中断任务一启动程序")
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
        link.close()


if __name__ == "__main__":
    main()
