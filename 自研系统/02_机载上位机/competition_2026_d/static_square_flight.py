"""1.5m发现目标、在配置高度执行XY闭环的专项实飞入口。"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path

# basic仍按其原有方式导入顶层Lcode/t265；只在本专用入口补充搜索路径。
BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

# 必须在导入Mission_GPT（其模块常量在导入时读取环境变量）之前冻结末点行为。
os.environ["DRONE_ARRIVAL_HOLD_S"] = "0"
os.environ["DRONE_FINAL_WAYPOINT_HOLD_S"] = "0"

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.global_variable import sp_side  # noqa: E402
from Mission_GPT import mission  # noqa: E402
from main import wait_for_start_button  # noqa: E402
from t265 import t265_class  # noqa: E402

from .static_square_servo import StaticSquareServo, StaticSquareServoConfig  # noqa: E402
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


class ServoTelemetryLogger:
    """控制线程只入队；后台线程以最高10Hz批量写JSONL，不调用fsync。"""

    def __init__(self, path: Path, interval_s: float = 0.10) -> None:
        self.path = path
        self.interval_s = interval_s
        self._queue: queue.Queue = queue.Queue(maxsize=512)
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_submit = 0.0
        self._last_mode: str | None = None
        self._last_observation_key: tuple[int, int] | None = None
        self.dropped = 0

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._running.set()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="servo-log")
        self._thread.start()

    def submit(self, now: float, snapshot) -> None:
        event = snapshot.mode != self._last_mode or snapshot.faulted or snapshot.finished
        if not event and now - self._last_submit < self.interval_s:
            return
        self._last_submit = now
        self._last_mode = snapshot.mode
        item = {"monotonic": round(now, 6), **asdict(snapshot)}
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def submit_observation(self, now: float, observation, phase: str) -> None:
        """记录只观察实飞的原始VS1帧，不生成任何控制指令。"""
        if observation is None:
            return
        key = (observation.stream_id, observation.seq)
        if key == self._last_observation_key:
            return
        self._last_observation_key = key
        item = {
            "record_type": "vision_observation",
            "monotonic": round(now, 6),
            "phase": phase,
            **asdict(observation),
            "age_s": observation.age_s(now),
        }
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def _worker(self) -> None:
        with self.path.open("a", encoding="utf-8", buffering=8192) as handle:
            while self._running.is_set() or not self._queue.empty():
                try:
                    item = self._queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                handle.write(json.dumps(item, ensure_ascii=False) + "\n")


class StaticSquareMission(mission):
    def __init__(
        self, re_fc, se_fc, realsense_obj, serial_fc_ref, servo, telemetry,
        flight_config, observe_only=False, observe_duration_s=15.0,
    ):
        super().__init__(
            re_fc, se_fc, realsense_obj, serial_fc_ref,
            horizontal_velocity_provider=servo,
        )
        self.servo = servo
        self.telemetry = telemetry
        self.flight_config = flight_config
        self.observe_only = bool(observe_only)
        self.observe_duration_s = max(1.0, float(observe_duration_s))
        if self.observe_only:
            self.targets = [
                [0.0, 0.0, float(flight_config["discovery_height_m"])],
                [0.0, 0.0, float(flight_config["final_waypoint_height_m"])],
            ]
        else:
            self.targets = [
                [0.0, 0.0, float(flight_config["discovery_height_m"])],
                [0.0, 0.0, float(flight_config["test_height_m"])],
                [0.0, 0.0, float(flight_config["final_waypoint_height_m"])],
            ]
        self._visual_phase = "APPROACH_1P5M"
        self._discovery_waypoint_reached = False
        self._discovery_wait_start: float | None = None
        self._discovery_confirm_count = 0
        self._discovery_last_key = None
        self._test_waypoint_reached = False
        self._stable_since: float | None = None
        self._stable_wait_start: float | None = None
        self._handoff_confirm_count = 0
        self._handoff_last_key = None
        self._lost_hold_latched = False
        self._observe_started: float | None = None

    def navigate(self, pos, yaw):
        now = time.monotonic()
        if self.observe_only and self.target_index == 0 and self._discovery_waypoint_reached:
            observation = self.servo.reader.latest(
                now, max(1.0, self.servo.config.max_observation_age_s)
            )
            self.telemetry.submit_observation(now, observation, "HOVER_1P5M")
            if self._observe_started is not None and (
                now - self._observe_started >= self.observe_duration_s
            ):
                logger.info(
                    f"1.5m只观察采集完成（{self.observe_duration_s:.1f}s），"
                    "进入0.15m末航点"
                )
                target = self.targets[0]
                distance = math.hypot(pos[0] - target[0], pos[1] - target[1])
                super()._advance_waypoint("observe_complete", pos, target, distance)

        if (
            not self.observe_only
            and
            self.target_index == 0
            and self._discovery_waypoint_reached
            and not self.servo.finished
        ):
            observation = self.servo.reader.latest(
                now, self.servo.config.max_observation_age_s
            )
            if self.servo.observation_usable_for_preflight(observation, now):
                key = (observation.stream_id, observation.seq)
                if key != self._discovery_last_key:
                    self._discovery_last_key = key
                    self._discovery_confirm_count += 1
                    logger.info(
                        "1.5m AprilTag确认 "
                        f"{self._discovery_confirm_count}/"
                        f"{int(self.flight_config['discovery_confirm_frames'])}"
                    )
                    if self._discovery_confirm_count >= int(
                        self.flight_config["discovery_confirm_frames"]
                    ):
                        discovery_height = float(
                            self.flight_config["discovery_height_m"]
                        )
                        test_height = float(self.flight_config["test_height_m"])
                        height_action = (
                            f"保持{test_height:.1f}m高度"
                            if abs(test_height - discovery_height) <= 0.02
                            else f"进入{test_height:.1f}m高度航点"
                        )
                        logger.info(
                            "1.5m目标身份确认完成，立即开启XY持续追踪；"
                            f"{height_action}"
                        )
                        self.targets[1][0] = float(pos[0])
                        self.targets[1][1] = float(pos[1])
                        self.servo.arm(now, (pos[0], pos[1]))
                        self.servo.set_completion_enabled(False)
                        self._lost_hold_latched = False
                        self._visual_phase = (
                            "TRACKING_AT_FIXED_HEIGHT"
                            if abs(test_height - discovery_height) <= 0.02
                            else "TRACKING_WHILE_DESCENDING"
                        )
                        target = self.targets[0]
                        distance = math.hypot(pos[0] - target[0], pos[1] - target[1])
                        super()._advance_waypoint(
                            "apriltag_confirmed", pos, target, distance
                        )
            else:
                self._discovery_confirm_count = 0
                self._discovery_last_key = None

            if (
                self.target_index == 0
                and self._discovery_wait_start is not None
                and now - self._discovery_wait_start
                >= float(self.flight_config["discovery_timeout_s"])
            ):
                logger.error(
                    "1.5m AprilTag确认超时，取消视觉接管并进入"
                    f"{float(self.flight_config['test_height_m']):.1f}m航点"
                )
                self.servo.abort_before_arm("tag_acquisition_timeout")
                self._visual_phase = "DISCOVERY_TIMEOUT"
                target = self.targets[0]
                distance = math.hypot(pos[0] - target[0], pos[1] - target[1])
                super()._advance_waypoint(
                    "tag_acquisition_timeout", pos, target, distance
                )

        if not self.observe_only and self.target_index == 1 and not self.servo.finished:
            snapshot = self.servo.snapshot()
            if self.servo.active:
                # XY由视觉持续控制时，让基础航点的XY跟随飞机当前位置，
                # 使基础导航只负责下降高度；视觉丢失时也会原地保持而非返航。
                self.targets[1][0] = float(pos[0])
                self.targets[1][1] = float(pos[1])
            if self.servo.active and snapshot.mode == "LOST":
                if not self._lost_hold_latched:
                    # 在丢失瞬间锁住当前位置，避免T265把飞机反向拉回最初接管点，
                    # 与重新捕获后的视觉指令形成往复抖动。
                    self.targets[1][0] = float(pos[0])
                    self.targets[1][1] = float(pos[1])
                    self.x_pid.reset()
                    self.y_pid.reset()
                    self._lost_hold_latched = True
            elif snapshot.mode in (
                "FULL_TRACK", "COLOR_TRACK", "TEMPORAL_TRACK", "CENTERED",
                "PARTIAL_COARSE"
            ):
                self._lost_hold_latched = False

            velocity = self.realsense.get_velocity() if self.realsense else (0.0, 0.0, 0.0)
            confidence = self.realsense.get_tracking_confidence() if self.realsense else 0
            test_height = float(self.flight_config["test_height_m"])
            height_tolerance = float(self.flight_config["stable_height_tolerance_m"])
            stable = (
                self._test_waypoint_reached
                and test_height - height_tolerance <= pos[2] <= test_height + height_tolerance
                and math.hypot(velocity[0], velocity[1])
                < float(self.flight_config["stable_horizontal_speed_m_s"])
                and confidence >= int(self.flight_config["stable_min_confidence"])
            )
            if stable:
                if self._stable_since is None:
                    self._stable_since = now
                    self._visual_phase = "STABILIZING"
                elif now - self._stable_since >= float(
                    self.flight_config["stable_duration_s"]
                ):
                    if self.servo.active:
                        if not self.servo.completion_enabled:
                            platform_speed = math.hypot(
                                *self.servo.config.platform_velocity_m_s
                            )
                            if platform_speed > 1e-6:
                                if self._visual_phase != "MOVING_PLATFORM_TRACK":
                                    self._visual_phase = "MOVING_PLATFORM_TRACK"
                                    logger.info(
                                        f"已在{test_height:.1f}m稳定，保持XY伴飞"
                                        "直到专项测试时限结束"
                                    )
                            else:
                                self.servo.set_completion_enabled(True)
                                self._visual_phase = "VISUAL_SERVO_AT_TEST_HEIGHT"
                                logger.info(
                                    f"已在{test_height:.1f}m稳定，继续XY追踪并允许"
                                    "居中完成"
                                )
                    elif not self.servo.finished:
                        observation = self.servo.reader.latest(
                            now, self.servo.config.max_observation_age_s
                        )
                        if self.servo.observation_usable_for_preflight(observation, now):
                            key = (observation.stream_id, observation.seq)
                            if key != self._handoff_last_key:
                                self._handoff_last_key = key
                                self._handoff_confirm_count += 1
                        else:
                            self._handoff_confirm_count = 0
                            self._handoff_last_key = None
                        if self._handoff_confirm_count >= int(
                            self.flight_config["handoff_confirm_frames"]
                        ):
                            self.targets[1][0] = float(pos[0])
                            self.targets[1][1] = float(pos[1])
                            self.servo.arm(now, (pos[0], pos[1]))
                            self.servo.set_completion_enabled(True)
                            self._lost_hold_latched = True
                            self._visual_phase = "VISUAL_SERVO_AT_TEST_HEIGHT"
                            logger.info(
                                f"{test_height:.1f}m目标重新确认，XY持续追踪恢复"
                            )
            else:
                self._stable_since = None
                self._handoff_confirm_count = 0
                self._handoff_last_key = None
                if self._test_waypoint_reached:
                    self._visual_phase = "WAIT_STABLE"
            if (
                self._test_waypoint_reached
                and not self.servo.active
                and self._stable_wait_start is not None
                and now - self._stable_wait_start
                >= float(self.flight_config["stable_wait_timeout_s"])
            ):
                logger.error(
                    f"{test_height:.1f}m稳定或目标确认12秒内未满足，"
                    "取消视觉接管并进入安全下降"
                )
                self.servo.abort_before_arm("stabilization_timeout")

        if (
            not self.observe_only
            and
            self.target_index == 1
            and self._test_waypoint_reached
            and self.servo.finished
        ):
            reason = "visual_fault" if self.servo.faulted else "visual_complete"
            logger.warning(f"视觉阶段结束: {self.servo.reason}; 交回T265并进入0.15m末航点")
            self.horizontal_velocity_provider = None
            self.x_pid.reset()
            self.y_pid.reset()
            self._visual_phase = "FAULT_LOCKED" if self.servo.faulted else "COMPLETE"
            target = self.targets[1]
            distance = math.hypot(pos[0] - target[0], pos[1] - target[1])
            super()._advance_waypoint(reason, pos, target, distance)

        if self.target_index == 0 and self._discovery_waypoint_reached:
            # 1.5m发现阶段只保持T265定点，等待新鲜AprilTag连续帧。
            self._arrival_window.clear()
            self.arrival_confirmed_time = None
            self.arrival_start_time = time.time()
        if self.target_index == 1 and self._test_waypoint_reached:
            # 专用视觉阶段自己管理持续时间；阻止basic普通航点窗口每30ms重复
            # 触发“停留完成/超时”日志和推进请求。
            self._arrival_window.clear()
            self.arrival_confirmed_time = None
            self.arrival_start_time = time.time()
        super().navigate(pos, yaw)
        self.telemetry.submit(now, self.servo.snapshot())

    def _advance_waypoint(self, reason, pos, target, arrival_distance):
        if self.target_index == 0:
            self._discovery_waypoint_reached = True
            if self.observe_only:
                if self._observe_started is None:
                    self._observe_started = time.monotonic()
                    self._visual_phase = "OBSERVE_ONLY_1P5M"
                    logger.info(
                        f"到达1.5m，开始只观察采集{self.observe_duration_s:.1f}s；"
                        "视觉不会输出XY速度"
                    )
                self.arrival_start_time = time.time()
                self.arrival_confirmed_time = None
                return
            if self._discovery_wait_start is None:
                self._discovery_wait_start = time.monotonic()
                self._visual_phase = "CONFIRM_APRILTAG"
                logger.info("到达1.5m，开始等待AprilTag连续确认")
            self.arrival_start_time = time.time()
            self.arrival_confirmed_time = None
            return
        if self.target_index == 1:
            self._test_waypoint_reached = True
            if self._stable_wait_start is None:
                self._stable_wait_start = time.monotonic()
            self.arrival_start_time = time.time()
            self.arrival_confirmed_time = None
            if self.servo.active:
                return
            if not self.servo.finished:
                self._visual_phase = "WAIT_STABLE_TEST_HEIGHT"
                return
        super()._advance_waypoint(reason, pos, target, arrival_distance)


def _load_config(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    square = data["static_square_test"]
    cybercam = data["cybercam"]
    discovery_height = float(square["discovery_height_m"])
    test_height = float(square["test_height_m"])
    height_tolerance = float(square["stable_height_tolerance_m"])
    config = StaticSquareServoConfig(
        image_cx_px=float(square["image_center_px"][0]),
        image_cy_px=float(square["image_center_px"][1]),
        focal_x_px=float(square["focal_px"][0]),
        focal_y_px=float(square["focal_px"][1]),
        min_quality_full=int(square["min_quality"]),
        min_quality_partial=int(square["partial_min_quality"]),
        max_observation_age_s=float(square["lost_timeout_s"]),
        confirm_frames=int(square["confirm_frames"]),
        full_max_speed_m_s=float(square["max_speed_m_s"]),
        partial_max_speed_m_s=float(square["partial_max_speed_m_s"]),
        max_accel_m_s2=float(square["max_accel_m_s2"]),
        max_jerk_m_s3=float(square["max_jerk_m_s3"]),
        kp=float(square["position_kp"]),
        kd=float(square["velocity_kd"]),
        platform_velocity_m_s=(
            float(square["platform_velocity_m_s"][0]),
            float(square["platform_velocity_m_s"][1]),
        ),
        velocity_deadband_m_s=float(square["velocity_deadband_m_s"]),
        target_velocity_feedforward_gain=float(
            square["target_velocity_feedforward_gain"]
        ),
        deadband_m=float(square["position_deadband_m"]),
        centered_hold_s=float(square["centered_hold_s"]),
        max_duration_s=float(square["max_duration_s"]),
        soft_radius_m=float(square["soft_radius_m"]),
        hard_radius_m=float(square["hard_radius_m"]),
        min_height_m=min(discovery_height, test_height) - height_tolerance,
        max_height_m=max(discovery_height, test_height) + height_tolerance,
        low_confidence_grace_s=float(square["low_confidence_grace_s"]),
        t265_jump_window_s=float(square["t265_jump_window_s"]),
        t265_jump_m=float(square["t265_jump_m"]),
        max_measurement_innovation_m=float(square["measurement_innovation_m"]),
        vision_target_source=str(square["vision_target_source"]),
    )
    return data, cybercam, square, config


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-visual-servo", action="store_true")
    parser.add_argument("--observe-only", action="store_true")
    parser.add_argument("--observe-seconds", type=float, default=15.0)
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    parser.add_argument("--telemetry", type=Path, default=Path("static_square_servo.jsonl"))
    args = parser.parse_args(argv)
    if args.enable_visual_servo == args.observe_only:
        parser.error(
            "安全门禁：必须且只能选择 --enable-visual-servo 或 --observe-only"
        )
    if args.observe_seconds <= 0:
        parser.error("--observe-seconds 必须大于0")

    _, cybercam_cfg, square_cfg, servo_cfg = _load_config(args.config)
    reader = CyberCamReader(
        port=str(cybercam_cfg["port"]),
        baudrate=int(cybercam_cfg["baudrate"]),
    )
    if not reader.start():
        raise RuntimeError("VS1串口启动失败，禁止起飞")
    deadline = time.monotonic() + 3.0
    while reader.stats()["accepted_frames"] == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    if reader.stats()["accepted_frames"] == 0:
        reader.close()
        raise RuntimeError("3秒内未收到任何VS1帧，禁止起飞")

    servo = StaticSquareServo(reader, servo_cfg)
    if args.observe_only:
        logger.info("VS1通信门禁通过；只观察模式不会启用视觉XY控制")
    else:
        discovery_height = float(square_cfg["discovery_height_m"])
        test_height = float(square_cfg["test_height_m"])
        height_action = (
            f"保持{test_height:.1f}m高度"
            if abs(test_height - discovery_height) <= 0.02
            else f"同步下降到{test_height:.1f}m"
        )
        logger.info(
            f"VS1通信门禁通过；将在1.5m确认{servo_cfg.vision_target_source}后"
            f"立即持续追踪，并{height_action}"
        )

    telemetry = ServoTelemetryLogger(args.telemetry)
    telemetry.start()
    serial_fc = None
    mission_obj = None
    try:
        if not wait_for_start_button():
            logger.error("一键起飞门禁失败，程序退出；飞控不会解锁")
            return
        re_fc = [0] * 14
        se_fc = [170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255]
        realsense = t265_class()
        port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)
        mission_obj = StaticSquareMission(
            re_fc, se_fc, realsense, serial_fc, servo, telemetry, square_cfg,
            observe_only=args.observe_only,
            observe_duration_s=args.observe_seconds,
        )
        mission_obj.start()
        while mission_obj.task_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("用户中断")
        if mission_obj is not None:
            mission_obj.emergency()
            if not mission_obj.task_running:
                mission_obj.stop_all()
    finally:
        telemetry.close()
        reader.close()
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()


if __name__ == "__main__":
    main()
