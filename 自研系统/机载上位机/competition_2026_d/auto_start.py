"""Competition D unified boot entry for task selection and safe dispatch.

The process owns every shared UART exactly once.  T265 is deliberately not
constructed until a valid CAR_START has selected a task and all shared
preflight gates still pass.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.Logger import logger  # noqa: E402
from Lcode.global_variable import fc_last_rx_time, sp_side  # noqa: E402
from Lcode.gpio_led import set_rgb_led  # noqa: E402
from t265 import t265_class  # noqa: E402

from shared.competition_2026_d_protocol import (  # noqa: E402
    Device,
    Flag,
    MessageType,
    pack_payload,
    unpack_payload,
)

from .Lcode.air_ground_link import AirGroundLink, LinkConfig  # noqa: E402
from .payload_servo import build_payload_actuator  # noqa: E402
from .task1_flight import Task1FlightMission  # noqa: E402
from .task1_start import (  # noqa: E402
    READY_BT,
    READY_FC_LINK,
    READY_PAYLOAD_LOCKED,
    READY_VISION,
    Task1StartGate,
    _build_telemetry_sample as _build_task1_telemetry_sample,
    _load_task_config as _load_task1_config,
    _stop_mission_safely,
    _wait_for_fc,
    _wait_for_vision,
    build_joint_drop_test_config,
)
from .task1_telemetry import Task1TelemetryPublisher  # noqa: E402
from .task2_flight import Task2FlightMission  # noqa: E402
from .task2_start import (  # noqa: E402
    _build_telemetry_sample as _build_task2_telemetry_sample,
    _load_formation_config,
    _load_landing_config,
    _load_task_config as _load_task2_config,
    build_vision_landing_test_config,
)
from .task2_telemetry import Task2TelemetryPublisher  # noqa: E402
from .vision.cybercam_reader import CyberCamReader  # noqa: E402


TASK1_MODE = 1
TASK2_MODE = 2
SUPPORTED_TASK_MASK = (1 << 0) | (1 << 1)
ALL_SHARED_READY = (
    READY_BT | READY_PAYLOAD_LOCKED | READY_VISION | READY_FC_LINK
)


@dataclass(frozen=True)
class PendingTaskSelection:
    frame: object
    task_mode: int
    car_config_hash: int


class AutoStartGate(Task1StartGate):
    """Accept task 1 or 2 but defer ACK until the red warning lamp is on."""

    def __init__(
        self,
        link: AirGroundLink,
        *,
        config_hash: int,
        readiness_provider: Callable[[], bool],
        ready_interval_s: float = 0.5,
        readiness_loss_timeout_s: float = 45.0,
        **kwargs,
    ) -> None:
        self.readiness_provider = readiness_provider
        self.readiness_loss_timeout_s = max(
            1.0, float(readiness_loss_timeout_s)
        )
        self._pending_selection: PendingTaskSelection | None = None
        self._selected_task_mode: int | None = None
        super().__init__(
            link,
            config_hash=config_hash,
            ready_bits=ALL_SHARED_READY,
            ready_interval_s=ready_interval_s,
            **kwargs,
        )

    @property
    def selected_task_mode(self) -> int | None:
        with self._lock:
            return self._selected_task_mode

    def wait_selection(
        self, timeout_s: float | None = None
    ) -> PendingTaskSelection | None:
        started = self.clock()
        next_ready = 0.0
        next_diagnostic = started + 3.0
        not_ready_since: float | None = None
        while True:
            with self._lock:
                pending = self._pending_selection
            if pending is not None:
                return pending
            now = self.clock()
            if timeout_s is not None and now - started >= timeout_s:
                return None
            ready = bool(self.readiness_provider())
            if ready:
                not_ready_since = None
            elif not_ready_since is None:
                not_ready_since = now
            elif now - not_ready_since >= self.readiness_loss_timeout_s:
                raise RuntimeError(
                    "共享预检连续失效"
                    f"{self.readiness_loss_timeout_s:.0f}秒，退出等待并由systemd恢复"
                )
            if ready and now >= next_ready:
                next_ready = now + self.ready_interval_s
                self.link.publish(
                    MessageType.UAV_READY,
                    pack_payload(
                        MessageType.UAV_READY,
                        (
                            SUPPORTED_TASK_MASK,
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
                    "自启动等待任务ID: "
                    f"ready={ready}, bytes={stats.rx_bytes}, "
                    f"valid_frames={stats.rx_frames}, "
                    f"parser_rejected={stats.rx_rejected}, "
                    f"wrong_dest={stats.rx_wrong_dest}, "
                    f"start_rejected={self.rejected_start_frames}"
                )
            self._start_event.wait(0.05)

    def confirm_selection(self, selection: PendingTaskSelection) -> int | None:
        with self._lock:
            if self._pending_selection is not selection:
                raise RuntimeError("CAR_START选择已失效")
            self._session_id = int(selection.frame.session_id)
            self._car_config_hash = int(selection.car_config_hash)
            self._selected_task_mode = int(selection.task_mode)
            self._car_position = None
            self._last_car_position_seq = None
            self._pending_selection = None
        ack_seq = self.link.acknowledge(selection.frame, result=0)
        self._start_event.set()
        return ack_seq

    def reject_selection(
        self, selection: PendingTaskSelection, *, result: int = 1
    ) -> int | None:
        with self._lock:
            if self._pending_selection is selection:
                self._pending_selection = None
                self._start_event.clear()
        return self.link.acknowledge(selection.frame, result=result)

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
            logger.warning("拒绝CAR_START: payload长度或格式错误")
            return
        if task_mode not in (TASK1_MODE, TASK2_MODE) or int(frame.session_id) == 0:
            self.rejected_start_frames += 1
            self.link.acknowledge(frame, result=1)
            logger.warning(
                "拒绝CAR_START: task_mode=%d, session_id=%d",
                task_mode,
                int(frame.session_id),
            )
            return
        with self._lock:
            session_id = self._session_id
            selected_mode = self._selected_task_mode
            pending = self._pending_selection
            if session_id is not None:
                result = (
                    0
                    if session_id == int(frame.session_id)
                    and selected_mode == int(task_mode)
                    else 1
                )
            elif pending is None:
                self._pending_selection = PendingTaskSelection(
                    frame=frame,
                    task_mode=int(task_mode),
                    car_config_hash=int(car_config_hash),
                )
                self._start_event.set()
                logger.info(
                    "收到待确认任务ID: task_mode=%d, session=%d",
                    task_mode,
                    int(frame.session_id),
                )
                return
            else:
                same_pending = (
                    int(pending.frame.session_id) == int(frame.session_id)
                    and pending.task_mode == int(task_mode)
                )
                if same_pending:
                    return
                result = 1
        self.link.acknowledge(frame, result=result)


def classify_t265_usb(lsusb_output: str) -> str:
    text = str(lsusb_output).lower()
    if "8087:0b37" in text:
        return "ready"
    if "03e7:2150" in text:
        return "need_replug"
    return "not_found"


def read_t265_usb_state() -> str:
    try:
        result = subprocess.run(
            ["lsusb"], capture_output=True, text=True, timeout=5, check=False
        )
    except (OSError, subprocess.SubprocessError):
        return "error"
    return classify_t265_usb(result.stdout)


def wait_for_t265_replug(
    timeout_s: float | None = None,
    *,
    health_provider: Callable[[], bool] | None = None,
) -> bool:
    started = time.monotonic()
    last_state = None
    while timeout_s is None or time.monotonic() - started < timeout_s:
        if health_provider is not None and not bool(health_provider()):
            logger.error("等待T265拔插期间共享硬件预检失效")
            return False
        state = read_t265_usb_state()
        if state != last_state:
            logger.info("T265 USB状态: %s", state)
            last_state = state
        if state == "ready":
            return True
        time.sleep(0.5)
    return False


def _cybercam_duplex_ready(reader: CyberCamReader) -> bool:
    stats = reader.stats()
    now = time.monotonic()
    last_vs1 = stats.get("last_received_monotonic")
    last_pong = stats.get("last_pong_monotonic")
    return bool(
        stats.get("running")
        and last_vs1 is not None
        and last_pong is not None
        and 0.0 <= now - float(last_vs1) <= 1.5
        and 0.0 <= now - float(last_pong) <= 2.5
    )


def _fc_link_ready(max_age_s: float = 1.0) -> bool:
    last_rx = float(fc_last_rx_time.value)
    age = time.time() - last_rx
    return last_rx > 0.0 and 0.0 <= age <= max_age_s


def _wait_for_ack_write(
    link: AirGroundLink, before_count: int, timeout_s: float = 0.30
) -> bool:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while time.monotonic() < deadline:
        if link.stats.tx_ack_frames > before_count:
            return True
        time.sleep(0.005)
    return link.stats.tx_ack_frames > before_count


def _build_link(bluetooth: dict) -> AirGroundLink:
    return AirGroundLink(
        LinkConfig(
            port=str(bluetooth["port"]),
            baudrate=int(bluetooth["baudrate"]),
            write_timeout_s=float(bluetooth.get("write_timeout_s", 0.20)),
            ack_timeout_s=float(bluetooth["ack_timeout_s"]),
            max_retries=int(bluetooth["max_retries"]),
            max_consecutive_tx_errors=int(
                bluetooth.get("max_consecutive_tx_errors", 3)
            ),
        )
    )


def main(argv=None) -> None:
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
        help="等待小车任务ID的秒数；默认无限等待",
    )
    parser.add_argument(
        "--t265-replug-timeout",
        type=float,
        default=None,
        help="等待T265拔插完成的秒数；默认无限等待",
    )
    parser.add_argument(
        "--task-light-seconds",
        type=float,
        default=1.0,
        help="ACK前显示任务颜色的时间，默认1秒",
    )
    args = parser.parse_args(argv)

    if args.task_light_seconds < 0.0 or args.task_light_seconds > 3.0:
        parser.error("--task-light-seconds必须在0~3秒之间")

    raw_config = args.config.read_bytes()
    data = json.loads(raw_config.decode("utf-8"))
    config_hash = zlib.crc32(raw_config) & 0xFFFFFFFF
    bluetooth = data["bluetooth"]
    cybercam = data["cybercam"]
    square = data["static_square_test"]
    payload_cfg = data.get("payload_servo", {})

    task1_config = build_joint_drop_test_config(_load_task1_config(data))
    task2_config = build_vision_landing_test_config(_load_task2_config(data))
    landing_config = _load_landing_config(data)
    formation_config = _load_formation_config(data)
    task1_offset = (
        float(data["task1"].get("vision_target_offset_x_m", 0.0)),
        float(data["task1"].get("vision_target_offset_y_m", 0.0)),
    )

    link = None
    reader = None
    serial_fc = None
    mission_obj = None
    telemetry = None
    telemetry_finished = False
    realsense = None
    actuator = None
    start_confirmed = False

    try:
        if not set_rgb_led("W"):
            raise RuntimeError("自启动白色预检状态灯点亮失败")

        actuator, payload_hardware = build_payload_actuator(
            release_hold_s=float(payload_cfg.get("release_hold_s", 1.0))
        )
        if not payload_hardware.ready:
            raise RuntimeError("投放舵机未能锁定，禁止进入自动待命")

        link = _build_link(bluetooth)
        if not link.start():
            raise RuntimeError("蓝牙链路启动失败")

        reader = CyberCamReader(
            port=str(cybercam["port"]),
            baudrate=int(cybercam["baudrate"]),
        )
        if not reader.start() or not _wait_for_vision(reader, timeout_s=45.0):
            raise RuntimeError("Cyber Camera双向通信预检失败")
        logger.info("Cyber Camera VS1与PING/PONG预检通过")

        re_fc = [0] * 14
        se_fc = [
            170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255
        ]
        fc_port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(fc_port, 460800)
        serial_fc.listen_start(re_fc)
        # Only command frames run before CAR_START.  No T265 object exists yet.
        serial_fc.send_start(se_fc, None, vel_freq=100, cmd_freq=50)
        if not _wait_for_fc():
            raise RuntimeError("飞控串口3秒内未收到遥测")

        logger.info("等待人工拔插T265；此阶段不创建T265对象、不启动管线")
        if not wait_for_t265_replug(
            args.t265_replug_timeout,
            health_provider=lambda: bool(
                link.is_running
                and reader.is_running()
                and _fc_link_ready(max_age_s=3.0)
            ),
        ):
            raise RuntimeError("等待T265拔插超时")

        t265_cache = {"checked_at": float("-inf"), "ready": False}

        def t265_usb_ready_cached() -> bool:
            now = time.monotonic()
            if now - t265_cache["checked_at"] >= 0.5:
                t265_cache["checked_at"] = now
                t265_cache["ready"] = read_t265_usb_state() == "ready"
            return bool(t265_cache["ready"])

        def shared_ready() -> bool:
            return bool(
                link is not None
                and link.is_running
                and _cybercam_duplex_ready(reader)
                and _fc_link_ready()
                and t265_usb_ready_cached()
            )

        gate = AutoStartGate(
            link,
            config_hash=config_hash,
            readiness_provider=shared_ready,
            state_max_age_s=float(bluetooth.get("car_state_max_age_s", 0.30)),
            state_max_speed_m_s=float(
                bluetooth.get("car_speed_max_m_s", 0.30)
            ),
            state_max_component_m_s=float(
                bluetooth.get("car_component_max_m_s", 0.40)
            ),
        )
        logger.info(
            "统一自启动已就绪：T265仅完成USB拔插检查；等待CAR_START.task_mode=1或2"
        )

        while True:
            selection = gate.wait_selection(args.start_timeout)
            if selection is None:
                logger.error("等待任务ID超时，自启动取消")
                return
            if not shared_ready():
                gate.reject_selection(selection, result=1)
                logger.warning("收到任务ID时共享预检已失效，已NAK并继续等待")
                continue

            task_color = "G" if selection.task_mode == TASK1_MODE else "B"
            if not set_rgb_led(task_color):
                gate.reject_selection(selection, result=1)
                raise RuntimeError("任务选择状态灯点亮失败")
            logger.info(
                "任务ID确认: task_mode=%d，显示%s灯%.1f秒",
                selection.task_mode,
                "绿" if task_color == "G" else "蓝",
                args.task_light_seconds,
            )
            if args.task_light_seconds:
                time.sleep(args.task_light_seconds)
            if not shared_ready():
                gate.reject_selection(selection, result=1)
                set_rgb_led("W")
                logger.warning("任务灯显示期间共享预检失效，已NAK并继续等待")
                continue
            if not set_rgb_led("R"):
                gate.reject_selection(selection, result=1)
                raise RuntimeError("红色起飞警示灯点亮失败")
            warning_started_at = time.monotonic()
            ack_before = link.stats.tx_ack_frames
            ack_seq = gate.confirm_selection(selection)
            start_confirmed = True
            if ack_seq is None or not _wait_for_ack_write(link, ack_before):
                raise RuntimeError("CAR_START ACK未写入蓝牙串口，禁止启动任务")
            logger.info(
                "CAR_START ACK已写入: task_mode=%d, session=%d, ack_seq=%s",
                selection.task_mode,
                int(selection.frame.session_id),
                ack_seq,
            )
            break

        # This is the first point where the T265 object may be constructed.
        realsense = t265_class()
        serial_fc.send_start(None, realsense, vel_freq=100, cmd_freq=50)

        if selection.task_mode == TASK1_MODE:
            mission_obj = Task1FlightMission(
                re_fc,
                se_fc,
                realsense,
                serial_fc,
                vision_reader=reader,
                payload_actuator=actuator,
                task_config=task1_config,
                vision_config={
                    "max_age_s": float(cybercam["max_age_s"]),
                    "min_quality": min(
                        int(task1_config.vision_min_quality),
                        int(task1_config.acquire_vision_min_quality),
                    ),
                    "image_center_px": tuple(square["image_center_px"]),
                    "focal_px": tuple(square["focal_px"]),
                    "target_offset_xy_m": task1_offset,
                },
                car_speed_provider=None,
            )
            telemetry = Task1TelemetryPublisher(
                link,
                session_id=int(selection.frame.session_id),
                state_interval_s=1.0
                / max(1.0, float(bluetooth.get("uav_state_hz", 10.0))),
            )
            def telemetry_builder():
                return _build_task1_telemetry_sample(mission_obj, realsense)

            logger.info("启动任务一联合投放模式：巡航/伴飞高度1.20m")
        else:
            def car_position_provider():
                position = gate.latest_car_position()
                return position.position_xy_m if position is not None else None

            mission_obj = Task2FlightMission(
                re_fc,
                se_fc,
                realsense,
                serial_fc,
                vision_reader=reader,
                payload_actuator=actuator,
                task_config=task2_config,
                formation_config=formation_config,
                landing_config=landing_config,
                vision_config={
                    "max_age_s": float(cybercam["max_age_s"]),
                    "min_quality": int(task2_config.vision_min_quality),
                    "image_center_px": tuple(square["image_center_px"]),
                    "focal_px": tuple(square["focal_px"]),
                },
                car_speed_provider=gate.car_speed,
                car_position_provider=car_position_provider,
                car_velocity_provider=gate.car_velocity,
            )
            telemetry = Task2TelemetryPublisher(
                link,
                session_id=int(selection.frame.session_id),
                state_interval_s=1.0
                / max(1.0, float(bluetooth.get("uav_state_hz", 10.0))),
            )
            def telemetry_builder():
                return _build_task2_telemetry_sample(mission_obj)

            logger.info("启动任务二视觉动态降落模式：巡航/等待高度1.20m")

        mission_obj.preflight_warning_started_at = warning_started_at
        if not link.is_running or not _cybercam_duplex_ready(reader):
            raise RuntimeError("任务启动前通信链路失效")
        mission_obj.start()
        if not mission_obj.task_running:
            raise RuntimeError("T265或起飞预检失败，任务未启动")
        while mission_obj.task_running:
            sample = telemetry_builder()
            if sample is not None:
                telemetry.update(sample)
            time.sleep(0.03)

        telemetry.finish(
            mission_success=bool(mission_obj.director.mission_success),
            faulted=bool(mission_obj.emergency_stop),
        )
        telemetry_finished = True
        link.wait_pending(0.65)
        if not mission_obj.director.mission_success:
            raise RuntimeError("任务未成功完成")
        logger.info("统一自启动任务成功结束；服务保持停止，等待下次重新上电")
    except KeyboardInterrupt:
        logger.warning("用户或systemd中止统一自启动程序")
        _stop_mission_safely(mission_obj)
    except Exception:
        _stop_mission_safely(mission_obj)
        if start_confirmed:
            logger.exception(
                "CAR_START已确认后发生故障；以正常退出阻止systemd自动重新武装"
            )
            return
        raise
    finally:
        _stop_mission_safely(mission_obj)
        if telemetry is not None and not telemetry_finished:
            mission_success = bool(
                mission_obj is not None
                and mission_obj.director.mission_success
            )
            telemetry.finish(mission_success=mission_success, faulted=True)
            if link is not None:
                link.wait_pending(0.65)
        try:
            set_rgb_led("OFF")
        except Exception:
            pass
        if reader is not None:
            reader.close()
        if realsense is not None:
            try:
                if realsense.is_running():
                    realsense.stop()
            except Exception:
                pass
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()
