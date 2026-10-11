"""Car Raspberry Pi coordinate gateway.

Data flow:
  1. 发给无人机：CAR_POSITION 和 CAR_STATE 的位置/速度直接置零。
  2. 发给地面站：
     - CAR_START（含 task_mode，握手后连发 3 帧）
     - UAV_STATE 降频转发：无人机 10Hz 上报，树莓派以 2Hz 转发最新帧
     - 转发无人机事件（UAV_EVENT / UAV_READY）
     小车移动情况由地面站自行模拟。
"""

from __future__ import annotations

import argparse
import logging
import signal
import struct
import threading
import time
from typing import Optional, Protocol

from coordinate_protocol import CarPosition, CarState, UavPosition
from dcp_codec import (
    ACK,
    ACK_REQUIRED,
    BROADCAST,
    CAR,
    CAR_POSITION,
    CAR_START,
    CAR_STATE,
    DROP_RELEASED,
    EVENT,
    FAULT_EVENT,
    GROUND,
    IS_ACK,
    MISSION_COMPLETE,
    RETAKEOFF_STARTED,
    TOUCHDOWN_CONFIRMED,
    UAV,
    UAV_EVENT,
    UAV_READY,
    UAV_STATE,
    DcpFrame,
    DcpStreamParser,
    build_frame,
    seq_is_new,
)


# ---------------------------------------------------------------------------
# Frequently changed field configuration
# ---------------------------------------------------------------------------
UAV_SERIAL_PORT = "/dev/ttyUAV"        # UAV <-> car Raspberry Pi (udev 固定名，原 ttyUSB0)
GROUND_SERIAL_PORT = "/dev/ttyGROUND"  # car Raspberry Pi -> ground station (udev 固定名，原 ttyUSB1)
SERIAL_BAUD = 115200
# 地面站 DL-20 波特率独立配置（地面站 DL-20 改成 9600 后需单独设置）。
GROUND_SERIAL_BAUD = 9600
# 地面站链路开关：False 时不打开物理串口、不发送任何数据（DL-20 可拔走单独测试）。
# 改回 True 即恢复向地面站转发数据。
GROUND_LINK_ENABLED = True

CAR_TO_UAV_HZ = 10.0
CAR_STATE_TO_UAV_HZ = 5.0       # CAR_STATE 发往无人机的频率（5Hz，足够控制用）
# UAV_STATE 转发给地面站的频率：无人机以 10Hz 上报，树莓派降频到 2Hz 转发，
# 减轻 DL-20 蓝牙链路负载。每次只转发缓存中最新的那一帧。
UAV_STATE_RELAY_HZ = 2.0

# 程序最长运行时间（秒）。到时间自动退出，0 表示不限时。
MAX_RUN_TIME_S = 150.0   # 2.5 分钟

# 启动握手：发 CAR_START 后等待无人机 ACK 的最长时间（秒）。
# 第一阶段等 CAR_START_ACK_TIMEOUT_S 秒；仍未收到 ACK 则进入第二阶段，
# 再等 CAR_START_EXTRA_WAIT_S 秒（期间继续重发 CAR_START）。
# 两阶段都超时后不再退出，直接进入主循环（session_id 保持 0）。
CAR_START_ACK_TIMEOUT_S = 10.0
CAR_START_EXTRA_WAIT_S = 2.0
# 重发 CAR_START 的间隔（秒）。无人机端要求 100~120ms 重发，取 0.12s（约 8.3Hz）。
CAR_START_RESEND_INTERVAL_S = 0.12
# 本机任务配置：1=抛投任务, 2=动态起降任务。
CAR_TASK_MODE = 1
# 本机生成的 session_id。0 表示由外部注入；非 0 时启动后用此值发 CAR_START。
CAR_SESSION_ID = 0x2026D001

# 启动按钮：按下后才开始发 CAR_START 握手。
# 使用 BCM 编号。按钮一端接 GPIO，另一端接 3.3V，启用内部下拉，按下时输入变 HIGH。
# 设为 None 则跳过按钮等待，启动后立即开始握手（便于自动化测试）。
CAR_START_BUTTON_PIN = 24
# 按下反馈引脚：按钮确认按下后，该引脚产生反馈脉冲（如接 LED 或驱动信号）。
# 物理引脚 16 = BCM GPIO23。
CAR_START_BUTTON_FEEDBACK_PIN = 23
# 任务切换：单击选任务1（抛投），双击选任务2（动态起降）。
# 单击反馈：BCM23 持续 HIGH 1.0s 后回 LOW。
# 双击反馈：BCM23 连发 5 次脉冲，每次 HIGH 100ms + LOW 100ms（共 1.0s）。
CAR_START_BUTTON_FEEDBACK_HOLD_S = 1.0          # 单击高电平持续秒
CAR_DOUBLE_CLICK_WINDOW_S = 0.4                 # 两次按下间隔在此范围内才算双击
CAR_DOUBLE_CLICK_PULSE_COUNT = 5                # 双击连发的高电平次数
CAR_DOUBLE_CLICK_PULSE_HIGH_S = 0.1            # 双击每次高电平持续秒
CAR_DOUBLE_CLICK_PULSE_LOW_S = 0.1             # 双击每次低电平持续秒
# 按钮采样与消抖
CAR_BUTTON_DEBOUNCE_SAMPLES = 3                 # 连续 HIGH 次数确认按下（约 30ms）
CAR_BUTTON_POLL_INTERVAL_S = 0.01              # 采样间隔（10ms）

# RGB 状态指示灯（BCM 编号）。
# 状态映射：绿灯=程序运行等待按钮，红灯=按钮按下等待ACK，蓝灯=收到ACK进入任务。
# 接线：R/B/G 引脚各串一个限流电阻（约 220~330Ω）到 LED 对应引脚，共阴极接地。
RGB_R_PIN = 6
RGB_G_PIN = 5
RGB_B_PIN = 0

# The attachment defines UAV_STATE as H-origin coordinates. These values move
# the UAV coordinate into the same field-global frame as the car.
UAV_INITIAL_X_MM = 750
UAV_INITIAL_Y_MM = 750


class RGBIndicator:
    """RGB 状态指示灯控制器。

    状态映射：
      green()  程序已启动，等待按钮按下
      red()    按钮已按下，等待无人机 ACK
      blue()   已收到 ACK，进入任务状态
      off()    关闭所有颜色

    环境兼容：无 RPi.GPIO 时降级为 no-op，不影响 Windows 测试。
    接线：R/G/B 引脚各串一个限流电阻到 LED，共阴极接地。
    """

    def __init__(self, r_pin: int, g_pin: int, b_pin: int) -> None:
        self.r_pin = r_pin
        self.g_pin = g_pin
        self.b_pin = b_pin
        self._gpio = None
        try:
            import RPi.GPIO as GPIO
            self._gpio = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for p in (r_pin, g_pin, b_pin):
                GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
        except ImportError:
            logging.info("RGBIndicator: RPi.GPIO unavailable, LED disabled (no-op)")

    def _set(self, r: bool, g: bool, b: bool) -> None:
        if self._gpio is None:
            return
        self._gpio.output(self.r_pin, self._gpio.HIGH if r else self._gpio.LOW)
        self._gpio.output(self.g_pin, self._gpio.HIGH if g else self._gpio.LOW)
        self._gpio.output(self.b_pin, self._gpio.HIGH if b else self._gpio.LOW)

    def green(self) -> None:
        self._set(r=False, g=True, b=False)

    def red(self) -> None:
        self._set(r=True, g=False, b=False)

    def blue(self) -> None:
        self._set(r=False, g=False, b=True)

    def off(self) -> None:
        self._set(r=False, g=False, b=False)

    def close(self) -> None:
        if self._gpio is None:
            return
        self.off()
        for p in (self.r_pin, self.g_pin, self.b_pin):
            self._gpio.cleanup(p)


def wait_for_start_button(
    pin: Optional[int],
    stop_event: Optional[threading.Event] = None,
    feedback_pin: Optional[int] = None,
    feedback_hold_s: float = CAR_START_BUTTON_FEEDBACK_HOLD_S,
    rgb: Optional["RGBIndicator"] = None,
    double_click_window_s: float = CAR_DOUBLE_CLICK_WINDOW_S,
    pulse_count: int = CAR_DOUBLE_CLICK_PULSE_COUNT,
    pulse_high_s: float = CAR_DOUBLE_CLICK_PULSE_HIGH_S,
    pulse_low_s: float = CAR_DOUBLE_CLICK_PULSE_LOW_S,
    debounce_samples: int = CAR_BUTTON_DEBOUNCE_SAMPLES,
    poll_interval_s: float = CAR_BUTTON_POLL_INTERVAL_S,
) -> tuple[bool, int]:
    """阻塞等待 GPIO 按钮按下，根据单击/双击选择任务模式后返回。

    任务切换逻辑（参考 communication_protocol.md task_mode）：
      - 单击（窗口内未检测到第二次按下）→ task_mode=1（抛投任务）
        反馈：BCM23 持续 HIGH feedback_hold_s 秒后回 LOW。
      - 双击（窗口内检测到第二次按下）→ task_mode=2（动态起降任务）
        反馈：BCM23 连发 pulse_count 次脉冲，每次 HIGH pulse_high_s + LOW pulse_low_s。

    接线：按钮一端接 BCM=pin，另一端接 3.3V，启用内部下拉。
    按下时引脚电平变 HIGH（上升沿），松开时为 LOW。

    若 rgb 非空，按下确认后立即切换为红灯（表示等待 ACK）。

    环境兼容：
      - 树莓派：用 RPi.GPIO 轮询，poll_interval_s 采样 + 软件消抖。
      - 测试环境（无 RPi.GPIO，如 Windows）：用 Enter 键模拟按下（按一次=单击任务1，
        连按两次 Enter 且中间不输入内容=双击任务2；实现简化为直接选任务1）。
      - pin=None：直接返回 (True, CAR_TASK_MODE)，跳过等待（便于自动化脚本）。

    返回值 (success, task_mode)：
      (True, 1)  单击，任务1（抛投）
      (True, 2)  双击，任务2（动态起降）
      (False, 0) 收到 stop_event（Ctrl+C）应中止退出
    """
    if pin is None:
        if rgb is not None:
            rgb.red()
        return True, CAR_TASK_MODE
    try:
        import RPi.GPIO as GPIO
    except ImportError:
        if rgb is not None:
            rgb.red()
        try:
            input(f"[BUTTON] RPi.GPIO unavailable; press Enter to send CAR_START (simulates GPIO{pin}, task_mode={CAR_TASK_MODE})... ")
        except (EOFError, KeyboardInterrupt):
            return False, 0
        return True, CAR_TASK_MODE
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    if feedback_pin is not None:
        GPIO.setup(feedback_pin, GPIO.OUT, initial=GPIO.LOW)
    logging.info(
        "Waiting for GPIO%d button: single-click=task1(drop, BCM%d HIGH %.1fs), "
        "double-click=task2(takeoff, BCM%d %d pulses).",
        pin, feedback_pin, feedback_hold_s,
        feedback_pin, pulse_count,
    )

    def _stop_requested() -> bool:
        return stop_event is not None and stop_event.is_set()

    def _wait_button_release() -> bool:
        """阻塞等待按钮释放（电平回到 LOW）。返回 False 表示收到 stop_event。"""
        while GPIO.input(pin) == GPIO.HIGH:
            if _stop_requested():
                return False
            time.sleep(poll_interval_s)
        return True

    def _wait_second_press(window_s: float) -> bool:
        """在 window_s 时间窗口内检测第二次按下。返回 True 表示检测到第二次按下。"""
        deadline = time.monotonic() + window_s
        stable = 0
        while time.monotonic() < deadline:
            if _stop_requested():
                return False
            if GPIO.input(pin) == GPIO.HIGH:
                stable += 1
                if stable >= debounce_samples:
                    return True
            else:
                stable = 0
            time.sleep(poll_interval_s)
        return False

    def _feedback_single_click() -> None:
        """单击反馈：BCM23 持续 HIGH feedback_hold_s 秒后回 LOW。"""
        if feedback_pin is None:
            return
        GPIO.output(feedback_pin, GPIO.HIGH)
        logging.info("Feedback single-click: GPIO%d HIGH %.2fs", feedback_pin, feedback_hold_s)
        time.sleep(feedback_hold_s)
        GPIO.output(feedback_pin, GPIO.LOW)
        logging.info("Feedback single-click: GPIO%d LOW", feedback_pin)

    def _feedback_double_click() -> None:
        """双击反馈：BCM23 连发 pulse_count 次脉冲。"""
        if feedback_pin is None:
            return
        total_s = pulse_count * (pulse_high_s + pulse_low_s)
        logging.info(
            "Feedback double-click: GPIO%d %d pulses (HIGH %.0fms + LOW %.0fms), total %.2fs",
            feedback_pin, pulse_count, pulse_high_s * 1000, pulse_low_s * 1000, total_s,
        )
        for _ in range(pulse_count):
            GPIO.output(feedback_pin, GPIO.HIGH)
            time.sleep(pulse_high_s)
            GPIO.output(feedback_pin, GPIO.LOW)
            time.sleep(pulse_low_s)
        logging.info("Feedback double-click: GPIO%d done", feedback_pin)

    try:
        while True:
            if _stop_requested():
                return False, 0
            # 第一阶段：等待第一次按下（消抖）
            if GPIO.input(pin) == GPIO.HIGH:
                stable = 0
                while GPIO.input(pin) == GPIO.HIGH:
                    if _stop_requested():
                        return False, 0
                    stable += 1
                    if stable >= debounce_samples:
                        break
                    time.sleep(poll_interval_s)
                if stable < debounce_samples:
                    time.sleep(poll_interval_s)
                    continue
                logging.info("First press on GPIO%d.", pin)
                # 等待第一次按下释放
                if not _wait_button_release():
                    return False, 0
                # 第二阶段：开启双击窗口，检测第二次按下
                if _wait_second_press(double_click_window_s):
                    # 双击：任务2（动态起降）
                    logging.info("Double-click detected -> task_mode=2 (动态起降).")
                    if rgb is not None:
                        rgb.red()
                    _feedback_double_click()
                    return True, 2
                else:
                    # 单击：任务1（抛投）
                    logging.info("Single-click detected -> task_mode=1 (抛投).")
                    if rgb is not None:
                        rgb.red()
                    _feedback_single_click()
                    return True, 1
            time.sleep(poll_interval_s)
    except KeyboardInterrupt:
        return False, 0
    finally:
        # 只清理按钮输入引脚；反馈引脚保持 LOW 输出，避免 cleanup 后变高阻态悬浮在 1.65V。
        # 反馈引脚保持 LOW（0V）输出状态，由系统在程序退出时统一回收。
        GPIO.cleanup(pin)


class SerialLike(Protocol):
    in_waiting: int

    def read(self, size: int = 1) -> bytes: ...
    def write(self, data: bytes) -> int: ...
    def flush(self) -> None: ...
    def close(self) -> None: ...


class DummySerial:
    """空操作串口：DL-20 未连接时用作占位，所有方法均为 no-op。"""
    in_waiting = 0

    def read(self, size: int = 1) -> bytes:
        return b""

    def write(self, data: bytes) -> int:
        return len(data)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


class SequenceCounter:
    def __init__(self) -> None:
        self._value = 0
        self._lock = threading.Lock()

    def next(self) -> int:
        with self._lock:
            self._value = (self._value + 1) & 0xFFFF
            return self._value


class CarGateway:
    def __init__(self, uav_serial: SerialLike, ground_serial: SerialLike,
                 rgb: Optional[RGBIndicator] = None) -> None:
        self.uav_serial = uav_serial
        self.ground_serial = ground_serial
        self.parser = DcpStreamParser()
        self.stop_event = threading.Event()
        self.started_at = time.monotonic()
        self.tx_seq = SequenceCounter()
        self.uav_write_lock = threading.Lock()
        self.ground_write_lock = threading.Lock()
        self.session_lock = threading.Lock()
        self.active_session = 0
        self.last_uav_state: Optional[tuple[int, int]] = None
        self.seen_events: set[tuple[int, int, int]] = set()
        self.car_position_count = 0
        self.car_state_count = 0
        self.uav_state_relay_count = 0
        # 最新待转发的 UAV_STATE 帧（_handle_uav_state 更新，_uav_state_relay_loop 取出转发）
        self._latest_uav_state_relay: Optional[bytes] = None
        self._uav_state_relay_lock = threading.Lock()
        self.rgb = rgb
        # 任务模式：1=抛投任务，2=动态起降任务。由按钮单击/双击在握手前选定。
        # 默认 CAR_TASK_MODE；main() 中根据 wait_for_start_button 返回值覆盖。
        self.task_mode: int = CAR_TASK_MODE

    def sender_ms(self) -> int:
        return int((time.monotonic() - self.started_at) * 1000) & 0xFFFFFFFF

    def set_active_session(self, session_id: int) -> None:
        if session_id:
            with self.session_lock:
                self.active_session = session_id

    def get_active_session(self) -> int:
        with self.session_lock:
            return self.active_session

    def build_car_position_frame(self) -> bytes:
        """发给无人机的 CAR_POSITION：位置直接置零。"""
        session_id = self.get_active_session()
        payload = CarPosition(0, 0, 0, 0).pack()
        return build_frame(
            CAR_POSITION,
            payload,
            flags=0,
            source=CAR,
            destination=UAV,
            session_id=session_id,
            seq=self.tx_seq.next(),
            sender_ms=self.sender_ms(),
        )

    def emit_car_position_once(self) -> None:
        frame = self.build_car_position_frame()
        with self.uav_write_lock:
            self.uav_serial.write(frame)
            self.uav_serial.flush()
        self.car_position_count += 1

    def build_car_state_frame(self) -> bytes:
        """组一帧 CAR_STATE（13 字节扩展格式，含 vx/vy），发给无人机。

        位置/速度全部置零（按任务要求，无人机不需要小车实际位置）。
        """
        session_id = self.get_active_session()
        state = CarState(
            segment=0,
            track_s_mm=0,
            speed_mm_s=0,
            heading_cdeg=0,
            flags=0,
            vx_mm_s=0,
            vy_mm_s=0,
        )
        return build_frame(
            CAR_STATE,
            state.pack(),
            flags=0,
            source=CAR,
            destination=UAV,
            session_id=session_id,
            seq=self.tx_seq.next(),
            sender_ms=self.sender_ms(),
        )

    def emit_car_state_once(self) -> None:
        """向无人机发一帧 CAR_STATE（13 字节扩展格式，全部置零）。"""
        frame = self.build_car_state_frame()
        with self.uav_write_lock:
            self.uav_serial.write(frame)
            self.uav_serial.flush()
        self.car_state_count += 1

    def _car_state_loop(self) -> None:
        period = 1.0 / CAR_STATE_TO_UAV_HZ
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.emit_car_state_once()
            except Exception:
                logging.exception("Failed to send CAR_STATE; will retry on the next tick")
            deadline += period
            delay = max(0.0, deadline - time.monotonic())
            self.stop_event.wait(delay)

    def _car_position_loop(self) -> None:
        period = 1.0 / CAR_TO_UAV_HZ
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            try:
                self.emit_car_position_once()
            except Exception:
                logging.exception("Failed to send CAR_POSITION; will retry on the next 10 Hz tick")
            deadline += period
            delay = max(0.0, deadline - time.monotonic())
            self.stop_event.wait(delay)

    def _is_new_uav_state(self, frame: DcpFrame) -> bool:
        current = (frame.session_id, frame.seq)
        if self.last_uav_state is None or self.last_uav_state[0] != frame.session_id:
            self.last_uav_state = current
            return True
        if seq_is_new(frame.seq, self.last_uav_state[1]):
            self.last_uav_state = current
            return True
        return False

    def _write_ground(self, data: bytes) -> bool:
        """向地面站写数据。如果 DL-20 阻塞则清空输出缓冲后重试一次。"""
        if not GROUND_LINK_ENABLED:
            return False
        try:
            with self.ground_write_lock:
                self.ground_serial.write(data)
                self.ground_serial.flush()
            return True
        except Exception:
            # DL-20 阻塞：清空输出缓冲，重试一次发送最新帧
            try:
                self.ground_serial.reset_output_buffer()
            except Exception:
                pass
            try:
                with self.ground_write_lock:
                    self.ground_serial.write(data)
                    self.ground_serial.flush()
                logging.warning("Ground DL-20 blocked, cleared buffer and retried OK")
                return True
            except Exception:
                # The ground link is telemetry-only. Its failure must not stop the
                # independent 10 Hz safety/coordination link from car to UAV.
                logging.exception("Ground-station serial write failed after clear; frame discarded")
                return False

    def _handle_uav_state(self, frame: DcpFrame) -> None:
        """收到无人机 UAV_STATE：打印位置，设置 session，缓存最新帧供 2Hz 转发。"""
        if not self._is_new_uav_state(frame):
            return
        uav_local = UavPosition.unpack(frame.payload)
        # 在终端实时打印无人机回传的位置数据，便于调试通信链路。
        # 局部坐标 = 无人机自报的相对坐标；全局坐标 = 叠加 UAV_INITIAL 偏移后的场地坐标。
        uav_global_x = UAV_INITIAL_X_MM + uav_local.x_mm
        uav_global_y = UAV_INITIAL_Y_MM + uav_local.y_mm
        print(
            f"[UAV_STATE] seq={frame.seq} session=0x{frame.session_id:08X} "
            f"local=({uav_local.x_mm:+d},{uav_local.y_mm:+d},{uav_local.z_mm:+d})mm "
            f"global=({uav_global_x:+d},{uav_global_y:+d})mm "
            f"sender_ms={frame.sender_ms}",
            flush=True,
        )
        self.set_active_session(frame.session_id)
        # 缓存最新帧（改 destination=GROUND），供 _uav_state_relay_loop 以 2Hz 取出转发。
        # 无人机以 10Hz 上报，这里每次都覆盖缓存，保证转发的总是最新一帧。
        relayed = build_frame(
            UAV_STATE,
            frame.payload,
            flags=frame.flags,
            source=frame.source,
            destination=GROUND,
            session_id=frame.session_id,
            seq=frame.seq,
            sender_ms=frame.sender_ms,
        )
        with self._uav_state_relay_lock:
            self._latest_uav_state_relay = relayed

    def _uav_state_relay_loop(self) -> None:
        """2Hz 线程：取出缓存的最新 UAV_STATE 帧转发给地面站。

        无人机以 10Hz 上报，这里以 2Hz（每 500ms）取一帧转发，
        减轻 DL-20 蓝牙链路负载。取出后清空缓存，下次只发更新的帧。
        """
        period = 1.0 / UAV_STATE_RELAY_HZ
        while not self.stop_event.is_set():
            try:
                with self._uav_state_relay_lock:
                    frame = self._latest_uav_state_relay
                    self._latest_uav_state_relay = None
                if frame is not None:
                    if self._write_ground(frame):
                        self.uav_state_relay_count += 1
            except Exception:
                logging.exception("Failed to relay UAV_STATE to ground")
            self.stop_event.wait(period)

    def _ack_uav_event(self, frame: DcpFrame) -> None:
        """回 ACK 给无人机。每个 ACK 连发 3 次（间隔 50ms）提高可靠性。

        无人机端收到任一帧即停止重发。连发 3 次是为了应对蓝牙链路的丢包。

        无人机端 ACK 匹配要求（固定字段）：
          type        = 0x04 (ACK)
          flags       = 0x02 (IS_ACK)
          source      = CAR (0x02)
          destination = UAV (0x01)
          session_id  = UAV_EVENT 的 session_id
          payload:
            acked_type = 0x20 (固定，所有 UAV 事件统一用 0x20)
            acked_seq  = UAV_EVENT 帧头 seq
            result     = 0

        每次 write 用 try/except 隔离，单次失败不中断后续发送。
        """
        ack_payload = struct.pack("<BHB", UAV_EVENT, frame.seq, 0)
        sent_ok = 0
        for i in range(3):
            ack = build_frame(
                ACK,
                ack_payload,
                flags=IS_ACK,
                source=CAR,
                destination=UAV,
                session_id=frame.session_id,
                seq=self.tx_seq.next(),
                sender_ms=self.sender_ms(),
            )
            try:
                with self.uav_write_lock:
                    self.uav_serial.write(ack)
                    self.uav_serial.flush()
                sent_ok += 1
            except Exception as exc:
                logging.warning(
                    "ACK write failed (attempt %d/3) for event type=0x%02X seq=%d: %s",
                    i + 1, frame.message_type, frame.seq, exc,
                )
            if i < 2:
                time.sleep(0.05)
        logging.info(
            "ACK sent %d/3 for event type=0x%02X seq=%d acked_seq=%d",
            sent_ok, frame.message_type, frame.seq, frame.seq,
        )

    # phase 值 -> 中文名（协议第100-112行）
    UAV_PHASE_NAMES = {
        0: "BOOT", 1: "WAIT_START", 2: "READY", 3: "起飞", 4: "HOLD_3S",
        5: "拦截", 6: "伴飞", 7: "抛投", 8: "下降", 9: "触地",
        10: "平台停留", 11: "二次起飞", 12: "返航", 13: "降落H",
        14: "任务完成", 15: "故障", 18: "地面待命",
    }

    def _parse_and_log_uav_event(self, frame: DcpFrame) -> str:
        """按协议第72-76行解析各事件类型，终端打印中文描述，返回事件名。

        每个事件是独立的 message_type，payload 格式各不相同：
          DROP_RELEASED (0x20)      <IB>   5B  elapsed_ms:u32, quality:u8
          TOUCHDOWN_CONFIRMED (0x21) <IB>  5B  elapsed_ms:u32, confidence:u8
          RETAKEOFF_STARTED (0x22)  <I>    4B  elapsed_ms:u32
          MISSION_COMPLETE (0x23)   <BI>   5B  result:u8, elapsed_ms:u32
          FAULT_EVENT (0x30)        <HBB>  4B  fault_code:u16, severity:u8, detail:u8
        """
        mt = frame.message_type
        p = frame.payload
        if mt == DROP_RELEASED:
            # 兼容两种格式：
            #   新版 DROP_RELEASED <IB>  5B  elapsed_ms:u32, quality:u8
            #   旧版 UAV_EVENT   <BI>  5B  phase:u8, elapsed_ms:u32
            # 区分依据：新版第一个字段是 elapsed_ms(u32)，值通常 > 0；
            #          旧版第一个字段是 phase(u8)，值在 0~18 之间。
            # 如果第一个字节 ≤ 18 且后 4 字节解出的 elapsed 合理，判为旧版 phase 事件。
            if len(p) != 5:
                raise ValueError(f"DROP_RELEASED payload must be 5 bytes, got {len(p)}")
            first_byte = p[0]
            if first_byte <= 18:
                # 旧版 UAV_EVENT <BI>：phase + elapsed_ms
                phase, elapsed_ms = struct.unpack("<BI", p)
                phase_name = self.UAV_PHASE_NAMES.get(phase, f"未知phase={phase}")
                print(
                    f"[UAV_EVENT] 阶段切换  seq={frame.seq} phase={phase}({phase_name}) elapsed={elapsed_ms}ms",
                    flush=True,
                )
                return f"阶段切换→{phase_name}"
            # 新版 DROP_RELEASED <IB>：elapsed_ms + quality
            elapsed_ms, quality = struct.unpack("<IB", p)
            print(
                f"[UAV_EVENT] 抛投释放  seq={frame.seq} elapsed={elapsed_ms}ms quality={quality}/100",
                flush=True,
            )
            return "抛投释放"
        if mt == TOUCHDOWN_CONFIRMED:
            if len(p) != 5:
                raise ValueError(f"TOUCHDOWN_CONFIRMED payload must be 5 bytes, got {len(p)}")
            elapsed_ms, confidence = struct.unpack("<IB", p)
            print(
                f"[UAV_EVENT] 触地确认  seq={frame.seq} elapsed={elapsed_ms}ms confidence={confidence}/100",
                flush=True,
            )
            return "触地确认"
        if mt == RETAKEOFF_STARTED:
            if len(p) != 4:
                raise ValueError(f"RETAKEOFF_STARTED payload must be 4 bytes, got {len(p)}")
            elapsed_ms = struct.unpack("<I", p)[0]
            print(
                f"[UAV_EVENT] 二次起飞  seq={frame.seq} elapsed={elapsed_ms}ms",
                flush=True,
            )
            return "二次起飞"
        if mt == MISSION_COMPLETE:
            if len(p) != 5:
                raise ValueError(f"MISSION_COMPLETE payload must be 5 bytes, got {len(p)}")
            result, elapsed_ms = struct.unpack("<BI", p)
            print(
                f"[UAV_EVENT] 任务完成  seq={frame.seq} result={result} elapsed={elapsed_ms}ms",
                flush=True,
            )
            return "任务完成"
        if mt == FAULT_EVENT:
            if len(p) != 4:
                raise ValueError(f"FAULT_EVENT payload must be 4 bytes, got {len(p)}")
            fault_code, severity, detail = struct.unpack("<HBB", p)
            print(
                f"[UAV_EVENT] 故障事件  seq={frame.seq} code={fault_code} severity={severity} detail={detail}",
                flush=True,
            )
            return "故障事件"
        # 未知事件类型，原样转发不解析
        print(
            f"[UAV_EVENT] 未知事件 type=0x{mt:02X} seq={frame.seq} len={len(p)}",
            flush=True,
        )
        return f"未知事件0x{mt:02X}"

    def _build_car_start_frame(self, seq: int) -> bytes:
        """组一帧 CAR_START。payload=<BI>，5字节。

        无人机端要求 flags=0x05：ACK_REQUIRED(0x01) | EVENT(0x04)。
        """
        payload = struct.pack("<BI", self.task_mode, CAR_SESSION_ID)
        return build_frame(
            CAR_START,
            payload,
            flags=ACK_REQUIRED | EVENT,
            source=CAR,
            destination=UAV,
            session_id=CAR_SESSION_ID,
            seq=seq,
            sender_ms=self.sender_ms(),
        )

    def _match_car_start_ack(self, frame: DcpFrame, sent_seq: int) -> Optional[int]:
        """判断帧是否为本次 CAR_START 的 ACK，返回 result 或 None。

        匹配条件（四要素缺一不可）：
          - source = UAV, message_type = ACK, flags 含 IS_ACK
          - frame.session_id == CAR_START.session_id (CAR_SESSION_ID)
          - acked_type == 0x03 (CAR_START)
          - acked_seq == CAR_START.seq (sent_seq)

        返回值：
          - None：不是本次 CAR_START 的 ACK
          - 0：无人机接受并起飞（应切换 session）
          - 非0：无人机拒绝（不切换 session，保持 session_id=0）
        """
        if frame.source != UAV or frame.message_type != ACK:
            return None
        if not (frame.flags & IS_ACK):
            return None
        if frame.session_id != CAR_SESSION_ID:
            return None
        if len(frame.payload) != 4:
            return None
        acked_type, acked_seq, result = struct.unpack("<BHB", frame.payload)
        if acked_type != CAR_START or acked_seq != sent_seq:
            return None
        return result

    def wait_for_uav_ready_and_start(self) -> bool:
        """发送 CAR_START 并等待无人机 ACK。

        两阶段超时：
          - 第一阶段：等 CAR_START_ACK_TIMEOUT_S 秒（默认 10s）。
          - 仍未收到则进入第二阶段：再等 CAR_START_EXTRA_WAIT_S 秒（默认 2s），
            期间继续重发 CAR_START。
          - 两阶段都超时后不再退出，直接进入主循环（session_id 保持 0，
            SESSION_VALID=0），CAR_POSITION 继续以 session_id=0 发送。

        收到匹配 ACK：
          - result==0：切换 session，返回 True 进入主循环
          - result!=0：不切换 session（保持 0），仍返回 True 进入主循环
        """
        start = time.monotonic()
        deadline1 = start + CAR_START_ACK_TIMEOUT_S
        deadline2 = deadline1 + CAR_START_EXTRA_WAIT_S
        next_send_at = start
        logging.info(
            "Sending CAR_START (task_mode=%d, session=0x%08X), waiting up to %.1fs for ACK...",
            self.task_mode, CAR_SESSION_ID, CAR_START_ACK_TIMEOUT_S,
        )
        sent_seq = -1
        car_start_raw: Optional[bytes] = None  # 缓存首次构建的完整原始帧
        send_count = 0
        extra_phase = False
        while not self.stop_event.is_set():
            now = time.monotonic()
            if now >= deadline2:
                # 两阶段都超时：不再退出，继续运行（session_id 保持 0）
                logging.warning(
                    "CAR_START ACK not received within %.1fs (+%.1fs extra, sent %d times, seq=%d). "
                    "Continuing with session_id=0.",
                    CAR_START_ACK_TIMEOUT_S, CAR_START_EXTRA_WAIT_S, send_count, sent_seq,
                )
                return True
            if not extra_phase and now >= deadline1:
                extra_phase = True
                logging.warning(
                    "CAR_START ACK not received within %.1fs; waiting %.1fs more...",
                    CAR_START_ACK_TIMEOUT_S, CAR_START_EXTRA_WAIT_S,
                )
            if now >= next_send_at:
                if car_start_raw is None:
                    # 首次：分配 seq、构建帧、保存原始字节（后续重发完全复用）
                    sent_seq = self.tx_seq.next()
                    car_start_raw = self._build_car_start_frame(sent_seq)
                    logging.info("CAR_START built once, seq=%d (saved %d bytes)", sent_seq, len(car_start_raw))
                # 重发：直接写缓存的原始帧，不重新分配 seq、不重新 build
                with self.uav_write_lock:
                    self.uav_serial.write(car_start_raw)
                    self.uav_serial.flush()
                send_count += 1
                logging.info("CAR_START sent #%d, seq=%d", send_count, sent_seq)
                next_send_at = now + CAR_START_RESEND_INTERVAL_S

            # 非阻塞读串口，处理 ACK 及其他早期帧。
            waiting = int(getattr(self.uav_serial, "in_waiting", 0))
            if waiting:
                incoming = self.uav_serial.read(max(1, min(1024, waiting)))
                for frame in self.parser.feed(incoming):
                    try:
                        result = self._match_car_start_ack(frame, sent_seq)
                        if result is not None:
                            if result == 0:
                                self.set_active_session(CAR_SESSION_ID)
                                logging.info(
                                    "CAR_START ACK received (seq=%d, result=0). "
                                    "UAV bound to session 0x%08X.",
                                    sent_seq, CAR_SESSION_ID,
                                )
                            else:
                                # 无人机拒绝：session 保持 0，SESSION_VALID=0。
                                # 仍进入主循环，CAR_POSITION 用 session_id=0 继续发送。
                                logging.warning(
                                    "CAR_START ACK received (seq=%d, result=%d). "
                                    "UAV rejected; keeping session_id=0.",
                                    sent_seq, result,
                                )
                            # 收到 ACK：切换蓝灯，表示进入任务状态
                            if self.rgb is not None:
                                self.rgb.blue()
                            return True
                        self.handle_frame(frame)
                    except ValueError as exc:
                        logging.warning("Rejected frame during CAR_START handshake: %s", exc)
            else:
                # 让出 CPU，避免忙等。
                self.stop_event.wait(0.02)
        return False

    def send_task_mode_to_ground(self) -> None:
        """向地面站发送一帧 CAR_START，让地面站知道当前任务模式（task_mode）。

        payload=<BI>，5字节：task_mode:u8, car_config_hash:u32
        地面站解析 task_mode 即可知道是任务1（抛投）还是任务2（动态起降）。
        连发 3 次提高可靠性，地面站收到任一帧即可。
        """
        if not GROUND_LINK_ENABLED:
            logging.info("Ground link disabled, skip sending CAR_START to ground.")
            return
        seq = self.tx_seq.next()
        frame = self.build_car_start_frame_for_ground(seq)
        for i in range(3):
            try:
                with self.ground_write_lock:
                    self.ground_serial.write(frame)
                    self.ground_serial.flush()
            except Exception:
                logging.exception("Failed to send CAR_START to ground (attempt %d/3)", i + 1)
            if i < 2:
                time.sleep(0.05)
        logging.info("CAR_START sent to ground station (task_mode=%d, 3x)", self.task_mode)

    def build_car_start_frame_for_ground(self, seq: int) -> bytes:
        """组一帧发往地面站的 CAR_START（含 task_mode）。"""
        payload = struct.pack("<BI", self.task_mode, CAR_SESSION_ID)
        return build_frame(
            CAR_START,
            payload,
            flags=0,
            source=CAR,
            destination=GROUND,
            session_id=CAR_SESSION_ID,
            seq=seq,
            sender_ms=self.sender_ms(),
        )

    def _handle_uav_event(self, frame: DcpFrame) -> None:
        """处理无人机事件帧。

        严格按 competition_2026_d/communication_protocol.md 第72-76行：
          - 每个事件是独立的 message_type（DROP_RELEASED / TOUCHDOWN_CONFIRMED /
            RETAKEOFF_STARTED / MISSION_COMPLETE / FAULT_EVENT）
          - 所有事件都回 ACK（无人机端要求收到 ACK 才停止重发）
          - 按各自 payload 格式解析并打印中文事件名
          - 去重后转发给地面站

        去重键 = (session_id, message_type, seq)，使用 uint16 seq。
        重复帧仍回 ACK（无人机可能没收到上一次 ACK），但业务不重复执行。
        """
        # 入口日志：确认收到了事件帧
        is_dup = (frame.session_id, frame.message_type, frame.seq) in self.seen_events
        logging.info(
            "UAV_EVENT received type=0x%02X seq=%d session=0x%08X dup=%s",
            frame.message_type, frame.seq, frame.session_id, is_dup,
        )
        # 所有无人机事件都回 ACK，否则无人机会一直重发。
        # 重复帧也回 ACK：无人机可能没收到上一次 ACK。
        self._ack_uav_event(frame)
        # 去重：同一 (session, type, seq) 只处理一次业务
        event_key = (frame.session_id, frame.message_type, frame.seq)
        if event_key in self.seen_events:
            return
        self.seen_events.add(event_key)
        self.set_active_session(frame.session_id)
        # 解析并打印事件内容（中文）
        self._parse_and_log_uav_event(frame)
        # 转发给地面站：只改 destination 和 CRC，其余原样保留
        relayed = build_frame(
            frame.message_type,
            frame.payload,
            flags=frame.flags,
            source=frame.source,
            destination=GROUND,
            session_id=frame.session_id,
            seq=frame.seq,
            sender_ms=frame.sender_ms,
        )
        self._write_ground(relayed)

    # 所有无人机事件类型集合（用于 handle_frame 分发）
    UAV_EVENT_TYPES = (DROP_RELEASED, TOUCHDOWN_CONFIRMED, RETAKEOFF_STARTED,
                       MISSION_COMPLETE, FAULT_EVENT)

    def handle_frame(self, frame: DcpFrame) -> None:
        if frame.source != UAV or frame.destination not in (CAR, BROADCAST):
            return
        if frame.message_type == UAV_STATE:
            self._handle_uav_state(frame)
        elif frame.message_type in self.UAV_EVENT_TYPES:
            self._handle_uav_event(frame)
        elif frame.message_type == UAV_READY:
            relayed = build_frame(
                frame.message_type,
                frame.payload,
                flags=frame.flags,
                source=frame.source,
                destination=GROUND,
                session_id=frame.session_id,
                seq=frame.seq,
                sender_ms=frame.sender_ms,
            )
            self._write_ground(relayed)
        elif frame.message_type == ACK:
            # 主循环阶段收到的 ACK（非 CAR_START 握手阶段），静默忽略
            pass
        else:
            # 未知 message_type：记录警告，避免静默丢弃导致无人机收不到 ACK
            logging.warning(
                "Unknown UAV frame type=0x%02X seq=%d session=0x%08X dest=0x%02X len=%d (no ACK sent)",
                frame.message_type, frame.seq, frame.session_id,
                frame.destination, len(frame.payload),
            )

    def run_forever(self) -> None:
        workers = [
            threading.Thread(target=self._car_position_loop, name="car-position-tx", daemon=True),
            threading.Thread(target=self._car_state_loop, name="car-state-tx", daemon=True),
            threading.Thread(target=self._uav_state_relay_loop, name="uav-state-relay", daemon=True),
        ]
        for worker in workers:
            worker.start()

        next_report = time.monotonic() + 1.0
        try:
            while not self.stop_event.is_set():
                waiting = int(getattr(self.uav_serial, "in_waiting", 0))
                incoming = self.uav_serial.read(max(1, min(1024, waiting or 1)))
                if incoming:
                    for frame in self.parser.feed(incoming):
                        try:
                            self.handle_frame(frame)
                        except ValueError as exc:
                            logging.warning("Rejected UAV frame: %s", exc)
                if time.monotonic() >= next_report:
                    logging.info(
                        "car->uav=%d, car_state->uav=%d, uav_state->ground=%d, crc_errors=%d",
                        self.car_position_count,
                        self.car_state_count,
                        self.uav_state_relay_count,
                        self.parser.bad_crc,
                    )
                    next_report += 1.0
        finally:
            self.stop_event.set()
            for worker in workers:
                worker.join(timeout=1.0)
            self.uav_serial.close()
            self.ground_serial.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Car Raspberry Pi DCP coordinate gateway")
    parser.add_argument("--uav-port", default=UAV_SERIAL_PORT)
    parser.add_argument("--ground-port", default=GROUND_SERIAL_PORT)
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    # --no-button: 跳过 GPIO 按钮等待，启动后立即开始 CAR_START 握手（便于自动化测试）。
    parser.add_argument("--no-button", action="store_true",
                        help="skip GPIO button wait and start handshake immediately")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.uav_port == args.ground_port:
        raise SystemExit("UAV and ground serial ports must be different")
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Missing pyserial: python3 -m pip install -r requirements.txt") from exc

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    # RGB 状态指示灯：绿灯=运行等待按钮，红灯=按钮按下等ACK，蓝灯=收到ACK进入任务
    rgb = RGBIndicator(RGB_R_PIN, RGB_G_PIN, RGB_B_PIN)
    uav_serial = serial.Serial(args.uav_port, args.baud, timeout=0.05, write_timeout=0.5)
    if GROUND_LINK_ENABLED:
        ground_serial = serial.Serial(args.ground_port, GROUND_SERIAL_BAUD, timeout=0.05, write_timeout=0.5)
        logging.info("Ground link enabled on %s @ %d baud", args.ground_port, GROUND_SERIAL_BAUD)
    else:
        ground_serial = DummySerial()
        logging.warning("Ground link DISABLED (GROUND_LINK_ENABLED=False), using DummySerial.")
    gateway = CarGateway(uav_serial, ground_serial, rgb=rgb)

    def request_stop(_signum: int, _frame: object) -> None:
        gateway.stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    # 限时运行：到 MAX_RUN_TIME_S 秒自动停止（含按钮等待+握手+主循环全过程）。
    if MAX_RUN_TIME_S > 0:
        def _timeout_killer() -> None:
            deadline = time.monotonic() + MAX_RUN_TIME_S
            while time.monotonic() < deadline:
                if gateway.stop_event.is_set():
                    return
                time.sleep(0.5)
            if not gateway.stop_event.is_set():
                logging.info("Max run time %.0fs reached, stopping...", MAX_RUN_TIME_S)
                gateway.stop_event.set()
        threading.Thread(target=_timeout_killer, name="timeout-killer", daemon=True).start()
    logging.info(
        "Gateway started: UAV=%s, ground=%s, baud=%d, car->UAV=%.1f Hz",
        args.uav_port,
        args.ground_port,
        args.baud,
        CAR_TO_UAV_HZ,
    )
    # 程序启动：亮绿灯，表示正在等待按钮按下
    rgb.green()

    # 启动按钮：按下 GPIO24（物理18）后才开始发 CAR_START 握手。
    # 单击=任务1（抛投）：BCM23 持续 HIGH 1.0s
    # 双击=任务2（动态起降）：BCM23 连发 5 次脉冲（每次 HIGH 100ms + LOW 100ms）
    # --no-button 可跳过等待，用默认任务模式；测试环境（无 RPi.GPIO）用 Enter 键模拟。
    button_pin = None if args.no_button else CAR_START_BUTTON_PIN
    ok, task_mode = wait_for_start_button(
        button_pin,
        stop_event=gateway.stop_event,
        feedback_pin=CAR_START_BUTTON_FEEDBACK_PIN,
        feedback_hold_s=CAR_START_BUTTON_FEEDBACK_HOLD_S,
        rgb=rgb,
    )
    if not ok:
        logging.info("Aborted while waiting for start button.")
        rgb.off()
        uav_serial.close()
        ground_serial.close()
        return 0
    # 根据按钮单击/双击结果设置任务模式，CAR_START 会带上该 task_mode
    gateway.task_mode = task_mode
    logging.info("Task mode selected: %d (%s)", task_mode,
                 "抛投" if task_mode == 1 else "动态起降" if task_mode == 2 else "未知")
    # 启动握手：发 CAR_START 等无人机 ACK。
    # 两阶段超时（10s + 2s）后不再退出，直接进入主循环（session_id=0）。
    # 仅 Ctrl+C（stop_event）触发时才会中止退出。
    if not gateway.wait_for_uav_ready_and_start():
        logging.error("CAR_START handshake aborted (interrupted).")
        gateway.stop_event.set()
        rgb.off()
        uav_serial.close()
        ground_serial.close()
        return 1
    logging.info("Handshake complete, entering main loop.")
    # 握手完成后向地面站发送 CAR_START（含 task_mode），让地面站知道当前任务。
    # 连发 3 次提高可靠性，地面站收到任一帧即可。
    gateway.send_task_mode_to_ground()
    try:
        gateway.run_forever()
    finally:
        rgb.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
