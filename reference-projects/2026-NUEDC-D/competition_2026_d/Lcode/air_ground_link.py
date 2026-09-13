"""基于/dev/bt_serial的DCP v1双向、非阻塞链路。"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    Frame,
    MessageType,
    StreamParser,
    encode_frame,
    pack_payload,
    unpack_payload,
)


LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class LinkConfig:
    port: str = "/dev/bt_serial"
    baudrate: int = 115200
    queue_size: int = 128
    read_timeout_s: float = 0.02
    write_timeout_s: float = 0.20
    ack_timeout_s: float = 0.12
    max_retries: int = 4
    max_consecutive_tx_errors: int = 3
    seen_events: int = 128


@dataclass
class LinkStats:
    tx_frames: int = 0
    tx_ack_frames: int = 0
    rx_ack_success: int = 0
    rx_ack_negative: int = 0
    rx_ack_late: int = 0
    rx_ack_duplicate: int = 0
    rx_ack_unmatched: int = 0
    rx_bytes: int = 0
    rx_frames: int = 0
    tx_dropped: int = 0
    rx_rejected: int = 0
    rx_wrong_dest: int = 0
    retries: int = 0
    ack_timeouts: int = 0
    duplicate_events: int = 0
    io_errors: int = 0
    current_tx_error_streak: int = 0
    max_tx_error_streak: int = 0
    last_tx_hex: str = ""
    last_ack_tx_hex: str = ""
    last_rx_hex: str = ""
    last_frame_type: int | None = None
    last_frame_source: int | None = None
    last_frame_dest: int | None = None


@dataclass
class _Pending:
    frame: Frame
    raw: bytes
    deadline: float
    retries_left: int


class AirGroundLink:
    """收发线程不进入30ms飞行循环；回调只投递已校验帧。"""

    def __init__(self, config: LinkConfig | None = None, serial_factory=None) -> None:
        self.config = config or LinkConfig()
        self._serial_factory = serial_factory
        self._serial = None
        self._parser = StreamParser()
        self._tx_queue: queue.Queue[bytes] = queue.Queue(maxsize=self.config.queue_size)
        self._rx_queue: queue.Queue[Frame] = queue.Queue(maxsize=self.config.queue_size)
        self._callback_queue: queue.Queue[Frame] = queue.Queue(maxsize=self.config.queue_size)
        self._pending: dict[tuple[int, int, int], _Pending] = {}
        self._pending_lock = threading.Lock()
        self._ack_history_order: deque[tuple[int, int, int]] = deque()
        self._ack_history: dict[tuple[int, int, int], str] = {}
        self._callbacks: list[Callable[[Frame], None]] = []
        self._running = threading.Event()
        self._threads: list[threading.Thread] = []
        self._sequence = 0
        self._seen_order: deque[tuple[int, int, int, int]] = deque()
        self._seen_set: set[tuple[int, int, int, int]] = set()
        self.stats = LinkStats()

    def add_callback(self, callback: Callable[[Frame], None]) -> None:
        self._callbacks.append(callback)

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    def start(self) -> bool:
        if self._running.is_set():
            return True
        try:
            factory = self._serial_factory
            if factory is None:
                import serial
                factory = serial.Serial
            self._serial = factory(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=self.config.read_timeout_s,
                write_timeout=self.config.write_timeout_s,
            )
        except Exception as exc:
            LOG.error("蓝牙链路打开失败(%s): %s", self.config.port, exc)
            self.stats.io_errors += 1
            return False
        self._running.set()
        self._threads = [
            threading.Thread(target=self._rx_worker, daemon=True, name="dcp-rx"),
            threading.Thread(target=self._tx_worker, daemon=True, name="dcp-tx"),
            threading.Thread(target=self._dispatch_worker, daemon=True, name="dcp-dispatch"),
        ]
        for thread in self._threads:
            thread.start()
        return True

    def close(self) -> None:
        self._running.clear()
        for thread in self._threads:
            thread.join(timeout=0.5)
        self._threads.clear()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def publish(
        self,
        message_type: MessageType | int,
        payload: bytes,
        *,
        session_id: int,
        dest: Device | int = Device.BROADCAST,
        flags: Flag | int = Flag.NONE,
        sender_ms: int | None = None,
    ) -> int | None:
        if not self._running.is_set():
            return None
        seq = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        frame = Frame(
            message_type=int(message_type), flags=int(flags), source=Device.UAV,
            dest=int(dest), session_id=session_id, seq=seq,
            sender_ms=(int(time.monotonic() * 1000) if sender_ms is None else sender_ms) & 0xFFFFFFFF,
            payload=bytes(payload),
        )
        raw = encode_frame(frame)
        pending_key = None
        if int(flags) & int(Flag.ACK_REQUIRED):
            pending_key = (int(message_type), seq, session_id)
            with self._pending_lock:
                self._pending[pending_key] = _Pending(
                    frame, raw, time.monotonic() + self.config.ack_timeout_s,
                    self.config.max_retries,
                )
        if not self._enqueue(raw):
            if pending_key is not None:
                with self._pending_lock:
                    self._pending.pop(pending_key, None)
            return None
        return seq

    def get_nowait(self) -> Frame | None:
        try:
            return self._rx_queue.get_nowait()
        except queue.Empty:
            return None

    def wait_pending(self, timeout_s: float = 0.6) -> bool:
        """等待可靠事件收到ACK；仅供任务结束清理路径使用。"""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            with self._pending_lock:
                if not self._pending:
                    return True
            time.sleep(0.01)
        with self._pending_lock:
            return not self._pending

    def acknowledge(self, received: Frame, result: int = 0) -> int | None:
        """业务层校验完成后回复ACK；result=0表示接受。"""
        payload = pack_payload(
            MessageType.ACK,
            (received.message_type, received.seq, int(result) & 0xFF),
        )
        return self.publish(
            MessageType.ACK,
            payload,
            session_id=received.session_id,
            dest=received.source,
            flags=Flag.IS_ACK,
        )

    def _enqueue(self, raw: bytes) -> bool:
        try:
            self._tx_queue.put_nowait(raw)
            return True
        except queue.Full:
            try:
                self._tx_queue.get_nowait()
                self._tx_queue.task_done()
            except queue.Empty:
                pass
            self.stats.tx_dropped += 1
            try:
                self._tx_queue.put_nowait(raw)
                return True
            except queue.Full:
                self.stats.tx_dropped += 1
                return False

    def _rx_worker(self) -> None:
        while self._running.is_set():
            try:
                data = self._serial.read(256)
                if not data:
                    continue
                self.stats.rx_bytes += len(data)
                self.stats.last_rx_hex = bytes(data[-32:]).hex()
                frames = self._parser.feed(data)
                self.stats.rx_rejected = self._parser.rejected
                for frame in frames:
                    self.stats.last_frame_type = int(frame.message_type)
                    self.stats.last_frame_source = int(frame.source)
                    self.stats.last_frame_dest = int(frame.dest)
                    if frame.dest not in (Device.UAV, Device.BROADCAST):
                        self.stats.rx_wrong_dest += 1
                        continue
                    self.stats.rx_frames += 1
                    self._handle_frame(frame)
            except Exception as exc:
                self.stats.io_errors += 1
                LOG.error("DCP接收失败，关闭链路: %s", exc)
                self._running.clear()

    def _handle_frame(self, frame: Frame) -> None:
        if frame.message_type == MessageType.ACK or frame.flags & Flag.IS_ACK:
            try:
                acked_type, acked_seq, result = unpack_payload(
                    MessageType.ACK, frame.payload
                )
            except ValueError:
                self.stats.rx_rejected += 1
                return
            key = (acked_type, acked_seq, frame.session_id)
            with self._pending_lock:
                pending = self._pending.pop(key, None)
                previous_resolution = self._ack_history.get(key)
                if pending is not None:
                    resolution = "success" if result == 0 else "negative"
                    self._remember_ack_resolution_locked(key, resolution)
                elif previous_resolution == "timeout":
                    self._remember_ack_resolution_locked(key, "late")
            type_name = _message_type_name(acked_type)
            if pending is None and previous_resolution == "timeout":
                self.stats.rx_ack_late += 1
                LOG.warning(
                    "DCP可靠消息收到迟到ACK: type=%s(0x%02X), seq=%d, "
                    "session=%d, result=%d",
                    type_name,
                    acked_type,
                    acked_seq,
                    frame.session_id,
                    result,
                )
            elif pending is None and previous_resolution is not None:
                self.stats.rx_ack_duplicate += 1
                LOG.debug(
                    "DCP收到重复ACK: type=%s(0x%02X), seq=%d, "
                    "session=%d, result=%d",
                    type_name,
                    acked_type,
                    acked_seq,
                    frame.session_id,
                    result,
                )
            elif pending is None:
                self.stats.rx_ack_unmatched += 1
                LOG.warning(
                    "DCP收到无法匹配的ACK: type=%s(0x%02X), seq=%d, "
                    "session=%d, result=%d",
                    type_name,
                    acked_type,
                    acked_seq,
                    frame.session_id,
                    result,
                )
            elif result == 0:
                self.stats.rx_ack_success += 1
                LOG.info(
                    "DCP可靠消息ACK成功: type=%s(0x%02X), seq=%d, "
                    "session=%d",
                    type_name,
                    acked_type,
                    acked_seq,
                    frame.session_id,
                )
            else:
                self.stats.rx_ack_negative += 1
                LOG.warning(
                    "DCP可靠消息收到负ACK: type=%s(0x%02X), seq=%d, "
                    "session=%d, result=%d",
                    type_name,
                    acked_type,
                    acked_seq,
                    frame.session_id,
                    result,
                )
            return
        event_key = (frame.source, frame.session_id, frame.message_type, frame.seq)
        deferred_start_ack = (
            int(frame.message_type) == int(MessageType.CAR_START)
        )
        duplicate = event_key in self._seen_set
        if frame.flags & Flag.ACK_REQUIRED and not deferred_start_ack:
            self.acknowledge(frame)
        if duplicate and not deferred_start_ack:
            self.stats.duplicate_events += 1
            return
        if (
            not deferred_start_ack
            and (frame.flags & Flag.EVENT or frame.flags & Flag.ACK_REQUIRED)
        ):
            self._remember(event_key)
        try:
            self._rx_queue.put_nowait(frame)
        except queue.Full:
            try:
                self._rx_queue.get_nowait()
                self._rx_queue.task_done()
            except queue.Empty:
                pass
            self._rx_queue.put_nowait(frame)
        if self._callbacks:
            try:
                self._callback_queue.put_nowait(frame)
            except queue.Full:
                try:
                    self._callback_queue.get_nowait()
                    self._callback_queue.task_done()
                except queue.Empty:
                    pass
                self._callback_queue.put_nowait(frame)

    def _remember(self, key: tuple[int, int, int, int]) -> None:
        self._seen_order.append(key)
        self._seen_set.add(key)
        while len(self._seen_order) > self.config.seen_events:
            self._seen_set.discard(self._seen_order.popleft())

    def _tx_worker(self) -> None:
        consecutive_errors = 0
        while self._running.is_set() or not self._tx_queue.empty():
            self._service_retries()
            try:
                raw = self._tx_queue.get(timeout=0.02)
            except queue.Empty:
                continue
            try:
                self._serial.write(raw)
                self.stats.tx_frames += 1
                consecutive_errors = 0
                self.stats.current_tx_error_streak = 0
                self.stats.last_tx_hex = raw.hex()
                if len(raw) > 2 and raw[2] == int(MessageType.ACK):
                    self.stats.tx_ack_frames += 1
                    self.stats.last_ack_tx_hex = raw.hex()
            except Exception as exc:
                self.stats.io_errors += 1
                self.stats.tx_dropped += 1
                consecutive_errors += 1
                self.stats.current_tx_error_streak = consecutive_errors
                self.stats.max_tx_error_streak = max(
                    self.stats.max_tx_error_streak,
                    consecutive_errors,
                )
                if (
                    consecutive_errors
                    >= self.config.max_consecutive_tx_errors
                ):
                    LOG.error(
                        "DCP连续发送失败%d次，关闭链路: %s",
                        consecutive_errors,
                        exc,
                    )
                    self._running.clear()
                else:
                    LOG.warning(
                        "DCP瞬时发送失败，丢弃当前帧并保持链路"
                        "（连续%d/%d次）: %s",
                        consecutive_errors,
                        self.config.max_consecutive_tx_errors,
                        exc,
                    )
            finally:
                self._tx_queue.task_done()

    def _service_retries(self) -> None:
        now = time.monotonic()
        resend: list[bytes] = []
        with self._pending_lock:
            for key, pending in list(self._pending.items()):
                if now < pending.deadline:
                    continue
                if pending.retries_left <= 0:
                    self._pending.pop(key, None)
                    self._remember_ack_resolution_locked(key, "timeout")
                    self.stats.ack_timeouts += 1
                    LOG.warning(
                        "DCP可靠消息ACK重试耗尽: type=%s(0x%02X), "
                        "seq=%d, session=%d, attempts=%d",
                        _message_type_name(pending.frame.message_type),
                        pending.frame.message_type,
                        pending.frame.seq,
                        pending.frame.session_id,
                        self.config.max_retries + 1,
                    )
                    continue
                pending.retries_left -= 1
                pending.deadline = now + self.config.ack_timeout_s
                resend.append(pending.raw)
                self.stats.retries += 1
        for raw in resend:
            self._enqueue(raw)

    def _remember_ack_resolution_locked(
        self,
        key: tuple[int, int, int],
        resolution: str,
    ) -> None:
        if key not in self._ack_history:
            self._ack_history_order.append(key)
        self._ack_history[key] = resolution
        while len(self._ack_history_order) > self.config.seen_events:
            oldest = self._ack_history_order.popleft()
            self._ack_history.pop(oldest, None)

    def _dispatch_worker(self) -> None:
        while self._running.is_set():
            try:
                frame = self._callback_queue.get(timeout=0.02)
            except queue.Empty:
                continue
            try:
                for callback in tuple(self._callbacks):
                    try:
                        callback(frame)
                    except Exception:
                        LOG.exception("DCP回调异常，已隔离")
            finally:
                self._callback_queue.task_done()


def _message_type_name(message_type: int) -> str:
    try:
        return MessageType(int(message_type)).name
    except ValueError:
        return "UNKNOWN"
