"""通过/dev/bt_serial发送盘点广播。

训练阶段没有接收端：发送在线程中完成，永不等待ACK，串口异常只记录并禁用链路，
不能阻塞飞行状态机。未来可靠模式可在同一帧格式上扩展ACK。
"""

import binascii
import json
import os
import queue
import struct
import threading
from collections.abc import Mapping
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import serial

from Lcode.Logger import logger


FRAME_HEAD = 0xAA
FRAME_TAIL = 0xFF
PROTOCOL_VERSION = 1
MAX_PAYLOAD_BYTES = 4096


class GroundMessageType(IntEnum):
    HEARTBEAT = 0x01
    STATE = 0x02
    TARGET_ID = 0x03
    ROUTE = 0x04
    INVENTORY_RESULT = 0x05
    MISSION_SUMMARY = 0x06
    FAULT = 0x07


@dataclass(frozen=True)
class GroundLinkConfig:
    mode: str = "broadcast"
    port: str = "/dev/bt_serial"
    baudrate: int = 115200
    queue_size: int = 128
    write_timeout_s: float = 0.05

    def __post_init__(self):
        if self.mode not in {"off", "broadcast", "reliable"}:
            raise ValueError("DRONE_GROUND_MODE只能是off/broadcast/reliable")
        if not self.port:
            raise ValueError("地面站端口不能为空")
        if self.baudrate <= 0:
            raise ValueError("地面站波特率必须为正数")
        if not 1 <= self.queue_size <= 4096:
            raise ValueError("广播队列长度必须在[1,4096]内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        return cls(
            mode=env.get("DRONE_GROUND_MODE", "broadcast").strip().lower(),
            port=env.get("DRONE_GROUND_PORT", "/dev/bt_serial").strip(),
            baudrate=int(env.get("DRONE_GROUND_BAUD", "115200")),
            queue_size=int(env.get("DRONE_GROUND_QUEUE_SIZE", "128")),
            write_timeout_s=float(env.get("DRONE_GROUND_WRITE_TIMEOUT_S", "0.05")),
        )


def encode_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    payload = bytes(payload)
    if not 0 <= int(message_type) <= 0xFF:
        raise ValueError("消息类型超出u8范围")
    if not 0 <= int(sequence) <= 0xFFFF:
        raise ValueError("序号超出u16范围")
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError("广播负载过长")
    body = struct.pack(
        "<BBHH", PROTOCOL_VERSION, int(message_type), int(sequence), len(payload)
    ) + payload
    crc = binascii.crc_hqx(body, 0xFFFF)
    return bytes([FRAME_HEAD]) + body + struct.pack("<H", crc) + bytes([FRAME_TAIL])


def decode_frame(frame: bytes):
    frame = bytes(frame)
    if len(frame) < 10 or frame[0] != FRAME_HEAD or frame[-1] != FRAME_TAIL:
        raise ValueError("广播帧头尾无效")
    version, message_type, sequence, length = struct.unpack("<BBHH", frame[1:7])
    expected_length = 1 + 6 + length + 2 + 1
    if len(frame) != expected_length:
        raise ValueError("广播帧长度无效")
    body = frame[1 : 7 + length]
    expected_crc = struct.unpack("<H", frame[7 + length : 9 + length])[0]
    if binascii.crc_hqx(body, 0xFFFF) != expected_crc:
        raise ValueError("广播帧CRC错误")
    if version != PROTOCOL_VERSION:
        raise ValueError(f"不支持的协议版本: {version}")
    return message_type, sequence, frame[7 : 7 + length]


class BroadcastGroundLink:
    def __init__(self, config: GroundLinkConfig = None, serial_factory=None):
        self.config = config or GroundLinkConfig.from_env()
        self._serial_factory = serial_factory or serial.Serial
        self._serial = None
        self._queue = queue.Queue(maxsize=self.config.queue_size)
        self._thread = None
        self._running = False
        self._sequence = 0
        self.sent_count = 0
        self.dropped_count = 0
        self.error_count = 0

    @property
    def available(self) -> bool:
        return self.config.mode == "off" or self._serial is not None

    def start(self) -> bool:
        if self.config.mode == "off":
            logger.info("地面广播已关闭")
            return True
        if self.config.mode == "reliable":
            logger.warning("可靠地面站模式尚未实现，暂按非阻塞广播模式运行")
        try:
            self._serial = self._serial_factory(
                port=self.config.port,
                baudrate=self.config.baudrate,
                timeout=0,
                write_timeout=self.config.write_timeout_s,
            )
        except Exception as exc:
            self._serial = None
            self.error_count += 1
            logger.error(f"地面广播端口打开失败({self.config.port})，飞行继续: {exc}")
            return False
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True, name="ground-broadcast")
        self._thread.start()
        logger.info(f"地面广播已启动: {self.config.port} @ {self.config.baudrate}")
        return True

    def publish(self, message_type: GroundMessageType, payload) -> Optional[int]:
        if self.config.mode == "off" or self._serial is None:
            return None
        if isinstance(payload, bytes):
            payload_bytes = payload
        else:
            payload_bytes = json.dumps(
                payload, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        frame = encode_frame(message_type, sequence, payload_bytes)
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            # 飞行控制优先：队列满时丢弃最旧广播，为最新状态让路。
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            self.dropped_count += 1
            try:
                self._queue.put_nowait(frame)
            except queue.Full:
                self.dropped_count += 1
                return None
        return sequence

    def _worker(self):
        while self._running or not self._queue.empty():
            try:
                frame = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self._serial.write(frame)
                self.sent_count += 1
            except Exception as exc:
                self.error_count += 1
                logger.error(f"地面广播写入失败，链路禁用但飞行继续: {exc}")
                self._running = False
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            finally:
                self._queue.task_done()

    def close(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception as exc:
                logger.error(f"关闭地面广播串口失败: {exc}")
            self._serial = None
