"""
cyber_cam_reader.py — Pi 端 UART 接收器

从 CyberCAM 板（核桃派）的串口读取检测结果，解析为 Detection 对象。

非阻塞设计：
  - read() 返回最新可用结果，或 None（无新帧）
  - start()/stop() 管理串口线程
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from typing import Optional

from .servo_controller import Detection


logger = logging.getLogger(__name__)


class CyberCamReader:
    """
    串口读取器。在后台线程中持续读取 UART，维护最新检测结果。

    用法：

        reader = CyberCamReader("/dev/ttyS3")
        if reader.start():
            while running:
                det = reader.read()
                if det is not None:
                    servo.tick(det, altitude_m)
            reader.stop()

    Parameters
    ----------
    port      : 串口设备路径
    baud      : 波特率（默认 115200）
    timeout   : 串口读取超时（秒），越小读取越灵敏
    queue_size : 保留最近 N 帧结果用于统计
    """

    def __init__(
        self,
        port: str = "/dev/ttyS3",
        baud: int = 115200,
        timeout: float = 0.01,
        queue_size: int = 30,
    ) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._serial = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._latest: Optional[Detection] = None
        self._queue: deque = deque(maxlen=queue_size)
        self._lock = threading.Lock()

    # ── 生命周期 ─────────────────────────────────────────────────── #

    def start(self) -> bool:
        """打开串口并启动读取线程。"""
        import serial
        try:
            self._serial = serial.Serial(
                self._port, self._baud, timeout=self._timeout,
            )
        except Exception as e:
            logger.error("[CCAM] Cannot open %s: %s", self._port, e)
            return False

        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop, daemon=True, name="cybercam-reader",
        )
        self._thread.start()
        logger.info("[CCAM] Reader started on %s @ %d", self._port, self._baud)
        return True

    def stop(self) -> None:
        self._running = False
        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        logger.info("[CCAM] Reader stopped")

    # ── 读取接口 ─────────────────────────────────────────────────── #

    def read(self) -> Optional[Detection]:
        """
        返回最新检测结果（线程安全，非阻塞）。
        连续调用会返回同一结果，直到收到新帧。
        """
        with self._lock:
            return self._latest

    def stats(self) -> dict:
        """最近 N 帧的统计信息（诊断用）。"""
        with self._lock:
            q = list(self._queue)
        if not q:
            return {"frames": 0}
        found_count = sum(1 for d in q if d.found)
        return {
            "frames": len(q),
            "found_ratio": found_count / len(q),
        }

    # ── 内部 ─────────────────────────────────────────────────────── #

    def _read_loop(self) -> None:
        buf = b""
        while self._running and self._serial is not None:
            try:
                # 读原始字节
                data = self._serial.read(64)
                if not data:
                    continue
                buf += data

                # 按换行符分割
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    self._parse_and_store(line)

            except Exception as e:
                logger.warning("[CCAM] Read error: %s", e)
                time.sleep(0.01)

    def _parse_and_store(self, line: bytes) -> None:
        """解析一行 ASCII 协议并存储。"""
        try:
            text = line.decode("ascii", errors="replace")
            parts = text.split(",")
            if len(parts) != 3:
                return
            dx = int(parts[0])
            dy = int(parts[1])
            found = parts[2] == "1"
        except (ValueError, UnicodeDecodeError):
            return

        det = Detection(found=found, dx_px=dx, dy_px=dy)

        with self._lock:
            self._latest = det
            self._queue.append(det)
