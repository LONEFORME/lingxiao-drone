"""
Pi ↔ K230 双向串口通信客户端
=============================
协议帧 (二进制, AA头+FF尾):
  Pi→K230: AA 10 grid_idx FF                       — 启动检测(4B)
  K230→Pi: AA 20 grid_idx cls cnt total conf FF    — 检测结果(8B)
  Pi→K230: AA 11 grid_idx FF                       — ACK确认(4B)

物理层: /dev/ttyS3, 115200 baud, 8N1
"""

import serial
import threading
import time
from Lcode.Logger import logger

FRAME_HEAD = 0xAA
CMD_START  = 0x10
CMD_ACK    = 0x11
CMD_RESULT = 0x20
RESULT_LEN = 8  # AA + CMD + grid_idx + cls + cnt + total + conf + FF
DEBUG = True   # 调试开关


class K230Client:
    """Pi端 K230 通信客户端"""

    def __init__(self, port="/dev/ttyS3", baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)
        self.result = None          # (grid_idx, cls, cnt, total, conf)
        self.result_event = threading.Event()
        self._running = True
        self._listen_thread = threading.Thread(target=self._listen, daemon=True)
        self._listen_thread.start()
        logger.info(f"K230已连接: {port} @ {baudrate}")

    def _listen(self):
        """守护线程: 接收 K230 的 RESULT 帧（AA...FF 对齐）"""
        buf = b''
        while self._running:
            try:
                chunk = self.ser.read()
                if chunk:
                    if DEBUG: logger.info(f"K230 RX raw: {chunk.hex()} buf+={len(chunk)}B total={len(buf)+len(chunk)}B")
                    buf += chunk
                while True:
                    p = buf.find(bytes([FRAME_HEAD]))
                    if p < 0:
                        buf = buf[-(RESULT_LEN - 1):]
                        break
                    if p > 0:
                        if DEBUG: logger.info(f"K230 RX skip {p}B noise: {buf[:p].hex()}")
                        buf = buf[p:]
                    if len(buf) < RESULT_LEN:
                        break
                    if buf[RESULT_LEN - 1] != 0xFF:
                        if DEBUG: logger.info(f"K230 RX misalign: tail={buf[RESULT_LEN-1]:02x} seek from {buf[:RESULT_LEN].hex()}")
                        buf = buf[1:]
                        continue
                    cmd = buf[1]
                    if cmd == CMD_RESULT:
                        grid_idx = buf[2]
                        cls_id = buf[3]
                        best_cnt = buf[4]
                        total_dets = buf[5]
                        avg_conf = buf[6]
                        logger.info(f"K230 RESULT parsed: grid={grid_idx} cls={cls_id} cnt={best_cnt} total={total_dets} conf={avg_conf}")
                        self.result = (grid_idx, cls_id, best_cnt, total_dets, avg_conf)
                        self.result_event.set()
                    buf = buf[RESULT_LEN:]
            except (OSError, serial.SerialException):
                time.sleep(0.01)
            except Exception:
                time.sleep(0.01)

    def send_start(self, grid_idx):
        """发送 START 帧: AA 10 grid_idx FF"""
        self.result = None
        self.result_event.clear()
        frame = bytes([FRAME_HEAD, CMD_START, grid_idx & 0xFF, 0xFF])
        self.ser.write(frame)
        logger.info(f"K230 START -> grid_idx={grid_idx}")
        if DEBUG: logger.info(f"K230 TX: {frame.hex()}")

    def send_ack(self, grid_idx):
        """发送 ACK 帧: AA 11 grid_idx FF"""
        frame = bytes([FRAME_HEAD, CMD_ACK, grid_idx & 0xFF, 0xFF])
        self.ser.write(frame)
        logger.info(f"K230 ACK   -> grid_idx={grid_idx}")
        if DEBUG: logger.info(f"K230 TX: {frame.hex()}")

    def poll_result(self):
        """非阻塞轮询"""
        if self.result_event.wait(timeout=0):
            self.result_event.clear()
            return self.result
        return None

    def close(self):
        """关闭串口和线程"""
        self._running = False
        time.sleep(0.05)
        if self.ser.is_open:
            self.ser.close()
        logger.info("K230串口已关闭")
