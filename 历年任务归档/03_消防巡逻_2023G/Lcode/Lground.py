"""无人机→消防车透明UART广播。见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md
"通信协议"一节。参照 original/Lcode/Lprotocol.py 的 Serial_dmz 透明UART模式
(AA...FF帧、独立发送线程)，但帧内容从cls/cnt改为坐标+帧类型：
  帧类型0=心跳位置帧(1Hz，满足基本要求(3)"每秒1次位置坐标")
  帧类型1=火情帧(一次性，HOVER_DROP阶段触发)
坐标单位cm(int16小端有符号)，比dm(赛题原始单位)精度更高，避免消防车侧计算/
显示巡逻航迹曲线(基本要求(4))出现"阶梯感"(见设计文档通信协议一节)。
"""
import threading
import time

import serial

from Lcode.Logger import logger

FRAME_TYPE_POSITION = 0x00
FRAME_TYPE_FIRE = 0x01


def _build_frame(frame_type: int, x_cm: int, y_cm: int) -> bytes:
    x_bytes = int(x_cm).to_bytes(2, byteorder="little", signed=True)
    y_bytes = int(y_cm).to_bytes(2, byteorder="little", signed=True)
    return bytes([0xAA, frame_type]) + x_bytes + y_bytes + bytes([0xFF])


def build_position_frame(x_cm: int, y_cm: int) -> bytes:
    return _build_frame(FRAME_TYPE_POSITION, x_cm, y_cm)


def build_fire_frame(x_cm: int, y_cm: int) -> bytes:
    return _build_frame(FRAME_TYPE_FIRE, x_cm, y_cm)


class Serial_ground(object):
    """透明UART链路，无人机→消防车单向广播，不需要接收(消防车侧不在本仓库范围)。"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.05)

    def send_position(self, x_cm: int, y_cm: int) -> None:
        self.ser.write(build_position_frame(x_cm, y_cm))

    def send_fire(self, x_cm: int, y_cm: int) -> None:
        self.ser.write(build_fire_frame(x_cm, y_cm))

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()
            logger.info("消防车广播串口已关闭")


def start_position_heartbeat(serial_ground: Serial_ground, get_position_cm, hz: float = 1.0) -> threading.Thread:
    """启动1Hz心跳位置广播后台线程(基本要求(3))。get_position_cm是无参可调用对象，
    返回(x_cm, y_cm)。"""
    interval = 1.0 / hz

    def _loop():
        while True:
            x_cm, y_cm = get_position_cm()
            try:
                serial_ground.send_position(x_cm, y_cm)
            except serial.SerialException as e:
                logger.error(f"地面站心跳位置帧发送失败，心跳线程退出: {e}")
                break
            time.sleep(interval)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    return t
