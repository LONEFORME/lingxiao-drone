import os
import sys
import threading

sys.path.insert(0, os.path.dirname(__file__))

import pytest
import serial

from Lcode.Lground import (
    build_position_frame, build_fire_frame, FRAME_TYPE_POSITION, FRAME_TYPE_FIRE,
    start_position_heartbeat,
)


class TestBuildPositionFrame:
    def test_frame_starts_with_AA_and_ends_with_FF(self):
        frame = build_position_frame(x_cm=150, y_cm=-30)
        assert frame[0] == 0xAA
        assert frame[-1] == 0xFF

    def test_frame_type_byte_is_position(self):
        frame = build_position_frame(x_cm=150, y_cm=-30)
        assert frame[1] == FRAME_TYPE_POSITION

    def test_negative_coordinate_roundtrips_as_signed_int16(self):
        frame = build_position_frame(x_cm=-100, y_cm=200)
        x_bytes = frame[2:4]
        y_bytes = frame[4:6]
        x = int.from_bytes(x_bytes, byteorder="little", signed=True)
        y = int.from_bytes(y_bytes, byteorder="little", signed=True)
        assert x == -100
        assert y == 200


class TestBuildFireFrame:
    def test_frame_type_byte_is_fire(self):
        frame = build_fire_frame(x_cm=80, y_cm=120)
        assert frame[1] == FRAME_TYPE_FIRE

    def test_coordinates_encoded_correctly(self):
        frame = build_fire_frame(x_cm=80, y_cm=120)
        x = int.from_bytes(frame[2:4], byteorder="little", signed=True)
        y = int.from_bytes(frame[4:6], byteorder="little", signed=True)
        assert x == 80
        assert y == 120


class _FakeSerialGroundRaises:
    """伪造Serial_ground，send_position()固定抛异常，用于验证心跳线程的
    异常处理(不像Lprotocol.py发送线程原来那样裸调用ser.write())。"""
    def __init__(self, exc):
        self._exc = exc
        self.calls = 0

    def send_position(self, x_cm, y_cm):
        self.calls += 1
        raise self._exc


class TestPositionHeartbeatExceptionHandling:
    """2026-07-17新增：start_position_heartbeat()的心跳线程原本裸调用
    serial_ground.send_position()，没有任何try/except——串口一抖动就静默
    崩溃退出，1Hz地面站位置广播(基本要求(3))会无声停止。跟Lprotocol.py里
    _send_t265_loop/_send_command_loop统一的"try/except SerialException+
    log+break"模式对齐。"""

    def test_serial_exception_logs_and_stops_thread(self, monkeypatch):
        logged = []
        monkeypatch.setattr("Lcode.Lground.logger.error", lambda msg, *a, **k: logged.append(msg))

        fake = _FakeSerialGroundRaises(serial.SerialException("端口已断开"))
        t = start_position_heartbeat(fake, get_position_cm=lambda: (0, 0), hz=1000.0)
        t.join(timeout=2.0)

        assert not t.is_alive()  # 线程应该已经退出，不是卡死/继续裸抛异常
        assert fake.calls == 1
        assert any("心跳" in m and "失败" in m for m in logged)

    def test_non_serial_exception_still_propagates_not_silently_swallowed(self, monkeypatch):
        """只捕获serial.SerialException，其他类型异常(比如get_position_cm本身
        有bug)不应该被这个try/except意外吞掉——那是另一类需要暴露出来的bug。"""
        def _broken_get_position():
            raise ValueError("上游坐标计算炸了")

        fake = _FakeSerialGroundRaises(serial.SerialException("不会用到"))

        # threading.Thread内部异常不会传播到主线程，只能通过excepthook间接观察；
        # 这里直接验证_loop本体在非SerialException下确实不会被我们新加的except吞掉，
        # 用threading.excepthook捕获未处理异常来断言。
        caught = []
        orig_hook = threading.excepthook
        threading.excepthook = lambda args: caught.append(args.exc_type)
        try:
            t = start_position_heartbeat(fake, get_position_cm=_broken_get_position, hz=1000.0)
            t.join(timeout=2.0)
        finally:
            threading.excepthook = orig_hook

        assert not t.is_alive()
        assert caught == [ValueError]
