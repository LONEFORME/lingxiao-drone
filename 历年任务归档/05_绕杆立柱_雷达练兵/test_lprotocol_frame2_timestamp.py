"""帧2(motor_pwm_mask所在的调试扩展帧)到达时间戳的单元测试。

背景：2026-07-10真机测试出现`land()`双条件确认(unlock_sta==0且motor_pwm_mask==0)
按预期触发、但用户现场确认电机实际没有停转的严重案例。怀疑之一是`motor_pwm_mask`
走的帧2被降频到约2.5秒才更新一次，`debug_data`里存的可能是一份滞后的旧快照，
但此前没有记录帧2包本身的到达时间，没法验证"读到的0到底是不是新鲜数据"。这里给
`debug_data`加一个到达时间戳字段，后续真机测试时可以跟`unlock_sta`翻转时刻对比，
量化`motor_pwm_mask`的真实更新延迟。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/circle_pole && python -m pytest test_lprotocol_frame2_timestamp.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.Lprotocol import Serial_fc


class FakeSerial:
    """伪造串口对象，按 listen_fc() 的读取顺序(单字节->2字节头->length+2字节体)
    逐段吐出预先构造好的帧字节。缓冲区吐完后把owner的fclisten_running置False，
    让 listen_fc() 的while循环能正常退出，不用真实起线程。"""

    def __init__(self, frame_bytes, owner):
        self._buf = bytearray(frame_bytes)
        self._owner = owner

    def read(self, size=1):
        if not self._buf:
            self._owner.fclisten_running = False
            return b""
        chunk = bytes(self._buf[:size])
        del self._buf[:size]
        return chunk


def _build_frame2(motor_pwm_mask):
    """构造一个合法的帧2(0x02, 19字节数据)：9个int16(全0，无关字段) + 1字节mask。"""
    data = bytearray(18) + bytes([motor_pwm_mask])
    header = bytes([0x02, 19])
    checksum = (sum(header) + sum(data)) & 0xFF
    return b"\xAA" + header + bytes(data) + bytes([checksum]) + b"\xFF"


def _make_serial_fc(frame_bytes):
    fc = Serial_fc.__new__(Serial_fc)  # 跳过__init__(会打开真实串口)
    fc.debug_data = {}
    fc.fclisten_running = True
    fc._last_laser_height_cm = 0.0
    fc.ser = FakeSerial(frame_bytes, fc)
    return fc


class TestFrame2Timestamp:
    def test_debug_data_includes_arrival_timestamp(self):
        fc = _make_serial_fc(_build_frame2(motor_pwm_mask=0x0F))

        before = time.time()
        fc.listen_fc(rxbuffer=[0] * 14)
        after = time.time()

        assert "motor_pwm_mask_t" in fc.debug_data
        assert before <= fc.debug_data["motor_pwm_mask_t"] <= after

    def test_motor_pwm_mask_value_still_parsed_correctly(self):
        fc = _make_serial_fc(_build_frame2(motor_pwm_mask=0x0F))

        fc.listen_fc(rxbuffer=[0] * 14)

        assert fc.debug_data["motor_pwm_mask"] == 0x0F


class TestLandTimeoutGaveupBit:
    def test_bit4_set_parses_as_land_timeout_gaveup_true(self):
        """motor_pwm_mask字节的bit4=1时，debug_data里land_timeout_gaveup应为True，
        且不影响原有bit0~3(motor_pwm_mask本身，诊断电机PWM用)的解析。"""
        # 0x1F = 0b00011111: bit0-3全1(m1~m4非零) + bit4=1(land_timeout_gaveup)
        fc = _make_serial_fc(_build_frame2(motor_pwm_mask=0x1F))

        fc.listen_fc(rxbuffer=[0] * 14)

        assert fc.debug_data["motor_pwm_mask"] == 0x1F
        assert fc.debug_data["land_timeout_gaveup"] is True

    def test_bit4_clear_parses_as_land_timeout_gaveup_false(self):
        """bit4=0时应为False，不是None——跟'字段不存在'(老固件/未收到帧2)要能区分开。"""
        fc = _make_serial_fc(_build_frame2(motor_pwm_mask=0x0F))  # bit0-3全1，bit4=0

        fc.listen_fc(rxbuffer=[0] * 14)

        assert fc.debug_data["land_timeout_gaveup"] is False
