import time

from Lcode.Lprotocol import Serial_fc
from Lcode.global_variable import fc_frame_counter, fc_last_rx_monotonic, lock


class FakeSerial:
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


def _frame1_with_yaw(yaw_x100):
    encoded = int(yaw_x100) & 0xFFFF
    data = bytearray(24)
    data[5] = encoded & 0xFF
    data[6] = (encoded >> 8) & 0xFF
    header = bytes([0x01, 24])
    checksum = (sum(header) + sum(data)) & 0xFF
    return b"\xAA" + header + bytes(data) + bytes([checksum, 0xFF])


def test_frame1_updates_yaw_monotonic_timestamp_and_counter_together():
    fc = Serial_fc.__new__(Serial_fc)
    fc.debug_data = {}
    fc.fclisten_running = True
    fc._last_laser_height_cm = 0.0
    fc.ser = FakeSerial(_frame1_with_yaw(-1234), fc)
    rxbuffer = [0] * 14
    with lock:
        counter_before = int(fc_frame_counter.value)
    before = time.monotonic()

    fc.listen_fc(rxbuffer)

    with lock:
        assert rxbuffer[3] == -1234
        assert fc_frame_counter.value == counter_before + 1
        assert fc_last_rx_monotonic.value >= before
