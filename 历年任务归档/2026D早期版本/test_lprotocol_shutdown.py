import threading

from Lcode import Lprotocol


class BlockingSerial:
    def __init__(self, *_, **__):
        self.is_open = True
        self.read_started = threading.Event()
        self.read_cancelled = threading.Event()
        self.closed_while_reading = False
        self.closed = False

    def reset_input_buffer(self):
        pass

    def read(self, _size=1):
        self.read_started.set()
        self.read_cancelled.wait(timeout=1.0)
        if self.closed:
            self.closed_while_reading = True
            raise TypeError("fd is already closed")
        return b""

    def cancel_read(self):
        self.read_cancelled.set()

    def close(self):
        self.closed = True
        self.is_open = False


def test_close_stops_blocking_listener_before_closing_serial(monkeypatch):
    fake = BlockingSerial()
    monkeypatch.setattr(Lprotocol.serial, "Serial", lambda **_: fake)
    fc = Lprotocol.Serial_fc("fake", 460800)

    fc.listen_start([0] * 14)
    assert fake.read_started.wait(timeout=0.2)

    fc.close()

    assert fake.read_cancelled.is_set()
    assert fc._listen_thread.is_alive() is False
    assert fake.closed is True
    assert fake.closed_while_reading is False


def test_close_is_idempotent(monkeypatch):
    fake = BlockingSerial()
    monkeypatch.setattr(Lprotocol.serial, "Serial", lambda **_: fake)
    fc = Lprotocol.Serial_fc("fake", 460800)

    fc.close()
    fc.close()

    assert fake.closed is True
