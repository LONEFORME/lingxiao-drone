from __future__ import annotations

import unittest

try:
    from .main import DuplexControlResponder
    from .protocol import encode_control, parse_control
except ImportError:
    from main import DuplexControlResponder
    from protocol import encode_control, parse_control


class FakeSerial:
    def __init__(self, fail_writes=0):
        self.rx = bytearray()
        self.writes = []
        self.fail_writes = fail_writes

    def read(self, size):
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, data):
        if self.fail_writes:
            self.fail_writes -= 1
            raise TimeoutError("simulated PONG timeout")
        self.writes.append(bytes(data))
        return len(data)

    def feed(self, data):
        self.rx.extend(data)


class CyberDuplexProtocolTest(unittest.TestCase):
    def test_control_roundtrip_and_crc(self):
        for command in ("PING", "PONG"):
            self.assertEqual(parse_control(encode_control(command, 9)), (command, 9))
        damaged = bytearray(encode_control("PING", 7))
        damaged[-3] ^= 1
        with self.assertRaises(ValueError):
            parse_control(damaged)

    def test_at_most_one_control_line_per_image_loop(self):
        serial = FakeSerial()
        serial.feed(b"".join(encode_control("PING", seq) for seq in range(3)))
        responder = DuplexControlResponder()
        self.assertTrue(responder.service(serial))
        self.assertEqual(len(serial.writes), 1)
        self.assertEqual(parse_control(serial.writes[0]), ("PONG", 0))
        self.assertTrue(responder.service(serial))
        self.assertEqual(len(serial.writes), 2)
        self.assertEqual(parse_control(serial.writes[1]), ("PONG", 1))

    def test_fragmented_ping_is_buffered_across_image_loops(self):
        serial = FakeSerial()
        responder = DuplexControlResponder()
        packet = encode_control("PING", 23)
        split = len(packet) // 2
        serial.feed(packet[:split])
        self.assertFalse(responder.service(serial))
        self.assertEqual(serial.writes, [])
        serial.feed(packet[split:])
        self.assertTrue(responder.service(serial))
        self.assertEqual(parse_control(serial.writes[0]), ("PONG", 23))

    def test_pong_write_failure_does_not_break_next_frame(self):
        serial = FakeSerial(fail_writes=1)
        responder = DuplexControlResponder()
        serial.feed(encode_control("PING", 4))
        self.assertFalse(responder.service(serial))
        self.assertEqual(responder.write_errors, 1)
        serial.feed(encode_control("PING", 5))
        self.assertTrue(responder.service(serial))
        self.assertEqual(parse_control(serial.writes[0]), ("PONG", 5))


if __name__ == "__main__":
    unittest.main()
