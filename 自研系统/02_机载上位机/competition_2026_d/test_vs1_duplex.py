from __future__ import annotations

import tempfile
import threading
import time
import unittest
from pathlib import Path

from .vision.cybercam_protocol import (
    encode_control,
    parse_control,
)
from .vision.cybercam_reader import CyberCamReader
from .vision.link_status import (
    LinkStatusPublisher,
    OledLinkEvaluator,
    read_link_status,
)


def encode_vs1(stream_id, seq, capture_ms, found, cx, cy, outer, inner, angle, quality, flags):
    import binascii
    values = (
        "VS1", str(stream_id), str(seq), str(capture_ms), "1" if found else "0",
        str(cx), str(cy), str(outer), str(inner), str(angle), str(quality), str(flags),
    )
    body = ",".join(values).encode("ascii")
    crc = binascii.crc_hqx(body, 0xFFFF)
    return body + b"," + f"{crc:04X}".encode("ascii") + b"\n"


class FakeStatusPublisher:
    def __init__(self):
        self.records = []

    def publish(self, values, now=None, force=False):
        self.records.append((dict(values), now, force))
        return True


class FakeSerial:
    def __init__(self, fail_writes=0):
        self.rx = bytearray()
        self.writes = []
        self.fail_writes = fail_writes
        self.closed = False
        self.lock = threading.Lock()

    def read(self, size):
        with self.lock:
            data = bytes(self.rx[:size])
            del self.rx[:size]
        if not data:
            time.sleep(0.002)
        return data

    def write(self, data):
        if self.fail_writes:
            self.fail_writes -= 1
            raise TimeoutError("simulated write timeout")
        self.writes.append(bytes(data))
        return len(data)

    def feed(self, data):
        with self.lock:
            self.rx.extend(data)

    def close(self):
        self.closed = True


class RdkDuplexProtocolTest(unittest.TestCase):
    def test_control_protocol_roundtrip_and_crc(self):
        for command in ("PING", "PONG"):
            packet = encode_control(command, 0xFFFFFFFF)
            self.assertEqual(parse_control(packet), (command, 0xFFFFFFFF))
        damaged = bytearray(encode_control("PING", 7))
        damaged[-3] ^= 1
        with self.assertRaises(ValueError):
            parse_control(damaged)

    def test_rdk_ping_write_timeout_does_not_stop_vs1_receive(self):
        serial = FakeSerial(fail_writes=1)
        status = FakeStatusPublisher()
        reader = CyberCamReader(
            serial_factory=lambda *_args, **_kwargs: serial,
            status_publisher=status,
        )
        self.assertTrue(reader.start())
        serial.feed(encode_vs1(1, 1, 10, True, 320, 240, 80, 0, 0, 90, 8))
        deadline = time.monotonic() + 0.5
        while reader.stats()["accepted_frames"] < 1 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(reader.stats()["accepted_frames"], 1)
        self.assertEqual(reader.stats()["ping_write_errors"], 1)
        self.assertTrue(reader.is_running())
        reader.close()

    def test_recent_out_of_order_pong_is_accepted_once(self):
        reader = CyberCamReader(status_publisher=FakeStatusPublisher())
        reader._pending_pings.extend(((10, 1.0), (11, 2.0), (12, 3.0)))
        reader._handle_line(encode_control("PONG", 10).strip(), 3.5)
        self.assertEqual(reader.pongs_received, 1)
        reader._handle_line(encode_control("PONG", 10).strip(), 3.6)
        self.assertEqual(reader.pongs_received, 1)
        self.assertEqual(reader.control_errors, 1)


class LinkStatusTest(unittest.TestCase):
    def test_snapshot_roundtrip_and_oled_debounce(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "status.json"
            publisher = LinkStatusPublisher(path, interval_s=0.0)
            publisher.started_monotonic = 1.0
            self.assertTrue(publisher.publish({
                "running": True,
                "last_vs1_monotonic": 10.0,
                "last_pong_monotonic": 10.0,
            }, now=10.0))
            snapshot = read_link_status(path)
            evaluator = OledLinkEvaluator()
            self.assertEqual(evaluator.evaluate(snapshot, now=10.1), {
                "cam_to_rdk": "OK", "rdk_to_cam": "OK",
            })
            self.assertEqual(evaluator.evaluate(snapshot, now=14.0), {
                "cam_to_rdk": "OK", "rdk_to_cam": "OK",
            })
            self.assertEqual(evaluator.evaluate(snapshot, now=15.0), {
                "cam_to_rdk": "LOST", "rdk_to_cam": "LOST",
            })

    def test_invalid_snapshot_fails_closed_and_pid_change_shows_restart(self):
        evaluator = OledLinkEvaluator()
        first = {
            "version": 1, "pid": 1, "started_monotonic": 1.0,
            "updated_monotonic": 5.0, "running": True,
            "last_vs1_monotonic": 5.0, "last_pong_monotonic": 5.0,
        }
        self.assertEqual(evaluator.evaluate(first, 5.0)["cam_to_rdk"], "OK")
        restarted = dict(first, pid=2, started_monotonic=4.0)
        self.assertEqual(evaluator.evaluate(restarted, 5.1), {
            "cam_to_rdk": "RESTART", "rdk_to_cam": "RESTART",
        })
        self.assertEqual(evaluator.evaluate(None, 5.2), {
            "cam_to_rdk": "LOST", "rdk_to_cam": "LOST",
        })


if __name__ == "__main__":
    unittest.main()
