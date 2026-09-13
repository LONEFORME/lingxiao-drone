import threading
import time
import unittest

from drone_control.competition_2026_d.Lcode.air_ground_link import AirGroundLink, LinkConfig
from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    Frame,
    MessageType,
    decode_frame,
    encode_frame,
    pack_payload,
)


class FakeSerial:
    def __init__(self, write_failures=0):
        self.rx = bytearray()
        self.writes = []
        self.lock = threading.Lock()
        self.closed = False
        self.write_failures = int(write_failures)

    def feed(self, data):
        with self.lock:
            self.rx.extend(data)

    def read(self, size):
        with self.lock:
            if not self.rx:
                data = b""
            else:
                data = bytes(self.rx[:size])
                del self.rx[:size]
                return data
        time.sleep(0.002)
        return data

    def write(self, data):
        with self.lock:
            if self.write_failures > 0:
                self.write_failures -= 1
                raise TimeoutError("simulated write timeout")
            self.writes.append(bytes(data))
        return len(data)

    def close(self):
        self.closed = True


class AirGroundLinkTest(unittest.TestCase):
    def test_ack_and_duplicate_event_are_non_reentrant(self):
        serial = FakeSerial()
        link = AirGroundLink(
            LinkConfig(read_timeout_s=0.002, ack_timeout_s=0.02),
            serial_factory=lambda **_kwargs: serial,
        )
        received = []
        link.add_callback(received.append)
        self.assertTrue(link.start())
        incoming = Frame(
            MessageType.UAV_EVENT,
            int(Flag.ACK_REQUIRED | Flag.EVENT),
            Device.CAR,
            Device.UAV,
            99,
            7,
            100,
            pack_payload(MessageType.UAV_EVENT, (3, 1234)),
        )
        raw = encode_frame(incoming)
        serial.feed(raw + raw)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and (not received or len(serial.writes) < 2):
            time.sleep(0.005)
        link.close()
        self.assertEqual(len(received), 1)
        self.assertEqual(link.stats.duplicate_events, 1)
        self.assertGreaterEqual(len(serial.writes), 2)  # 重复事件仍分别ACK
        ack = decode_frame(serial.writes[0])
        self.assertEqual(ack.message_type, MessageType.ACK)
        self.assertEqual(ack.dest, Device.CAR)
        self.assertGreaterEqual(link.stats.tx_ack_frames, 2)
        self.assertEqual(link.stats.last_ack_tx_hex, serial.writes[-1].hex())

    def test_car_start_ack_is_deferred_to_business_layer(self):
        serial = FakeSerial()
        link = AirGroundLink(
            LinkConfig(read_timeout_s=0.002, ack_timeout_s=0.02),
            serial_factory=lambda **_kwargs: serial,
        )
        received = []
        link.add_callback(received.append)
        self.assertTrue(link.start())
        incoming = Frame(
            MessageType.CAR_START,
            int(Flag.ACK_REQUIRED),
            Device.CAR,
            Device.UAV,
            99,
            7,
            100,
            pack_payload(MessageType.CAR_START, (1, 1234)),
        )
        raw = encode_frame(incoming)
        serial.feed(raw + raw)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and len(received) < 2:
            time.sleep(0.005)
        self.assertEqual(len(serial.writes), 0)
        self.assertEqual(len(received), 2)
        ack_frame_seq = link.acknowledge(incoming, result=1)
        self.assertIsNotNone(ack_frame_seq)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not serial.writes:
            time.sleep(0.005)
        link.close()
        ack = decode_frame(serial.writes[0])
        self.assertEqual(
            ack.payload,
            pack_payload(MessageType.ACK, (MessageType.CAR_START, 7, 1)),
        )
        self.assertEqual(link.stats.tx_ack_frames, 1)
        self.assertEqual(link.stats.last_ack_tx_hex, serial.writes[0].hex())

    def test_reliable_uav_event_accepts_matching_positive_ack(self):
        serial = FakeSerial()
        link = AirGroundLink(
            LinkConfig(read_timeout_s=0.002, ack_timeout_s=0.05),
            serial_factory=lambda **_kwargs: serial,
        )
        self.assertTrue(link.start())
        event_seq = link.publish(
            MessageType.UAV_EVENT,
            pack_payload(MessageType.UAV_EVENT, (3, 1234)),
            session_id=99,
            dest=Device.CAR,
            flags=Flag.ACK_REQUIRED | Flag.EVENT,
        )
        self.assertIsNotNone(event_seq)
        ack = Frame(
            MessageType.ACK,
            int(Flag.IS_ACK),
            Device.CAR,
            Device.UAV,
            99,
            100,
            200,
            pack_payload(
                MessageType.ACK,
                (MessageType.UAV_EVENT, event_seq, 0),
            ),
        )
        serial.feed(encode_frame(ack))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and link.stats.rx_ack_success < 1:
            time.sleep(0.005)
        self.assertTrue(link.wait_pending(0.1))
        serial.feed(encode_frame(ack))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and link.stats.rx_ack_duplicate < 1:
            time.sleep(0.005)
        link.close()
        self.assertEqual(link.stats.rx_ack_success, 1)
        self.assertEqual(link.stats.rx_ack_negative, 0)
        self.assertEqual(link.stats.rx_ack_duplicate, 1)
        self.assertEqual(link.stats.rx_ack_unmatched, 0)

    def test_negative_ack_is_visible_and_not_counted_as_success(self):
        serial = FakeSerial()
        link = AirGroundLink(
            LinkConfig(read_timeout_s=0.002, ack_timeout_s=0.05),
            serial_factory=lambda **_kwargs: serial,
        )
        self.assertTrue(link.start())
        event_seq = link.publish(
            MessageType.UAV_EVENT,
            pack_payload(MessageType.UAV_EVENT, (3, 1234)),
            session_id=99,
            dest=Device.CAR,
            flags=Flag.ACK_REQUIRED | Flag.EVENT,
        )
        ack = Frame(
            MessageType.ACK,
            int(Flag.IS_ACK),
            Device.CAR,
            Device.UAV,
            99,
            101,
            201,
            pack_payload(
                MessageType.ACK,
                (MessageType.UAV_EVENT, event_seq, 1),
            ),
        )
        serial.feed(encode_frame(ack))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and link.stats.rx_ack_negative < 1:
            time.sleep(0.005)
        self.assertTrue(link.wait_pending(0.1))
        link.close()
        self.assertEqual(link.stats.rx_ack_success, 0)
        self.assertEqual(link.stats.rx_ack_negative, 1)

    def test_reliable_event_logs_timeout_after_fixed_frame_retries(self):
        serial = FakeSerial()
        link = AirGroundLink(
            LinkConfig(
                read_timeout_s=0.002,
                ack_timeout_s=0.01,
                max_retries=1,
            ),
            serial_factory=lambda **_kwargs: serial,
        )
        self.assertTrue(link.start())
        link.publish(
            MessageType.UAV_EVENT,
            pack_payload(MessageType.UAV_EVENT, (3, 1234)),
            session_id=99,
            dest=Device.CAR,
            flags=Flag.ACK_REQUIRED | Flag.EVENT,
        )
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and link.stats.ack_timeouts < 1:
            time.sleep(0.005)
        event = decode_frame(serial.writes[0])
        late_ack = Frame(
            MessageType.ACK,
            int(Flag.IS_ACK),
            Device.CAR,
            Device.UAV,
            event.session_id,
            200,
            300,
            pack_payload(
                MessageType.ACK,
                (MessageType.UAV_EVENT, event.seq, 0),
            ),
        )
        serial.feed(encode_frame(late_ack))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and link.stats.rx_ack_late < 1:
            time.sleep(0.005)
        serial.feed(encode_frame(late_ack))
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and link.stats.rx_ack_duplicate < 1:
            time.sleep(0.005)
        link.close()
        self.assertEqual(link.stats.ack_timeouts, 1)
        self.assertEqual(len(serial.writes), 2)
        self.assertEqual(serial.writes[0], serial.writes[1])
        self.assertEqual(link.stats.rx_ack_late, 1)
        self.assertEqual(link.stats.rx_ack_duplicate, 1)
        self.assertEqual(link.stats.rx_ack_unmatched, 0)

    def test_single_write_timeout_drops_frame_but_keeps_link_running(self):
        serial = FakeSerial(write_failures=1)
        link = AirGroundLink(
            LinkConfig(
                read_timeout_s=0.002,
                write_timeout_s=0.01,
                max_consecutive_tx_errors=3,
            ),
            serial_factory=lambda **_kwargs: serial,
        )
        self.assertTrue(link.start())
        link.publish(
            MessageType.UAV_STATE,
            pack_payload(MessageType.UAV_STATE, (0, 0, 0)),
            session_id=99,
            dest=Device.CAR,
        )
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and link.stats.io_errors < 1:
            time.sleep(0.005)
        self.assertTrue(link.is_running)
        link.publish(
            MessageType.UAV_STATE,
            pack_payload(MessageType.UAV_STATE, (1, 2, 3)),
            session_id=99,
            dest=Device.CAR,
        )
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not serial.writes:
            time.sleep(0.005)
        self.assertTrue(link.is_running)
        link.close()
        self.assertEqual(link.stats.io_errors, 1)
        self.assertEqual(link.stats.tx_dropped, 1)
        self.assertEqual(link.stats.current_tx_error_streak, 0)
        self.assertEqual(link.stats.max_tx_error_streak, 1)
        self.assertEqual(len(serial.writes), 1)


if __name__ == "__main__":
    unittest.main()
