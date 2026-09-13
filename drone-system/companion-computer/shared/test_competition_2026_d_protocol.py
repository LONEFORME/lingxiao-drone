import unittest
import struct

from shared.competition_2026_d_protocol import (
    CarStateFlag,
    CarStatePayload,
    Device,
    Flag,
    Frame,
    MessageType,
    StreamParser,
    UavPhase,
    decode_frame,
    encode_frame,
    pack_car_state,
    pack_payload,
    seq_is_newer,
    unpack_car_state,
    unpack_payload,
)


class DcpProtocolTest(unittest.TestCase):
    def test_golden_car_start(self):
        frame = Frame(
            message_type=MessageType.CAR_START,
            flags=Flag.ACK_REQUIRED | Flag.EVENT,
            source=Device.CAR,
            dest=Device.UAV,
            session_id=0x12345678,
            seq=0x002A,
            sender_ms=1000,
            payload=pack_payload(MessageType.CAR_START, (2, 0x89ABCDEF)),
        )
        raw = encode_frame(frame)
        self.assertEqual(
            raw.hex(),
            "aa0103050201785634122a00e8030000050002efcdab8982f9ff",
        )
        decoded = decode_frame(raw)
        self.assertEqual(decoded, frame)
        self.assertEqual(unpack_payload(decoded.message_type, decoded.payload), (2, 0x89ABCDEF))

    def test_stream_recovers_after_corrupt_frame(self):
        good = encode_frame(Frame(MessageType.HEARTBEAT, 0, Device.UAV, Device.CAR, 7, 9, 20, b"\x02\x00\x00"))
        bad = bytearray(good)
        bad[-3] ^= 0x40
        parser = StreamParser()
        frames = parser.feed(b"noise" + bytes(bad) + good[:8])
        self.assertEqual(frames, [])
        frames = parser.feed(good[8:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 9)
        self.assertGreaterEqual(parser.rejected, 1)

    def test_official_extended_car_state_golden_frame(self):
        raw = bytes.fromhex(
            "AA 01 10 00 02 01 78 56 34 12 2A 00 E8 03 00 00 "
            "0D 00 01 DC 05 82 00 00 00 11 00 82 00 00 00 B9 "
            "42 FF"
        )
        frame = decode_frame(raw)
        self.assertEqual(frame.message_type, MessageType.CAR_STATE)
        self.assertEqual(frame.source, Device.CAR)
        self.assertEqual(frame.dest, Device.UAV)
        state = unpack_car_state(frame.payload)
        self.assertEqual(
            state,
            CarStatePayload(
                segment=1,
                track_s_mm=1500,
                speed_mm_s=130,
                heading_cdeg=0,
                state_flags=0x0011,
                vx_mm_s=130,
                vy_mm_s=0,
            ),
        )
        self.assertEqual(encode_frame(frame), raw)
        self.assertEqual(raw[-3:-1], bytes.fromhex("B9 42"))

    def test_car_state_extended_round_trip_signed_values(self):
        values = (
            2,
            65535,
            -130,
            -18000,
            int(CarStateFlag.ENCODER_SPEED_VALID),
            -120,
            50,
        )
        payload = pack_car_state(values)
        self.assertEqual(len(payload), 13)
        self.assertEqual(unpack_payload(MessageType.CAR_STATE, payload), values)

    def test_legacy_car_state_is_explicitly_compatible(self):
        payload = struct.pack("<BHhhH", 1, 100, 130, 0, 0x0011)
        state = unpack_car_state(payload)
        self.assertEqual(state.speed_mm_s, 130)
        self.assertFalse(state.has_world_velocity)
        self.assertIsNone(state.vx_mm_s)
        with self.assertRaises(ValueError):
            unpack_payload(MessageType.CAR_STATE, payload)

    def test_new_car_state_packer_rejects_missing_velocity(self):
        with self.assertRaises(ValueError):
            pack_payload(MessageType.CAR_STATE, (1, 100, 130, 0, 0x0011))

    def test_sequence_wrap(self):
        self.assertTrue(seq_is_newer(0, 65535))
        self.assertFalse(seq_is_newer(65535, 0))
        self.assertFalse(seq_is_newer(42, 42))

    def test_uav_state_payload_schema(self):
        values = (100, -200, 1500)
        payload = pack_payload(MessageType.UAV_STATE, values)
        self.assertEqual(unpack_payload(MessageType.UAV_STATE, payload), values)

    def test_uav_event_payload_schema(self):
        values = (UavPhase.FORMATION_FOLLOW, 1234)
        payload = pack_payload(MessageType.UAV_EVENT, values)
        self.assertEqual(unpack_payload(MessageType.UAV_EVENT, payload), values)

    def test_coordinate_fusion_payload_schemas(self):
        car_position = (1500, 2000, 25, 0x000D)
        payload = pack_payload(MessageType.CAR_POSITION, car_position)
        self.assertEqual(len(payload), 12)
        self.assertEqual(
            unpack_payload(MessageType.CAR_POSITION, payload),
            car_position,
        )

        fused = (1500, 2000, 750, 750, 1500, 42, 1234, 25, 0x000F)
        payload = pack_payload(MessageType.FUSED_POSITION, fused)
        self.assertEqual(len(payload), 28)
        self.assertEqual(
            unpack_payload(MessageType.FUSED_POSITION, payload),
            fused,
        )


if __name__ == "__main__":
    unittest.main()
