import io
import json
import time

from Lcode.ground_link import (
    BroadcastGroundLink,
    GroundLinkConfig,
    GroundMessageType,
    decode_frame,
    encode_frame,
)
from Lcode.state_debug_logger import StateDebugConfig, StateTrace


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.frames = []
        self.closed = False

    def write(self, frame):
        self.frames.append(bytes(frame))
        return len(frame)

    def close(self):
        self.closed = True


def test_ground_default_port_and_training_mode():
    config = GroundLinkConfig.from_env({})
    assert config.port == "/dev/bt_serial"
    assert config.mode == "broadcast"


def test_frame_crc_roundtrip_and_corruption_detection():
    frame = encode_frame(GroundMessageType.STATE, 7, b'{"state":"TAKEOFF"}')
    message_type, sequence, payload = decode_frame(frame)
    assert message_type == GroundMessageType.STATE
    assert sequence == 7
    assert json.loads(payload) == {"state": "TAKEOFF"}

    broken = bytearray(frame)
    broken[8] ^= 0x01
    try:
        decode_frame(broken)
    except ValueError as exc:
        assert "CRC" in str(exc)
    else:
        raise AssertionError("损坏帧必须被CRC拒绝")


def test_broadcast_writes_without_receiver_or_ack():
    fake = FakeSerial()
    link = BroadcastGroundLink(
        GroundLinkConfig(queue_size=4), serial_factory=lambda **kwargs: fake
    )
    assert link.start() is True
    sequence = link.publish(GroundMessageType.STATE, {"state": "TRANSIT"})
    assert sequence == 0
    deadline = time.time() + 1.0
    while not fake.frames and time.time() < deadline:
        time.sleep(0.01)
    link.close()
    assert len(fake.frames) == 1
    assert decode_frame(fake.frames[0])[1] == 0
    assert fake.closed is True


def test_missing_broadcast_port_fails_soft():
    def fail(**kwargs):
        raise OSError("missing")

    link = BroadcastGroundLink(GroundLinkConfig(), serial_factory=fail)
    assert link.start() is False
    assert link.publish(GroundMessageType.STATE, {}) is None
    assert link.error_count == 1


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


def _records(stream):
    return [json.loads(line) for line in stream.getvalue().splitlines()]


def test_state_trace_keeps_transitions_when_debug_samples_off():
    stream = io.StringIO()
    clock = FakeClock()
    trace = StateTrace(
        stream=stream,
        config=StateDebugConfig(debug_enabled=False),
        clock=clock,
        wall_clock=clock,
    )
    trace.start("TAKEOFF", target=0)
    assert trace.sample(x=1.0) is False
    clock.value = 1.25
    trace.transition("TRANSIT", "takeoff_complete", target=1)
    trace.fault("camera_lost")
    records = _records(stream)
    assert [record["event"] for record in records] == [
        "state_enter",
        "state_exit",
        "state_enter",
        "fault",
    ]
    assert records[1]["duration_s"] == 1.25


def test_state_trace_debug_samples_are_throttled_per_state():
    stream = io.StringIO()
    clock = FakeClock()
    trace = StateTrace(
        stream=stream,
        config=StateDebugConfig(debug_enabled=True, sample_interval_s=0.1),
        clock=clock,
        wall_clock=clock,
    )
    trace.start("VISUAL_ALIGN")
    assert trace.sample(dx_px=20) is True
    clock.value = 0.05
    assert trace.sample(dx_px=10) is False
    clock.value = 0.11
    assert trace.sample(dx_px=5) is True
    samples = [r for r in _records(stream) if r["event"] == "state_sample"]
    assert [sample["dx_px"] for sample in samples] == [20, 5]
