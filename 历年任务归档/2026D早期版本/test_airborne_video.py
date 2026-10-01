import threading
import time

from Lcode.airborne_video import AirborneVideoConfig, AirborneVideoManager
from Lcode.video_backends import (
    decode_udp_jpeg_packet,
    encode_udp_jpeg_packets,
)
from Lcode.video_source import (
    SnapshotResult,
    VideoFrame,
    VideoPublisher,
    VideoPublisherConfig,
    VideoSource,
    VideoSourceConfig,
)


class MemorySource(VideoSource):
    def __init__(self, frames=3):
        self.frames = frames
        self.running = False
        self.sequence = 0

    def start(self):
        self.running = True
        return True

    def read_frame(self, timeout_s=0.5):
        if self.frames <= 0:
            time.sleep(min(timeout_s, 0.01))
            return None
        self.frames -= 1
        frame = VideoFrame(self.sequence, time.time(), 1, 1, "jpeg", b"jpeg")
        self.sequence += 1
        return frame

    def snapshot(self, point_id, output_dir, timeout_s=1.0):
        return SnapshotResult(point_id, None, None, "unused")

    def is_running(self):
        return self.running

    def stop(self):
        self.running = False


class MemoryPublisher(VideoPublisher):
    def __init__(self, fail=False):
        self.fail = fail
        self.running = False
        self.frames = []

    def start(self):
        self.running = True
        return True

    def publish_frame(self, frame):
        if self.fail:
            return False
        self.frames.append(frame)
        return True

    def is_running(self):
        return self.running

    def stop(self):
        self.running = False


def config(max_failures=5):
    return AirborneVideoConfig(
        enabled=True,
        source=VideoSourceConfig(enabled=True, backend="memory"),
        publisher=VideoPublisherConfig(enabled=True, backend="memory"),
        max_fps=100,
        read_timeout_s=0.01,
        max_consecutive_failures=max_failures,
    )


def wait_for(predicate, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def test_airborne_video_publishes_and_stops():
    source = MemorySource(frames=3)
    publisher = MemoryPublisher()
    manager = AirborneVideoManager(config(), source, publisher)
    assert manager.start()
    assert wait_for(lambda: manager.stats()["frames_published"] == 3)
    assert manager.stop()
    assert not source.running
    assert not publisher.running


def test_airborne_video_failure_circuit_does_not_raise():
    source = MemorySource(frames=5)
    publisher = MemoryPublisher(fail=True)
    manager = AirborneVideoManager(config(max_failures=2), source, publisher)
    assert manager.start()
    assert wait_for(lambda: manager.stats()["circuit_open"])
    manager.stop()
    assert manager.stats()["failures"] >= 2


def test_udp_jpeg_packets_are_bounded_and_describe_reassembly():
    jpeg = b"x" * 3000
    packets = encode_udp_jpeg_packets(jpeg, frame_id=42, max_datagram=500)
    assert all(len(packet) <= 500 for packet in packets)
    decoded = [decode_udp_jpeg_packet(packet) for packet in packets]
    assert {header["frame_id"] for header, _ in decoded} == {42}
    assert [header["chunk_index"] for header, _ in decoded] == list(
        range(len(packets))
    )
    assert b"".join(payload for _, payload in decoded) == jpeg
