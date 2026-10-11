import json

import numpy as np

from Lcode.laser_pointer import LaserConfig, LaserPointer
from Lcode.qr_vision import (
    QRConsensus,
    QRConsensusConfig,
    QRDetection,
    VisionDebugCapture,
    VisionDebugConfig,
    point_inside_qr,
)
from Lcode.sensor_gimbal import GimbalConfig, SensorGimbal
from Lcode.warehouse_model import FaceId


class FakePWM:
    def __init__(self):
        self.started = False
        self.stopped = False
        self.duties = []

    def start(self):
        self.started = True

    def set_duty_ns(self, value):
        self.duties.append(value)

    def stop(self):
        self.stopped = True


def test_gimbal_face_mapping_is_explicit_not_count_based():
    pwm = FakePWM()
    sleeps = []
    gimbal = SensorGimbal(
        GimbalConfig(settle_s=0.2), pwm=pwm, sleep_fn=sleeps.append
    )
    assert gimbal.start() is True
    assert gimbal.set_face(FaceId.A) is True
    assert gimbal.set_face(FaceId.B) is True
    assert gimbal.set_face(FaceId.C) is True
    assert pwm.duties == [500_000, 2_500_000, 500_000]
    assert sleeps == [0.2, 0.2, 0.2]
    gimbal.close()
    assert pwm.stopped is True


class FakeGPIO:
    BCM = 11
    OUT = 1
    LOW = 0
    HIGH = 1

    def __init__(self):
        self.outputs = []
        self.cleaned = False

    def setmode(self, mode):
        self.mode = mode

    def setup(self, pin, mode):
        self.setup_args = (pin, mode)

    def output(self, pin, value):
        self.outputs.append((pin, value))

    def cleanup(self):
        self.cleaned = True


def test_laser_starts_and_ends_low_with_half_second_default():
    gpio = FakeGPIO()
    config = LaserConfig(duration_s=0.5, pwm_period_s=0.005)
    laser = LaserPointer(config, gpio=gpio)
    assert laser.start() is True
    assert gpio.outputs[-1] == (19, gpio.LOW)
    assert laser.pulse_async(duration_s=0.1) is True
    assert laser.wait(timeout=0.5) is True
    assert gpio.outputs[-1] == (19, gpio.LOW)
    assert any(value == gpio.HIGH for _, value in gpio.outputs)
    laser.close()
    assert gpio.outputs[-1] == (19, gpio.LOW)
    assert gpio.cleaned is True


def _detection(number=7):
    return QRDetection(
        number=number,
        content=f"qr-{number}",
        corners=((100, 100), (200, 100), (200, 200), (100, 200)),
    )


def test_qr_consensus_requires_multiple_frames_and_safe_laser_point():
    consensus = QRConsensus(QRConsensusConfig(window_size=5, required_count=3, laser_margin_px=10, require_laser_inside=True))
    assert point_inside_qr(_detection().corners, (150, 150), 10) is True
    assert point_inside_qr(_detection().corners, (105, 150), 10) is False
    assert consensus.update(_detection(), (150, 150)) is None
    assert consensus.update(_detection(), (150, 150)) is None
    accepted = consensus.update(_detection(), (150, 150))
    assert accepted.number == 7

    consensus.reset()
    for _ in range(5):
        assert consensus.update(_detection(), (105, 150)) is None


def test_qr_consensus_can_accept_one_frame_without_laser_gate():
    consensus = QRConsensus(
        QRConsensusConfig(
            window_size=1,
            required_count=1,
            require_laser_inside=False,
        )
    )
    accepted = consensus.update(_detection(number=11), (0, 0))
    assert accepted.number == 11


def test_vision_debug_capture_switch_and_transit_throttle(tmp_path):
    writes = []

    def writer(path, frame):
        writes.append(path)
        open(path, "wb").write(b"jpg")
        return True

    frame = np.zeros((10, 10, 3), dtype=np.uint8)
    disabled = VisionDebugCapture(
        tmp_path / "off",
        VisionDebugConfig(enabled=False),
        image_writer=writer,
    )
    assert disabled.capture_fixed(frame, {}) is None
    assert writes == []

    enabled = VisionDebugCapture(
        tmp_path / "on",
        VisionDebugConfig(enabled=True, transit_interval_s=1.0, transit_interval_m=0.5),
        image_writer=writer,
    )
    first = enabled.maybe_capture_transit(frame, (0, 0), {"state": "TRANSIT"}, now=0)
    assert first is not None
    assert enabled.maybe_capture_transit(frame, (0.1, 0), {}, now=0.2) is None
    second = enabled.maybe_capture_transit(frame, (0.6, 0), {"state": "TRANSIT"}, now=0.3)
    assert second is not None
    metadata = json.loads(second.with_suffix(".json").read_text(encoding="utf-8"))
    assert metadata["state"] == "TRANSIT"


def test_vision_debug_capture_scan_is_throttled_and_marked_scan(tmp_path):
    writes = []

    def writer(path, frame):
        writes.append(path)
        open(path, "wb").write(b"jpg")
        return True

    capture = VisionDebugCapture(
        tmp_path,
        VisionDebugConfig(enabled=True, scan_interval_s=0.5),
        image_writer=writer,
    )
    frame = np.zeros((10, 10, 3), dtype=np.uint8)
    first = capture.capture_scan(frame, {"state": "VERIFY_QR"}, now=0.0)
    assert first is not None
    assert capture.capture_scan(frame, {"state": "TAKEOFF"}, now=0.2) is None
    second = capture.capture_scan(frame, {"state": "VERIFY_QR"}, now=0.6)
    assert second is not None
    assert all("_scan.jpg" in path for path in writes)
    assert json.loads(second.with_suffix(".json").read_text(encoding="utf-8"))["state"] == "VERIFY_QR"
