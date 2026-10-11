import io
import threading
from types import SimpleNamespace

import Mission_GPT as mg
import numpy as np
import pytest

from Lcode import inventory_controller as controller
from Lcode.inventory_planner import MissionWaypoint, WaypointKind
from Lcode.inventory_state import InventoryState, InventoryStateMachine
from Lcode.navigation_profile import NavigationProfileConfig
from Lcode.inventory_store import InventoryStore
from Lcode.qr_vision import QRConsensus, QRConsensusConfig, QRDetection
from Lcode.state_debug_logger import StateDebugConfig, StateTrace
from Lcode.vision_servo import VisionServoConfig
from Lcode.warehouse_model import FaceId, FlightPoint


class FakeGround:
    def __init__(self):
        self.messages = []

    def publish(self, message_type, payload):
        self.messages.append((message_type, payload))
        return len(self.messages)


def test_inventory_config_reads_raw_decode_profile():
    config = controller.InventoryMissionConfig.from_env(
        {"DRONE_QR_DECODE_PROFILE": "raw"}
    )

    assert config.qr_decode_profile == "raw"


def test_inventory_config_rejects_invalid_decode_profile():
    with pytest.raises(ValueError, match="raw/variants"):
        controller.InventoryMissionConfig.from_env(
            {"DRONE_QR_DECODE_PROFILE": "invalid"}
        )


class FakeGimbal:
    def __init__(self):
        self.started = True
        self.faces = []

    def set_face(self, face):
        self.faces.append(face)
        return True


class FakeLaser:
    class Config:
        duration_s = 0.5

    def __init__(self, events, pulse_result=True):
        self.config = self.Config()
        self.events = events
        self.pulse_result = pulse_result

    def pulse_async(self):
        self.events.append("laser_pulse")
        return self.pulse_result

    def wait(self, timeout):
        self.events.append("laser_wait")
        return self.pulse_result


class FakeFrame:
    shape = (100, 100, 3)


class FakeCamera:
    def __init__(self, frames):
        self.frames = list(frames)
        self.aim = (50.0, 50.0)

    def read(self):
        return self.frames.pop(0) if self.frames else None

    def laser_aim_point(self, frame):
        return self.aim


class SequencedCamera(FakeCamera):
    def __init__(self, samples):
        super().__init__([])
        self.samples = list(samples)
        self.last_sample = (None, None, None)

    def read_with_sequence(self):
        if self.samples:
            self.last_sample = self.samples.pop(0)
        return self.last_sample


class FakeDecoder:
    def __init__(self, detection):
        self.detection = detection

    def detect(self, frame):
        return self.detection

    def detect_geometry(self, frame):
        return self.detection


class FakeClock:
    def __init__(self, step=0.01):
        self.now = 0.0
        self.step = step

    def __call__(self):
        self.now += self.step
        return self.now


class EventStore(InventoryStore):
    def __init__(self, expected_slots, events):
        super().__init__(expected_slots)
        self.events = events

    def add(self, *args, **kwargs):
        self.events.append("store_add")
        return super().add(*args, **kwargs)


class FlightRealsense:
    def __init__(self):
        self.confidence = 3
        self.velocity = (0.0, 0.0, 0.0)

    def get_tracking_confidence(self):
        return self.confidence

    def get_velocity(self):
        return self.velocity


class FakeDriver:
    def __init__(self):
        self.holds = []
        self.aborted = False
        self.commands = []

    def hold_position(self, z_m):
        self.holds.append(z_m)

    def abort_to_land(self):
        self.aborted = True

    def set_speed(self, x, y, yaw, z):
        self.commands.append((x, y, yaw, z))


class CallbackCoordinator:
    def __init__(self, action=controller.WaypointArrivalAction.ADVANCE):
        self.state_machine = SimpleNamespace(state=InventoryState.TRANSIT)
        self.calls = []
        self.driver = None
        self.action = action

    def attach_driver(self, driver):
        self.driver = driver

    def on_waypoint_arrived(self, index, position, reason):
        self.calls.append((index, position, reason))
        return self.action


def _machine():
    trace = StateTrace(stream=io.StringIO(), config=StateDebugConfig(debug_enabled=True))
    ground = FakeGround()
    machine = InventoryStateMachine(trace, ground)
    for state, reason in (
        (InventoryState.WAIT_BUTTON, "test"),
        (InventoryState.INIT_FLIGHT_HW, "button"),
        (InventoryState.PREFLIGHT, "hardware"),
        (InventoryState.WARNING_5S, "preflight"),
        (InventoryState.TAKEOFF, "warning_done"),
        (InventoryState.TRANSIT, "takeoff"),
    ):
        machine.transition(state, reason)
    return machine


def _detection(number=1):
    corners = ((10.0, 10.0), (90.0, 10.0), (90.0, 90.0), (10.0, 90.0))
    return QRDetection(number, f"qr-{number}", corners)


def _coordinator(
    route,
    laser,
    camera=None,
    store=None,
    clock=None,
    config=None,
    decoder=None,
    consensus=None,
):
    events = laser.events
    return controller.InventoryMissionCoordinator(
        route=route,
        state_machine=_machine(),
        gimbal=FakeGimbal(),
        laser=laser,
        camera=camera or FakeCamera([FakeFrame()] * 5),
        decoder=decoder or FakeDecoder(_detection()),
        consensus=consensus or QRConsensus(
            QRConsensusConfig(window_size=3, required_count=2, laser_margin_px=0)
        ),
        store=store or EventStore({"A1"}, events),
        config=config or controller.InventoryMissionConfig(scan_timeout_s=1.0, scan_poll_s=0.0),
        clock=clock or FakeClock(),
        sleep_fn=lambda _: None,
    )


def _finish_scan(coordinator, index, position):
    action = coordinator.on_waypoint_arrived(index, position, "arrival")
    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    result = coordinator.wait_scan_for_test(coordinator.active_scan_generation, timeout_s=1.0)
    assert result is not None
    return coordinator.consume_scan_result(result, position)


def test_camera_fake_capture_does_not_require_cv2_constants(monkeypatch):
    class Capture:
        def isOpened(self):
            return True

        def read(self):
            return False, None

        def release(self):
            pass

    monkeypatch.setattr(controller, "cv2", None)
    camera = controller.CameraSource(
        controller.InventoryMissionConfig(camera_device="fake"),
        capture_factory=lambda device: Capture(),
    )
    assert camera.start() is True
    camera.close()


def test_camera_source_returns_latest_warmed_frame_and_closes_capture_thread():
    class Capture:
        def __init__(self):
            self.released = False
            self.reads = 0

        def isOpened(self):
            return True

        def read(self):
            self.reads += 1
            return True, np.full((4, 4, 3), self.reads, dtype=np.uint8)

        def set(self, *_):
            return True

        def release(self):
            self.released = True

    capture = Capture()
    camera = controller.CameraSource(
        controller.InventoryMissionConfig(camera_warmup_frames=1),
        capture_factory=lambda device: capture,
    )
    assert camera.start() is True
    first = camera.read()
    assert first is not None
    assert int(first[0, 0, 0]) >= 1
    camera.close()
    assert capture.released is True
    assert camera.read() is None


def test_scan_skips_duplicate_latest_frame_and_processes_next_sequence():
    laser = FakeLaser([])
    frame = FakeFrame()
    camera = SequencedCamera(
        [
            (1, frame, 10.0),
            (1, frame, 10.0),  # Same scan frame must not be decoded twice.
            (2, frame, 10.1),
        ]
    )

    class OneClearFrameDecoder(FakeDecoder):
        def __init__(self):
            super().__init__(None)
            self.calls = 0

        def detect(self, frame, target_point=None):
            self.calls += 1
            return _detection(1) if self.calls == 2 else None

    decoder = OneClearFrameDecoder()
    route = [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        laser,
        camera=camera,
        decoder=decoder,
        consensus=QRConsensus(
            QRConsensusConfig(window_size=1, required_count=1, laser_margin_px=0)
        ),
        config=controller.InventoryMissionConfig(scan_timeout_s=1.0, scan_poll_s=0.0),
    )

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival")
    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    result = coordinator.wait_scan_for_test(coordinator.active_scan_generation, timeout_s=1.0)
    consumed = coordinator.consume_scan_result(result, [0, 0, 1.4])
    assert consumed.outcome == controller.ScanConsumeOutcome.ADVANCE
    assert decoder.calls == 2


def test_scan_worker_skips_duplicate_sequences_and_uses_private_consensus():
    frame = FakeFrame()
    camera = SequencedCamera(
        [(1, frame, 10.0), (1, frame, 10.0), (2, frame, 10.1)]
    )

    class CountingDecoder:
        def __init__(self):
            self.calls = 0

        def detect(self, frame, target_point=None):
            self.calls += 1
            return _detection(1)

    decoder = CountingDecoder()
    shared_consensus = QRConsensus(
        QRConsensusConfig(window_size=2, required_count=2, laser_margin_px=0)
    )
    # Poison the injected instance: a worker reusing it would succeed after one frame.
    shared_consensus.update(_detection(1), (50.0, 50.0))
    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        FakeLaser([]),
        camera=camera,
        decoder=decoder,
        consensus=shared_consensus,
        config=controller.InventoryMissionConfig(scan_timeout_s=1.0, scan_poll_s=0.0),
    )

    request = coordinator.start_scan(0, route[0], [0, 0, 1.25])
    result = coordinator.wait_scan_for_test(request.generation, timeout_s=1.0)

    assert result.status == controller.ScanTaskStatus.SUCCEEDED
    assert decoder.calls == 2
    assert result.processed_frames == 2
    assert coordinator.consensus is shared_consensus


def test_scan_worker_converts_decoder_exception_to_failed_result():
    class BrokenDecoder:
        def detect(self, frame, target_point=None):
            raise RuntimeError("decode exploded")

    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(route, FakeLaser([]), decoder=BrokenDecoder())
    request = coordinator.start_scan(0, route[0], [0, 0, 1.25])
    result = coordinator.wait_scan_for_test(request.generation, timeout_s=1.0)

    assert result.status == controller.ScanTaskStatus.FAILED
    assert result.error_code == "decode_exception"
    assert "decode exploded" in result.error_detail


def test_cancelled_generation_rejects_late_worker_result():
    release = threading.Event()

    class BlockingDecoder:
        def detect(self, frame, target_point=None):
            release.wait(1.0)
            return None

    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(route, FakeLaser([]), decoder=BlockingDecoder())
    request = coordinator.start_scan(0, route[0], [0, 0, 1.25])
    coordinator.cancel_scan("test_cancel", join_timeout_s=0.0)
    coordinator._publish_scan_result(
        controller.ScanResult(
            request.generation,
            request.waypoint_index,
            request.slot_label,
            controller.ScanTaskStatus.SUCCEEDED,
            detection=_detection(1),
        )
    )

    assert coordinator.poll_scan_result(request.generation) is None
    release.set()
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)


def test_new_scan_is_refused_while_old_worker_is_alive():
    release = threading.Event()

    class BlockingDecoder:
        def detect(self, frame, target_point=None):
            release.wait(1.0)
            return None

    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(route, FakeLaser([]), decoder=BlockingDecoder())
    first = coordinator.start_scan(0, route[0], [0, 0, 1.25])

    with pytest.raises(RuntimeError, match="scan worker still active"):
        coordinator.start_scan(0, route[0], [0, 0, 1.25])

    assert first.generation == 1
    release.set()
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)


def test_inspect_arrival_enters_scan_without_waiting():
    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(route, FakeLaser([]))

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.25], "arrival")

    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    assert coordinator.active_scan_generation == 1
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)


def test_heading_recovery_invalidates_scan_without_waiting_for_worker():
    release = threading.Event()

    class BlockingDecoder:
        def detect(self, frame, target_point=None):
            release.wait(1.0)
            return None

    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(route, FakeLaser([]), decoder=BlockingDecoder())
    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.25], "arrival")

    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    assert coordinator.state_machine.state == InventoryState.VERIFY_QR
    assert coordinator.scan_worker_active is True

    coordinator.abort_scan_for_heading_recovery()

    assert coordinator.state_machine.state == InventoryState.TRANSIT
    assert coordinator.active_scan_generation is None
    assert coordinator.scan_worker_active is True

    release.set()
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)
    assert coordinator.scan_worker_active is False


def test_fov_precheck_three_negative_frames_still_enters_full_scan():
    class GeometryMissDecoder:
        def __init__(self):
            self.geometry_calls = 0

        def detect_geometry(self, frame, decode_content=False):
            assert decode_content is False
            self.geometry_calls += 1
            return None

        def detect(self, frame, target_point=None):
            return None

    decoder = GeometryMissDecoder()
    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        FakeLaser([]),
        camera=FakeCamera([FakeFrame() for _ in range(12)]),
        decoder=decoder,
        clock=FakeClock(step=0.01),
        config=controller.InventoryMissionConfig(
            scan_timeout_s=1.0,
            scan_poll_s=0.0,
            fov_precheck_enabled=True,
        ),
    )
    driver = FakeDriver()
    coordinator.attach_driver(driver)

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.25], "arrival")

    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    assert coordinator.state_machine.state == InventoryState.VERIFY_QR
    assert coordinator.active_scan_generation == 1
    assert decoder.geometry_calls == 3
    assert driver.holds == [1.25]
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)


def test_fov_precheck_geometry_hit_stops_early_and_saves_diagnostic():
    calls = []

    class GeometryHitDecoder:
        def detect_geometry(self, frame, decode_content=False):
            calls.append(("geometry", decode_content))
            return _detection(None)

        def detect(self, frame, target_point=None):
            return None

    class VisionDebug:
        def capture_scan(self, frame, metadata):
            calls.append(("capture", metadata.copy()))

    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        FakeLaser([]),
        camera=FakeCamera([FakeFrame() for _ in range(8)]),
        decoder=GeometryHitDecoder(),
        clock=FakeClock(step=0.01),
        config=controller.InventoryMissionConfig(
            scan_timeout_s=1.0,
            scan_poll_s=0.0,
            fov_precheck_enabled=True,
        ),
    )
    coordinator.vision_debug = VisionDebug()

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.25], "arrival")

    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    assert calls[0] == ("geometry", False)
    capture = calls[1][1]
    assert capture["capture"] == "fov_precheck"
    assert capture["slot_label"] == "A1"
    assert capture["frame_index"] == 1
    assert capture["geometry_seen"] is True
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)


def test_fov_precheck_exception_still_enters_full_scan():
    class ExplodingGeometryDecoder:
        def detect_geometry(self, frame, decode_content=False):
            raise RuntimeError("geometry backend failed")

        def detect(self, frame, target_point=None):
            return None

    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        FakeLaser([]),
        camera=FakeCamera([FakeFrame() for _ in range(6)]),
        decoder=ExplodingGeometryDecoder(),
        clock=FakeClock(step=0.01),
        config=controller.InventoryMissionConfig(
            scan_timeout_s=1.0,
            scan_poll_s=0.0,
            fov_precheck_enabled=True,
        ),
    )

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.25], "arrival")

    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    assert coordinator.state_machine.state == InventoryState.VERIFY_QR
    assert coordinator.active_scan_generation == 1
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)


def test_fov_precheck_is_disabled_by_default_for_flight():
    class MustNotRunGeometryDecoder:
        def detect_geometry(self, frame, decode_content=False):
            raise AssertionError("default flight path must not run synchronous geometry")

        def detect(self, frame, target_point=None):
            return None

    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        FakeLaser([]),
        camera=FakeCamera([FakeFrame() for _ in range(6)]),
        decoder=MustNotRunGeometryDecoder(),
    )

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.25], "arrival")

    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    assert coordinator.state_machine.state == InventoryState.VERIFY_QR
    coordinator.cancel_scan("cleanup", join_timeout_s=1.0)


def test_scan_success_side_effects_are_consumed_once_on_flight_thread():
    events = []
    route = [MissionWaypoint(FlightPoint(0, 0, 1.25), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        FakeLaser(events),
        consensus=QRConsensus(
            QRConsensusConfig(window_size=1, required_count=1, laser_margin_px=0)
        ),
    )
    request = coordinator.start_scan(0, route[0], [0, 0, 1.25])
    coordinator._go(InventoryState.APPROACH_SLOT, "test")
    coordinator._go(InventoryState.VISUAL_ALIGN, "test")
    coordinator._go(InventoryState.VERIFY_QR, "test")
    result = coordinator.wait_scan_for_test(request.generation, timeout_s=1.0)

    first = coordinator.consume_scan_result(result, [0, 0, 1.25])
    second = coordinator.consume_scan_result(result, [0, 0, 1.25])

    assert first.outcome == controller.ScanConsumeOutcome.ADVANCE
    assert second.outcome == controller.ScanConsumeOutcome.IGNORED
    assert events.count("laser_pulse") == 1
    assert events.count("store_add") == 1


def test_verify_qr_captures_frame_before_decoding():
    calls = []

    class OrderedDecoder:
        def detect(self, frame, target_point=None):
            calls.append("decoder.detect")
            return _detection(1)

    class OrderedVisionDebug:
        def capture_scan(self, frame, metadata):
            calls.append("capture_scan")
            assert metadata["state"] == InventoryState.VERIFY_QR.value

    laser = FakeLaser([])
    route = [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        laser,
        camera=FakeCamera([FakeFrame() for _ in range(20)]),
        decoder=OrderedDecoder(),
        consensus=QRConsensus(
            QRConsensusConfig(window_size=1, required_count=1, laser_margin_px=0)
        ),
    )
    coordinator.vision_debug = OrderedVisionDebug()

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival")
    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    result = coordinator.wait_scan_for_test(coordinator.active_scan_generation, timeout_s=1.0)
    assert result.status == controller.ScanTaskStatus.SUCCEEDED
    assert calls[:2] == ["capture_scan", "decoder.detect"]


def test_inspect_pulses_laser_before_persisting_and_reaches_end():
    events = []
    laser = FakeLaser(events)
    route = [
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.SET_GIMBAL, FaceId.A),
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1"),
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.LAND_APPROACH),
        MissionWaypoint(FlightPoint(0, 0, 0.2), WaypointKind.LAND),
    ]
    coordinator = _coordinator(route, laser)
    assert coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival")
    consumed = _finish_scan(coordinator, 1, [0, 0, 1.4])
    assert consumed.outcome == controller.ScanConsumeOutcome.ADVANCE
    assert events == ["laser_pulse", "laser_wait", "store_add"]
    assert coordinator.store.query_cargo(1).slot_label == "A1"
    assert coordinator.state_machine.state == InventoryState.RETURN
    assert coordinator.on_waypoint_arrived(2, [0, 0, 1.4], "arrival")
    assert coordinator.on_waypoint_arrived(3, [0, 0, 0.2], "arrival")
    assert coordinator.state_machine.state == InventoryState.LAND
    coordinator.state_machine.transition(InventoryState.END, "land_confirmed")


def test_face_change_transit_is_allowed_after_report():
    laser = FakeLaser([])
    route = [
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.SET_GIMBAL, FaceId.A),
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1"),
        MissionWaypoint(FlightPoint(0, 1, 1.4), WaypointKind.TRANSIT),
        MissionWaypoint(FlightPoint(0, 1, 1.4), WaypointKind.SET_GIMBAL, FaceId.B),
    ]
    coordinator = _coordinator(route, laser)
    assert coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival")
    consumed = _finish_scan(coordinator, 1, [0, 0, 1.4])
    assert consumed.outcome == controller.ScanConsumeOutcome.ADVANCE
    assert coordinator.state_machine.state == InventoryState.TRANSIT
    assert coordinator.on_waypoint_arrived(2, [0, 1, 1.4], "arrival")
    assert coordinator.on_waypoint_arrived(3, [0, 1, 1.4], "arrival")
    assert coordinator.gimbal.faces == [FaceId.A, FaceId.B]
    assert coordinator.state_machine.state == InventoryState.APPROACH_SLOT


def test_laser_failure_does_not_persist_result_and_faults():
    events = []
    laser = FakeLaser(events, pulse_result=False)
    route = [
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.SET_GIMBAL, FaceId.A),
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1"),
    ]
    coordinator = _coordinator(route, laser)
    driver = FakeDriver()
    coordinator.attach_driver(driver)
    coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival")
    consumed = _finish_scan(coordinator, 1, [0, 0, 1.4])
    assert consumed.outcome == controller.ScanConsumeOutcome.RETURN
    assert consumed.error_code == "laser_pulse_failed"
    assert coordinator.store.by_slot == {}
    assert driver.aborted is False


def test_scan_timeout_faults_and_keeps_store_empty():
    laser = FakeLaser([])
    route = [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        laser,
        camera=FakeCamera([]),
        clock=FakeClock(step=0.2),
    )
    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival")
    assert action == controller.WaypointArrivalAction.ENTER_SCAN
    result = coordinator.wait_scan_for_test(coordinator.active_scan_generation, timeout_s=1.0)
    consumed = coordinator.consume_scan_result(result, [0, 0, 1.4])
    # 扫码失败改为跳过该货位（ADVANCE），不再触发 RETURN
    assert consumed.outcome == controller.ScanConsumeOutcome.ADVANCE
    assert consumed.error_code == "qr_timeout"
    assert coordinator.store.by_slot == {}


def test_visual_servo_is_rejected_with_async_scan():
    laser = FakeLaser([])
    route = [
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1")
    ]
    detection = QRDetection(1, "qr-1", ((40.0, 20.0), (80.0, 20.0), (80.0, 60.0), (40.0, 60.0)))
    centered_detection = QRDetection(
        1, "qr-1", ((30.0, 30.0), (70.0, 30.0), (70.0, 70.0), (30.0, 70.0))
    )

    class MovingDecoder(FakeDecoder):
        def __init__(self):
            super().__init__(detection)
            self.geometry_calls = 0

        def detect_geometry(self, frame):
            self.geometry_calls += 1
            return detection if self.geometry_calls == 1 else centered_detection
    config = controller.InventoryMissionConfig(
        scan_timeout_s=1.0,
        scan_poll_s=0.0,
        vision_servo=VisionServoConfig(
            enabled=True,
            timeout_s=1.0,
            lost_timeout_s=0.2,
            center_tolerance_px=2.0,
            stable_frames=2,
            x_kp_cmd_per_px=0.1,
            z_kp_m_per_px=0.001,
        ),
    )
    coordinator = _coordinator(
        route,
        laser,
        camera=FakeCamera([FakeFrame()] * 8),
        clock=FakeClock(step=0.01),
        config=config,
        decoder=MovingDecoder(),
    )
    driver = FakeDriver()
    coordinator.attach_driver(driver)
    assert coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival") is False
    assert coordinator.state_machine.state == InventoryState.LAND
    assert driver.aborted is True


def test_visual_servo_ignores_large_target_jump():
    laser = FakeLaser([])
    route = [
        MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1")
    ]
    first = QRDetection(
        None, "", ((250.0, 250.0), (350.0, 250.0), (350.0, 350.0), (250.0, 350.0))
    )
    second = QRDetection(
        None, "", ((650.0, 250.0), (750.0, 250.0), (750.0, 350.0), (650.0, 350.0))
    )

    class JumpingDecoder(FakeDecoder):
        def __init__(self):
            super().__init__(None)
            self.calls = 0

        def detect_geometry(self, frame, decode_content=False):
            self.calls += 1
            return first if self.calls == 1 else second

    config = controller.InventoryMissionConfig(
        scan_timeout_s=1.0,
        scan_poll_s=0.0,
        vision_servo=VisionServoConfig(
            enabled=True,
            timeout_s=0.08,
            lost_timeout_s=1.0,
            max_center_jump_px=100.0,
        ),
    )
    coordinator = _coordinator(
        route,
        laser,
        camera=FakeCamera([FakeFrame()] * 8),
        clock=FakeClock(step=0.01),
        config=config,
        decoder=JumpingDecoder(),
    )
    driver = FakeDriver()
    coordinator.attach_driver(driver)
    assert coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival") is False
    assert coordinator.state_machine.state == InventoryState.LAND


def test_duplicate_qr_skips_slot_and_continues_before_laser():
    events = []
    store = EventStore({"A1", "A2", "B1"}, events)
    store.add(1, "B1", 1.0)
    events.clear()
    decoder = FakeDecoder(_detection(1))
    coordinator = _coordinator(
        [
            MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1"),
            MissionWaypoint(FlightPoint(0.5, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A2"),
        ],
        FakeLaser(events),
        store=store,
        decoder=decoder,
    )
    consumed = _finish_scan(coordinator, 0, [0, 0, 1.4])
    assert consumed.outcome == controller.ScanConsumeOutcome.ADVANCE
    assert consumed.error_code == "qr_duplicate"
    assert events == []
    assert coordinator.store.query_cargo(1).slot_label == "B1"
    assert coordinator.state_machine.state == InventoryState.TRANSIT

    # The next slot remains inspectable; the duplicate did not terminate the
    # mission or leave the scan state machine stuck in VERIFY_QR.
    decoder.detection = _detection(2)
    next_consumed = _finish_scan(coordinator, 1, [0.5, 0, 1.4])
    assert next_consumed.outcome == controller.ScanConsumeOutcome.ADVANCE
    assert coordinator.store.query_cargo(2).slot_label == "A2"
    assert events == ["laser_pulse", "laser_wait", "store_add"]


def test_return_transit_arrival_keeps_return_state():
    route = [MissionWaypoint(FlightPoint(-2.65, 0.05, 1.4), WaypointKind.TRANSIT)]
    coordinator = _coordinator(route, FakeLaser([]))
    coordinator.state_machine.transition(InventoryState.RETURN, "test_return")

    action = coordinator.on_waypoint_arrived(0, [-2.65, 0.05, 1.4], "cruise_arrival")

    assert action == controller.WaypointArrivalAction.ADVANCE
    assert coordinator.state_machine.state == InventoryState.RETURN


def test_return_takeoff_waypoint_warns_but_keeps_return_state(monkeypatch):
    route = [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.TAKEOFF)]
    coordinator = _coordinator(route, FakeLaser([]))
    coordinator.state_machine.transition(InventoryState.RETURN, "test_return")
    warnings = []
    monkeypatch.setattr(controller.logger, "warning", warnings.append)

    action = coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival")

    assert action == controller.WaypointArrivalAction.ADVANCE
    assert coordinator.state_machine.state == InventoryState.RETURN
    assert any("TAKEOFF" in message for message in warnings)


def test_flight_driver_routes_base_arrival_callback_through_coordinator():
    route = [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.TRANSIT)]
    callback = CallbackCoordinator()
    mission = controller.InventoryFlightMission(
        [0] * 14,
        [170, 2, 0, 0, 0, 0, 0, 0, 0, 0, 255],
        None,
        None,
        route,
        callback,
    )
    mission._advance_waypoint("test_arrival", [0.0, 0.0, 1.4], [0.0, 0.0, 1.4], 0.0)
    assert callback.driver is mission
    assert callback.calls == [(0, [0.0, 0.0, 1.4], "test_arrival")]
    assert mission.target_index == 1


def test_inventory_loop_exception_hook_syncs_return_to_land():
    route = [MissionWaypoint(FlightPoint(-2.65, 0.05, 1.4), WaypointKind.TRANSIT)]
    coordinator = _coordinator(route, FakeLaser([]))
    coordinator.state_machine.transition(InventoryState.RETURN, "test_return")
    mission = controller.InventoryFlightMission(
        [0] * 14,
        [170, 2, 0, 51, 51, 140, 51, 0, 51, 0, 255],
        None,
        None,
        route,
        coordinator,
    )
    mission.state = "NAVIGATE"
    mission._navigation_purpose = "return"

    mission.on_flight_loop_exception(ValueError("bad transition"), "NAVIGATE")

    assert mission.state == "LAND"
    assert mission.se_fc[3] == 51
    assert mission.se_fc[4] == 51
    assert coordinator.state_machine.state == InventoryState.LAND


def test_real_mission_navigates_complete_return_route(monkeypatch):
    route = [
        MissionWaypoint(FlightPoint(-2.65, 0.05, 1.4), WaypointKind.TRANSIT),
        MissionWaypoint(FlightPoint(-2.65, 3.5, 1.4), WaypointKind.TRANSIT),
        MissionWaypoint(FlightPoint(-2.5, 3.5, 1.4), WaypointKind.LAND_APPROACH),
    ]
    coordinator = _coordinator(route, FakeLaser([]))
    coordinator.state_machine.transition(InventoryState.RETURN, "test_return")
    realsense = FlightRealsense()
    mission = controller.InventoryFlightMission(
        [0] * 14,
        [170, 2, 0, 51, 51, 140, 51, 0, 51, 0, 255],
        realsense,
        None,
        route,
        coordinator,
    )
    mission.heading_source = "t265"
    mission.t265_ok = True
    mission._navigation_purpose = "return"
    mission.navigation_profile = NavigationProfileConfig(
        profile="precision", cruise_confirm_cycles=2, cruise_radius_m=0.15
    )
    monkeypatch.setattr(mg, "arrival_confirm_need", 1)
    monkeypatch.setattr(mg, "arrival_hold_s", 0.0)

    for _ in range(2):
        mission.navigate([-2.64, 0.05, 1.4], 0.0)
    assert mission.target_index == 1
    assert coordinator.state_machine.state == InventoryState.RETURN

    for _ in range(2):
        mission.navigate([-2.65, 3.49, 1.4], 0.0)
    assert mission.target_index == 2
    assert coordinator.state_machine.state == InventoryState.RETURN

    mission.navigate([-2.5, 3.5, 1.4], 0.0)
    mission.navigate([-2.5, 3.5, 1.4], 0.0)
    assert mission.state == "LAND"
    assert mission.target_index == 2
    assert coordinator.state_machine.state == InventoryState.LAND


def test_return_timeout_near_advances_once_through_coordinator():
    route = [
        MissionWaypoint(FlightPoint(-2.65, 0.05, 1.4), WaypointKind.TRANSIT),
        MissionWaypoint(FlightPoint(-2.5, 3.5, 1.4), WaypointKind.LAND_APPROACH),
    ]
    callback = CallbackCoordinator()
    callback.route = list(route)
    mission = controller.InventoryFlightMission(
        [0] * 14,
        [170, 2, 0, 0, 0, 0, 0, 0, 0, 0, 255],
        None,
        None,
        route,
        callback,
    )
    mission._navigation_purpose = "return"

    mission._advance_waypoint(
        "return_timeout_near",
        [-2.64, 0.05, 1.4],
        [-2.65, 0.05, 1.4],
        0.01,
    )

    assert callback.calls == [
        (0, [-2.64, 0.05, 1.4], "return_timeout_near")
    ]
    assert mission.target_index == 1
    assert mission.inventory_route == route
    assert callback.route == route
    assert mission.targets == [waypoint.point.as_list() for waypoint in route]


def test_return_timeout_near_does_not_advance_when_coordinator_lands():
    route = [
        MissionWaypoint(FlightPoint(-2.65, 0.05, 1.4), WaypointKind.TRANSIT),
        MissionWaypoint(FlightPoint(-2.5, 3.5, 1.4), WaypointKind.LAND_APPROACH),
    ]
    callback = CallbackCoordinator(controller.WaypointArrivalAction.LAND)
    mission = controller.InventoryFlightMission(
        [0] * 14,
        [170, 2, 0, 0, 0, 0, 0, 0, 0, 0, 255],
        None,
        None,
        route,
        callback,
    )
    mission._navigation_purpose = "return"

    mission._advance_waypoint(
        "return_timeout_near",
        [-2.64, 0.05, 1.4],
        [-2.65, 0.05, 1.4],
        0.01,
    )

    assert len(callback.calls) == 1
    assert mission.target_index == 0
    assert mission.state == "LAND"


def test_flight_driver_atomically_replaces_return_route():
    original = [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.TRANSIT)]
    return_route = [
        MissionWaypoint(FlightPoint(-2.65, 0.05, 1.4), WaypointKind.TRANSIT),
        MissionWaypoint(FlightPoint(-2.5, 3.5, 1.4), WaypointKind.LAND_APPROACH),
    ]
    callback = CallbackCoordinator()
    callback.route = list(original)
    mission = controller.InventoryFlightMission(
        [0] * 14,
        [170, 2, 0, 0, 0, 0, 0, 0, 0, 0, 255],
        None,
        None,
        original,
        callback,
    )
    mission.target_index = 1
    mission.last_target_index = 0

    generation = mission.replace_inventory_navigation_route(
        return_route, [-1.75, 0.05, 1.4]
    )

    assert mission.inventory_route == return_route
    assert callback.route == return_route
    assert mission.targets == [tuple(waypoint.point.as_list()) for waypoint in return_route]
    assert mission.target_index == 0
    assert mission.last_target_index == -1
    assert mission._navigation_purpose == "return"
    assert mission.state == "NAVIGATE"
    assert generation == 1


def test_hardware_scan_return_without_route_goes_directly_to_safe_land():
    calls = []

    class EmptyReturnCoordinator:
        def poll_scan_result(self, generation):
            assert generation == 7
            return object()

        def consume_scan_result(self, result, position):
            return controller.ScanConsumeResult(
                controller.ScanConsumeOutcome.RETURN,
                error_code="laser_pulse_failed",
            )

        def _abort(self, code, **fields):
            calls.append((code, fields))
            mission.state = "LAND"

    mission = controller.InventoryFlightMission.__new__(
        controller.InventoryFlightMission
    )
    mission.coordinator = EmptyReturnCoordinator()
    mission._scan_generation = 7
    mission.state = "SCAN"
    mission.end_scan_hold = lambda: calls.append(("scan_hold_ended", {}))

    mission.on_scan_tick([0.0, 0.0, 1.25], 0.0, {})

    assert mission.state == "LAND"
    assert calls == [
        ("scan_hold_ended", {}),
        ("scan_return_route_empty", {"error_code": "laser_pulse_failed"}),
    ]
