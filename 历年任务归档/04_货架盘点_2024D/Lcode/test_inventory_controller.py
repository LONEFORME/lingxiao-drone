import io
from types import SimpleNamespace

import numpy as np

from Lcode import inventory_controller as controller
from Lcode.inventory_planner import MissionWaypoint, WaypointKind
from Lcode.inventory_state import InventoryState, InventoryStateMachine
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
    def __init__(self):
        self.state_machine = SimpleNamespace(state=InventoryState.TRANSIT)
        self.calls = []
        self.driver = None

    def attach_driver(self, driver):
        self.driver = driver

    def on_waypoint_arrived(self, index, position, reason):
        self.calls.append((index, position, reason))
        return True


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


def _coordinator(route, laser, camera=None, store=None, clock=None, config=None, decoder=None):
    events = laser.events
    return controller.InventoryMissionCoordinator(
        route=route,
        state_machine=_machine(),
        gimbal=FakeGimbal(),
        laser=laser,
        camera=camera or FakeCamera([FakeFrame()] * 5),
        decoder=decoder or FakeDecoder(_detection()),
        consensus=QRConsensus(QRConsensusConfig(window_size=3, required_count=2, laser_margin_px=0)),
        store=store or EventStore({"A1"}, events),
        config=config or controller.InventoryMissionConfig(scan_timeout_s=1.0, scan_poll_s=0.0),
        clock=clock or FakeClock(),
        sleep_fn=lambda _: None,
    )


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
    assert coordinator.on_waypoint_arrived(1, [0, 0, 1.4], "arrival")
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
    assert coordinator.on_waypoint_arrived(1, [0, 0, 1.4], "arrival")
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
    assert coordinator.on_waypoint_arrived(1, [0, 0, 1.4], "arrival") is False
    assert coordinator.store.by_slot == {}
    assert driver.aborted is True
    assert coordinator.state_machine.state == InventoryState.LAND


def test_scan_timeout_faults_and_keeps_store_empty():
    laser = FakeLaser([])
    route = [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1")]
    coordinator = _coordinator(
        route,
        laser,
        camera=FakeCamera([]),
        clock=FakeClock(step=0.2),
    )
    assert coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival") is False
    assert coordinator.store.by_slot == {}
    assert coordinator.state_machine.state == InventoryState.LAND


def test_visual_servo_centers_geometry_before_qr_consensus():
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
    assert coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival") is True
    assert any(command[0] == 1 for command in driver.commands)
    assert coordinator.state_machine.state == InventoryState.TRANSIT


def test_duplicate_qr_faults_before_laser():
    events = []
    store = EventStore({"A1", "B1"}, events)
    store.add(1, "B1", 1.0)
    events.clear()
    coordinator = _coordinator(
        [MissionWaypoint(FlightPoint(0, 0, 1.4), WaypointKind.INSPECT, FaceId.A, "A1")],
        FakeLaser(events),
        store=store,
    )
    assert coordinator.on_waypoint_arrived(0, [0, 0, 1.4], "arrival") is False
    assert events == []
    assert coordinator.state_machine.state == InventoryState.LAND


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
