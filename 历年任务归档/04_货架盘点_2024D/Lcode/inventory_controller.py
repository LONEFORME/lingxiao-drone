"""Full warehouse-inventory mission integration.

This module connects the existing flight controller to the warehouse-specific
planner, gimbal, QR, laser, result-store, and state-trace components.  The
route-only entry point intentionally does not import this controller.
"""

import math
import os
import threading
import time
from collections import Counter
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional, Tuple

from Lcode.ground_link import GroundMessageType
from Lcode.inventory_planner import InventoryPlanner, MissionWaypoint, WaypointKind
from Lcode.inventory_state import ALLOWED_TRANSITIONS, InventoryState, InventoryStateMachine
from Lcode.inventory_store import InventoryConflict, InventoryStore
from Lcode.laser_pointer import LaserPointer
from Lcode.Logger import logger
from Lcode.qr_vision import QRConsensus, QRDecoder, QRDetection
from Lcode.sensor_gimbal import SensorGimbal
from Lcode.warehouse_model import FlightPoint
from Lcode.vision_servo import VisionServoConfig, VisionServoResult, servo_command
from Mission_GPT import mission as FlightMission

try:
    import cv2
except ImportError:
    cv2 = None


class WaypointArrivalAction(str, Enum):
    ADVANCE = "advance"
    ENTER_SCAN = "enter_scan"
    LAND = "land"
    STOP = "stop"


class ScanConsumeOutcome(str, Enum):
    RUNNING = "running"
    ADVANCE = "advance"
    RETURN = "return"
    EMERGENCY_LAND = "emergency_land"
    IGNORED = "ignored"


@dataclass(frozen=True)
class ScanConsumeResult:
    outcome: ScanConsumeOutcome
    return_route: tuple = ()
    error_code: Optional[str] = None


class ScanTaskStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CONSUMED = "consumed"


@dataclass(frozen=True)
class ScanRequest:
    generation: int
    waypoint_index: int
    slot_label: str
    hold_target: Tuple[float, float, float]
    timeout_s: float
    started_monotonic: float


@dataclass(frozen=True)
class ScanResult:
    generation: int
    waypoint_index: int
    slot_label: str
    status: ScanTaskStatus
    detection: Optional[QRDetection] = None
    error_code: Optional[str] = None
    error_detail: Optional[str] = None
    processed_frames: int = 0
    started_monotonic: float = 0.0
    finished_monotonic: float = 0.0
    decode_stats: Optional[dict] = None


@dataclass(frozen=True)
class InventoryMissionConfig:
    # The deployed UVC camera enumerates as video0 on the flight board.
    # Keep DRONE_CAMERA_DEVICE as an override for alternate capture nodes.
    camera_device: str = "/dev/video0"
    camera_width: int = 1280
    camera_height: int = 720
    camera_fps: int = 15
    # This board's UVC driver uses V4L2 values: 1=manual, 3=aperture priority.
    # Default to automatic exposure because the camera can otherwise be left at
    # exposure_time_absolute=50, which is too dark for the QR modules in flight.
    camera_auto_exposure: Optional[float] = 3.0
    camera_exposure: Optional[float] = None
    camera_gain: Optional[float] = None
    camera_auto_focus: Optional[float] = None
    camera_focus: Optional[float] = None
    camera_zoom: Optional[float] = None
    camera_warmup_frames: int = 10
    qr_mapping_file: str = "qr_mapping.txt"
    scan_timeout_s: float = 8.0
    scan_poll_s: float = 0.03
    laser_aim_x_ratio: float = 0.5
    laser_aim_y_ratio: float = 0.5
    vision_servo: VisionServoConfig = None
    async_qr_scan: bool = True
    fov_precheck_enabled: bool = False
    qr_decode_profile: str = "variants"

    def __post_init__(self):
        if self.vision_servo is None:
            object.__setattr__(self, "vision_servo", VisionServoConfig.from_env())
        if self.camera_width < 1 or self.camera_height < 1 or self.camera_fps < 1:
            raise ValueError("摄像头参数必须为正数")
        if self.camera_warmup_frames < 0:
            raise ValueError("camera_warmup_frames must not be negative")
        if self.scan_timeout_s < 1.0:
            raise ValueError("扫码超时不能小于 1 秒")
        if not 0.0 <= self.laser_aim_x_ratio <= 1.0:
            raise ValueError("激光瞄准点 X 比例必须在 [0,1] 内")
        if not 0.0 <= self.laser_aim_y_ratio <= 1.0:
            raise ValueError("激光瞄准点 Y 比例必须在 [0,1] 内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ

        def optional_float(name):
            raw = env.get(name, "").strip()
            return None if not raw else float(raw)

        raw_async = env.get("DRONE_ASYNC_QR_SCAN", "1").strip().lower()
        if raw_async not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_ASYNC_QR_SCAN只能是0/1/false/true")
        raw_fov_precheck = env.get("DRONE_QR_FOV_PRECHECK", "0").strip().lower()
        if raw_fov_precheck not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_QR_FOV_PRECHECK只能是0/1/false/true")
        qr_decode_profile = env.get(
            "DRONE_QR_DECODE_PROFILE", "variants"
        ).strip().lower()
        if qr_decode_profile not in {"raw", "variants"}:
            raise ValueError("DRONE_QR_DECODE_PROFILE只能是raw/variants")
        return cls(
            camera_device=env.get("DRONE_CAMERA_DEVICE", "/dev/video0"),
            camera_width=int(env.get("DRONE_CAMERA_WIDTH", "1280")),
            camera_height=int(env.get("DRONE_CAMERA_HEIGHT", "720")),
            camera_fps=int(env.get("DRONE_CAMERA_FPS", "15")),
            camera_auto_exposure=float(env.get("DRONE_CAMERA_AUTO_EXPOSURE", "3")),
            camera_exposure=optional_float("DRONE_CAMERA_EXPOSURE"),
            camera_gain=optional_float("DRONE_CAMERA_GAIN"),
            camera_auto_focus=optional_float("DRONE_CAMERA_AUTOFOCUS"),
            camera_focus=optional_float("DRONE_CAMERA_FOCUS"),
            camera_zoom=optional_float("DRONE_CAMERA_ZOOM"),
            camera_warmup_frames=int(env.get("DRONE_CAMERA_WARMUP_FRAMES", "10")),
            qr_mapping_file=env.get("DRONE_QR_MAPPING_FILE", "qr_mapping.txt"),
            scan_timeout_s=float(env.get("DRONE_QR_SCAN_TIMEOUT_S", "8.0")),
            scan_poll_s=float(env.get("DRONE_QR_SCAN_POLL_S", "0.03")),
            laser_aim_x_ratio=float(env.get("DRONE_LASER_AIM_X_RATIO", "0.5")),
            laser_aim_y_ratio=float(env.get("DRONE_LASER_AIM_Y_RATIO", "0.5")),
            vision_servo=VisionServoConfig.from_env(env),
            async_qr_scan=raw_async in {"1", "true"},
            fov_precheck_enabled=raw_fov_precheck in {"1", "true"},
            qr_decode_profile=qr_decode_profile,
        )


def require_async_scan_for_flight(config, dry_run=False):
    if not dry_run and not config.async_qr_scan:
        raise RuntimeError("async_scan_required")


class CameraSource:
    """Small, injectable OpenCV camera adapter used by the scan state."""

    def __init__(self, config=None, capture_factory=None):
        self.config = config or InventoryMissionConfig.from_env()
        self._capture_factory = capture_factory
        self._capture = None
        self._capture_thread = None
        self._capture_stop = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_sequence = 0
        self._frame_timestamp = None
        self.started = False

    @property
    def device(self):
        value = self.config.camera_device
        return int(value) if str(value).isdigit() else value

    def start(self) -> bool:
        if cv2 is None and self._capture_factory is None:
            logger.error("二维码摄像头需要 OpenCV")
            return False
        try:
            factory = self._capture_factory or cv2.VideoCapture
            self._capture = factory(self.device)
            if hasattr(self._capture, "isOpened") and not self._capture.isOpened():
                logger.error(f"二维码摄像头打开失败: {self.config.camera_device}")
                self.close()
                return False
            if hasattr(self._capture, "set") and cv2 is not None:
                # The UVC camera opens as 320x240 YUYV by default. Select
                # MJPEG before requesting the scan resolution; otherwise
                # OpenCV may silently keep the low-resolution mode.
                self._capture.set(
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*"MJPG"),
                )
                self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.camera_width)
                self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.camera_height)
                self._capture.set(cv2.CAP_PROP_FPS, self.config.camera_fps)
                self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if self.config.camera_auto_exposure is not None:
                    self._capture.set(
                        cv2.CAP_PROP_AUTO_EXPOSURE,
                        self.config.camera_auto_exposure,
                    )
                if self.config.camera_exposure is not None:
                    self._capture.set(cv2.CAP_PROP_EXPOSURE, self.config.camera_exposure)
                if self.config.camera_gain is not None:
                    self._capture.set(cv2.CAP_PROP_GAIN, self.config.camera_gain)
                if (
                    self.config.camera_auto_focus is not None
                    and hasattr(cv2, "CAP_PROP_AUTOFOCUS")
                ):
                    self._capture.set(
                        cv2.CAP_PROP_AUTOFOCUS,
                        self.config.camera_auto_focus,
                    )
                if (
                    self.config.camera_focus is not None
                    and hasattr(cv2, "CAP_PROP_FOCUS")
                ):
                    self._capture.set(cv2.CAP_PROP_FOCUS, self.config.camera_focus)
                if self.config.camera_zoom is not None and hasattr(cv2, "CAP_PROP_ZOOM"):
                    # The deployed UVC camera exposes zoom_absolute 0..3.
                    # Keep this opt-in because some alternate cameras reject
                    # CAP_PROP_ZOOM entirely.
                    self._capture.set(cv2.CAP_PROP_ZOOM, self.config.camera_zoom)
                # Give the UVC driver/ISP time to settle before QR processing.
                for _ in range(self.config.camera_warmup_frames):
                    ok, frame = self._capture.read()
                    if ok and frame is not None:
                        self._store_frame(frame)
            self._capture_stop.clear()
            self.started = True
            self._capture_thread = threading.Thread(
                target=self._capture_loop,
                name="inventory-camera-capture",
                daemon=True,
            )
            self._capture_thread.start()
            return True
        except Exception as exc:
            logger.error(f"二维码摄像头初始化失败: {exc}")
            self.close()
            return False

    def _capture_loop(self):
        while not self._capture_stop.is_set() and self._capture is not None:
            try:
                ok, frame = self._capture.read()
            except Exception as exc:
                logger.error(f"二维码摄像头后台取帧失败: {exc}")
                time.sleep(0.01)
                continue
            if ok and frame is not None:
                self._store_frame(frame)
            else:
                time.sleep(0.01)

    def _store_frame(self, frame):
        with self._frame_lock:
            self._frame_sequence += 1
            self._latest_frame = frame
            self._frame_timestamp = time.monotonic()

    def read_with_sequence(self):
        """Return the latest frame and its capture sequence/timestamp.

        The capture thread runs independently of QR decoding.  Consumers can
        use the sequence to avoid decoding the same latest frame repeatedly.
        """
        if not self.started:
            return None, None, None
        with self._frame_lock:
            if self._latest_frame is None:
                return None, None, None
            return (
                self._frame_sequence,
                self._latest_frame.copy(),
                self._frame_timestamp,
            )

    def read(self):
        _, frame, _ = self.read_with_sequence()
        return frame

    def laser_aim_point(self, frame):
        height, width = frame.shape[:2]
        return (
            width * self.config.laser_aim_x_ratio,
            height * self.config.laser_aim_y_ratio,
        )

    def close(self):
        self._capture_stop.set()
        if self._capture_thread is not None and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        self._capture_thread = None
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
        self._capture = None
        self.started = False
        with self._frame_lock:
            self._latest_frame = None
            self._frame_sequence = 0
            self._frame_timestamp = None


class InventoryMissionCoordinator:
    """Procedural actions executed when the flight driver reaches a waypoint."""

    FOV_CHECK_MAX_FRAMES = 3
    FOV_CHECK_BUDGET_S = 0.45

    def __init__(
        self,
        route,
        state_machine: InventoryStateMachine,
        gimbal: SensorGimbal,
        laser: LaserPointer,
        camera: CameraSource,
        decoder: QRDecoder,
        consensus: QRConsensus,
        store: InventoryStore,
        ground_link=None,
        config=None,
        clock=time.monotonic,
        sleep_fn=time.sleep,
        vision_debug=None,
        planner=None,
    ):
        self.route = list(route)
        self.state_machine = state_machine
        self.gimbal = gimbal
        self.laser = laser
        self.camera = camera
        self.decoder = decoder
        self.consensus = consensus
        self.store = store
        self.ground_link = ground_link
        self.config = config or InventoryMissionConfig.from_env()
        self._clock = clock
        self._sleep = sleep_fn
        self.vision_debug = vision_debug
        self.planner = planner or InventoryPlanner()
        self.driver = None
        self.last_detection = None
        self._scan_lock = threading.Lock()
        self._scan_generation = 0
        self._scan_request = None
        self._scan_result = None
        self._scan_thread = None
        self._scan_cancel = None
        self._scan_hard_deadline = None

    def attach_driver(self, driver):
        self.driver = driver

    def start_scan(self, waypoint_index, waypoint: MissionWaypoint, position):
        """Start one isolated QR worker without performing mission side effects."""
        hold_target = tuple(float(value) for value in position)
        if len(hold_target) != 3:
            raise ValueError("scan hold target must contain x, y, z")
        if waypoint.slot_label is None:
            raise ValueError("scan waypoint must have a slot label")

        with self._scan_lock:
            if self._scan_thread is not None and self._scan_thread.is_alive():
                raise RuntimeError("scan worker still active")
            self._scan_generation += 1
            started = self._clock()
            request = ScanRequest(
                generation=self._scan_generation,
                waypoint_index=int(waypoint_index),
                slot_label=waypoint.slot_label,
                hold_target=hold_target,
                timeout_s=self.config.scan_timeout_s,
                started_monotonic=started,
            )
            cancel_event = threading.Event()
            worker = threading.Thread(
                target=self._scan_worker,
                args=(request, cancel_event),
                name=f"inventory-qr-scan-{request.generation}",
                daemon=True,
            )
            self._scan_request = request
            self._scan_result = None
            self._scan_cancel = cancel_event
            self._scan_thread = worker
            self._scan_hard_deadline = started + request.timeout_s
            worker.start()
            return request

    def _scan_worker(self, request, cancel_event):
        consensus = QRConsensus(self.consensus.config)
        last_sequence = None
        processed = 0
        stats = {"decoded": 0, "laser_outside": 0, "unknown_mapping": 0, "max_consensus": 0}

        def publish_failure(code, detail=None):
            stats["max_consensus"] = max(
                stats["max_consensus"],
                max(
                    (v for v in Counter(
                        item.number for item in consensus._window if item is not None
                    ).values()),
                    default=0,
                ),
            )
            self._publish_scan_result(
                ScanResult(
                    request.generation,
                    request.waypoint_index,
                    request.slot_label,
                    ScanTaskStatus.FAILED,
                    error_code=code,
                    error_detail=detail,
                    processed_frames=processed,
                    started_monotonic=request.started_monotonic,
                    finished_monotonic=self._clock(),
                    decode_stats=dict(stats),
                )
            )

        while not cancel_event.is_set():
            if self._clock() >= request.started_monotonic + request.timeout_s:
                publish_failure("qr_timeout")
                return
            try:
                if hasattr(self.camera, "read_with_sequence"):
                    sequence, frame, frame_timestamp = self.camera.read_with_sequence()
                else:
                    sequence, frame_timestamp = None, None
                    frame = self.camera.read()
            except Exception as exc:
                publish_failure("camera_read_exception", str(exc))
                return

            if frame is None or (sequence is not None and sequence == last_sequence):
                self._sleep(self.config.scan_poll_s)
                continue
            last_sequence = sequence
            processed += 1
            metadata = {
                "state": InventoryState.VERIFY_QR.value,
                "capture": "scan_frame",
                "waypoint_index": request.waypoint_index,
                "slot_label": request.slot_label,
                "position": list(request.hold_target),
                "frame_sequence": sequence,
                "capture_timestamp": frame_timestamp,
                "processed_frame_count": processed,
                "frame_shape": list(frame.shape),
                "timestamp": time.time(),
            }
            if self.vision_debug is not None:
                try:
                    self.vision_debug.capture_scan(frame, metadata)
                except Exception as exc:
                    logger.warning(f"二维码调试图保存失败: {exc}")

            try:
                aim = self.camera.laser_aim_point(frame)
                try:
                    detection = self.decoder.detect(frame, target_point=aim)
                except TypeError:
                    detection = self.decoder.detect(frame)
                accepted = consensus.update(detection, aim)
                # 每帧解码统计
                if detection is not None and detection.number is not None:
                    stats["decoded"] += 1
                elif detection is not None:
                    stats["unknown_mapping"] += 1
                # 跟踪当前共识最高计数
                cur_counts = Counter(
                    item.number for item in consensus._window if item is not None
                ).values() if consensus._window else ()
                if cur_counts:
                    stats["max_consensus"] = max(stats["max_consensus"], max(cur_counts))
            except Exception as exc:
                publish_failure("decode_exception", str(exc))
                return
            if accepted is not None:
                self._publish_scan_result(
                    ScanResult(
                        request.generation,
                        request.waypoint_index,
                        request.slot_label,
                        ScanTaskStatus.SUCCEEDED,
                        detection=accepted,
                        processed_frames=processed,
                        started_monotonic=request.started_monotonic,
                        finished_monotonic=self._clock(),
                        decode_stats=dict(stats),
                    )
                )
                return
            self._sleep(self.config.scan_poll_s)

    def _publish_scan_result(self, result):
        with self._scan_lock:
            if self._scan_request is None:
                return
            if result.generation != self._scan_request.generation:
                return
            if result.generation != self._scan_generation:
                return
            if self._scan_result is not None:
                return
            self._scan_result = result

    def poll_scan_result(self, generation):
        with self._scan_lock:
            if generation != self._scan_generation:
                return None
            if self._scan_request is None or generation != self._scan_request.generation:
                return None
            return self._scan_result

    def cancel_scan(self, reason, join_timeout_s=2.0):
        """Invalidate the active generation and wait at most two seconds."""
        with self._scan_lock:
            worker = self._scan_thread
            cancel_event = self._scan_cancel
            self._scan_generation += 1
            self._scan_request = None
            self._scan_result = None
            self._scan_hard_deadline = None
            if cancel_event is not None:
                cancel_event.set()
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=max(0.0, min(float(join_timeout_s), 2.0)))
        exited = worker is None or not worker.is_alive()
        if not exited:
            logger.warning(f"二维码扫码线程取消超时: {reason}")
        return exited

    def wait_scan_for_test(self, generation, timeout_s=2.0):
        """Bounded test helper; production code must poll from the flight thread."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            result = self.poll_scan_result(generation)
            if result is not None:
                return result
            time.sleep(0.001)
        return self.poll_scan_result(generation)

    @property
    def active_scan_generation(self):
        with self._scan_lock:
            return self._scan_request.generation if self._scan_request is not None else None

    @property
    def scan_worker_active(self):
        with self._scan_lock:
            return self._scan_thread is not None and self._scan_thread.is_alive()

    def abort_scan_for_heading_recovery(self):
        """Invalidate scan side effects immediately without blocking flight control."""
        self.cancel_scan("heading_recovery", join_timeout_s=0.0)
        if self.state_machine.state == InventoryState.VERIFY_QR:
            self._go(InventoryState.TRANSIT, "heading_recovery")

    def shutdown(self, join_timeout_s=2.0):
        exited = self.cancel_scan("shutdown", join_timeout_s=join_timeout_s)
        if self.camera is not None:
            self.camera.close()
        return exited

    def _go(self, state, reason, **fields):
        state = InventoryState(state)
        if self.state_machine.state != state:
            self.state_machine.transition(state, reason, **fields)

    def on_takeoff_complete(self):
        self._go(InventoryState.TRANSIT, "takeoff_complete")

    def on_waypoint_arrived(self, index, position, reason):
        waypoint = self.route[index]
        self.state_machine.sample(
            waypoint_index=index,
            waypoint_kind=waypoint.kind.value,
            slot_label=waypoint.slot_label,
            arrival_reason=reason,
            position=list(position),
        )

        if waypoint.kind in {WaypointKind.TAKEOFF, WaypointKind.TRANSIT}:
            if self.state_machine.state == InventoryState.RETURN:
                if waypoint.kind == WaypointKind.TAKEOFF:
                    logger.warning("返航路线意外包含TAKEOFF航点，保持RETURN并继续")
            else:
                self._go(InventoryState.TRANSIT, "transit_arrival", waypoint_index=index)
            return WaypointArrivalAction.ADVANCE

        if waypoint.kind == WaypointKind.SET_GIMBAL:
            self._go(
                InventoryState.SET_GIMBAL,
                "set_face",
                face=waypoint.face.value if waypoint.face else None,
            )
            try:
                gimbal_ok = waypoint.face is not None and self.gimbal.set_face(waypoint.face)
            except Exception as exc:
                return self._abort(
                    "gimbal_set_exception", waypoint_index=index, error=str(exc)
                )
            if not gimbal_ok:
                return self._abort("gimbal_set_failed", waypoint_index=index)
            self._go(InventoryState.APPROACH_SLOT, "gimbal_set")
            return WaypointArrivalAction.ADVANCE

        if waypoint.kind == WaypointKind.INSPECT:
            return self._inspect_slot(index, waypoint, position)

        if waypoint.kind == WaypointKind.LAND_APPROACH:
            if self.state_machine.state == InventoryState.RETURN:
                self._go(InventoryState.LAND, "return_arrived")
                return WaypointArrivalAction.LAND
            self._go(InventoryState.RETURN, "landing_approach")
            return WaypointArrivalAction.ADVANCE

        if waypoint.kind == WaypointKind.LAND:
            self._go(InventoryState.LAND, "landing_point")
            return WaypointArrivalAction.LAND

        return self._abort("unknown_waypoint_kind", waypoint_index=index)

    def _quick_fov_geometry_check(self, index, waypoint, position):
        """Run a bounded diagnostic precheck without deciding scan eligibility."""
        started = self._clock()
        last_sequence = None
        checked = 0
        geometry_seen = False

        while checked < self.FOV_CHECK_MAX_FRAMES:
            if self._clock() - started >= self.FOV_CHECK_BUDGET_S:
                break
            try:
                if hasattr(self.camera, "read_with_sequence"):
                    sequence, frame, frame_timestamp = self.camera.read_with_sequence()
                    if sequence is not None and sequence == last_sequence:
                        self._sleep(self.config.scan_poll_s)
                        continue
                    last_sequence = sequence
                else:
                    sequence, frame_timestamp = None, None
                    frame = self.camera.read()
            except Exception as exc:
                logger.warning("快速FOV取帧异常，继续正常扫码: %s", exc)
                break

            if frame is None:
                self._sleep(self.config.scan_poll_s)
                continue

            checked += 1
            error = None
            try:
                try:
                    geometry = self.decoder.detect_geometry(
                        frame, decode_content=False
                    )
                except TypeError:
                    geometry = self.decoder.detect_geometry(frame)
                except AttributeError:
                    try:
                        geometry = self.decoder.detect(
                            frame,
                            target_point=self.camera.laser_aim_point(frame),
                        )
                    except TypeError:
                        geometry = self.decoder.detect(frame)
                geometry_seen = geometry is not None
            except Exception as exc:
                geometry = None
                error = str(exc)
                logger.warning("快速FOV几何检测异常，继续正常扫码: %s", exc)

            if self.vision_debug is not None:
                try:
                    self.vision_debug.capture_scan(
                        frame,
                        {
                            "state": InventoryState.VISUAL_ALIGN.value,
                            "capture": "fov_precheck",
                            "waypoint_index": index,
                            "slot_label": waypoint.slot_label,
                            "position": list(position),
                            "frame_index": checked,
                            "frame_sequence": sequence,
                            "capture_timestamp": frame_timestamp,
                            "geometry_seen": geometry_seen,
                            "error": error,
                            "timestamp": time.time(),
                        },
                    )
                except Exception as exc:
                    logger.warning("快速FOV诊断图保存失败，继续正常扫码: %s", exc)

            if geometry_seen:
                break
            if error is not None:
                break
            self._sleep(self.config.scan_poll_s)

        elapsed = self._clock() - started
        if geometry_seen:
            logger.info(
                "快速FOV检测到QR几何: slot=%s, frames=%d, elapsed=%.3fs",
                waypoint.slot_label,
                checked,
                elapsed,
            )
        else:
            logger.warning(
                "快速FOV未检测到QR几何，继续完整扫码: slot=%s, "
                "frames=%d, elapsed=%.3fs",
                waypoint.slot_label,
                checked,
                elapsed,
            )
        return geometry_seen

    def _inspect_slot(self, index, waypoint: MissionWaypoint, position):
        self._go(
            InventoryState.APPROACH_SLOT,
            "inspect_arrival",
            slot_label=waypoint.slot_label,
        )
        self._go(InventoryState.VISUAL_ALIGN, "hold_position_for_scan")
        if self.config.vision_servo.enabled:
            return self._abort(
                "vision_servo_not_supported_with_async_scan",
                waypoint_index=index,
                slot_label=waypoint.slot_label,
            )

        # 同步 OpenCV 几何调用无法被 Python 的软预算中断，实飞曾卡在
        # VISUAL_ALIGN，因此默认关闭，只允许台架诊断显式开启。
        self._hold_position(waypoint.point.z)
        if self.config.fov_precheck_enabled:
            self._quick_fov_geometry_check(index, waypoint, position)
        else:
            logger.info(
                "快速FOV预检已关闭，直接进入完整扫码: slot=%s",
                waypoint.slot_label,
            )

        self._go(InventoryState.VERIFY_QR, "visual_alignment_ready")
        self.start_scan(index, waypoint, position)
        return WaypointArrivalAction.ENTER_SCAN

    def consume_scan_result(self, result, current_position):
        if result is None:
            return ScanConsumeResult(ScanConsumeOutcome.RUNNING)
        with self._scan_lock:
            request = self._scan_request
            if request is None or result.generation != request.generation:
                return ScanConsumeResult(ScanConsumeOutcome.IGNORED)
            if result.generation != self._scan_generation:
                return ScanConsumeResult(ScanConsumeOutcome.IGNORED)
            if (
                self._scan_result is not None
                and self._scan_result.status == ScanTaskStatus.CONSUMED
            ):
                return ScanConsumeResult(ScanConsumeOutcome.IGNORED)
            self._scan_result = ScanResult(
                result.generation,
                result.waypoint_index,
                result.slot_label,
                ScanTaskStatus.CONSUMED,
                detection=result.detection,
                error_code=result.error_code,
                error_detail=result.error_detail,
                processed_frames=result.processed_frames,
                started_monotonic=result.started_monotonic,
                finished_monotonic=result.finished_monotonic,
            )

        accepted = result.detection
        if accepted is None or result.status == ScanTaskStatus.FAILED:
            # 扫码失败不中断飞行，跳过当前货位继续下一格。
            # 整条路线（如A面6点）走完后自然进入降落。
            logger.info(
                "跳过货位 %s（原因: %s），继续下一格",
                result.slot_label,
                result.error_code or "scan_no_detection",
            )
            # 把状态机从 VERIFY_QR 推走，否则下一格到达时 _inspect_slot
            # 试图从 VERIFY_QR → APPROACH_SLOT，会被状态机拦截报错。
            self._go(InventoryState.TRANSIT, "next_slot")
            return ScanConsumeResult(ScanConsumeOutcome.ADVANCE, error_code=result.error_code or "scan_no_detection")
        try:
            self.store.check_available(accepted.number, result.slot_label)
        except InventoryConflict as exc:
            # A readable but conflicting QR is a slot-level inventory miss, not
            # a flight-safety fault.  Record it and continue so one duplicate
            # cannot truncate the remaining inspection route.
            logger.warning(
                "跳过货位 %s（原因: qr_duplicate, %s），继续下一格",
                result.slot_label,
                exc,
            )
            self._go(InventoryState.TRANSIT, "next_slot")
            return ScanConsumeResult(
                ScanConsumeOutcome.ADVANCE,
                error_code="qr_duplicate",
            )

        self._go(InventoryState.ILLUMINATE, "qr_verified", cargo_id=accepted.number)
        try:
            if not self.laser.pulse_async():
                return ScanConsumeResult(ScanConsumeOutcome.RETURN, error_code="laser_pulse_failed")
            if not self.laser.wait(timeout=self.laser.config.duration_s + 0.5):
                return ScanConsumeResult(ScanConsumeOutcome.RETURN, error_code="laser_pulse_timeout")
        except Exception:
            return ScanConsumeResult(ScanConsumeOutcome.RETURN, error_code="laser_pulse_exception")

        stored = self.store.add(
            accepted.number,
            result.slot_label,
            self.consensus.config.required_count / self.consensus.config.window_size,
        )
        self._go(
            InventoryState.REPORT,
            "laser_complete",
            cargo_id=stored.cargo_id,
            slot_label=stored.slot_label,
        )
        self._publish_result(stored)
        next_kind = (
            self.route[result.waypoint_index + 1].kind
            if result.waypoint_index + 1 < len(self.route)
            else None
        )
        if next_kind == WaypointKind.SET_GIMBAL:
            self._go(InventoryState.SET_GIMBAL, "next_face")
        elif next_kind in {WaypointKind.LAND_APPROACH, WaypointKind.LAND}:
            self._go(InventoryState.RETURN, "inventory_complete")
        else:
            self._go(InventoryState.TRANSIT, "next_slot")
        return ScanConsumeResult(ScanConsumeOutcome.ADVANCE)

    def _run_visual_servo(self, index, waypoint, position):
        """Center QR geometry with bounded X velocity and Z setpoint changes."""
        config = self.config.vision_servo
        if not config.enabled:
            # 未启用时直接跳过，让调用方继续走 VERIFY_QR
            return VisionServoResult(True, float(waypoint.point.z), 0, 0, reason="disabled")
        started = self._clock()
        deadline = started + config.timeout_s
        last_seen = started
        stable = 0
        frames = 0
        base_z_m = float(waypoint.point.z)
        z_target_m = base_z_m
        last_error = (None, None)
        last_frame = None
        last_center = None
        face_sign = config.x_direction(waypoint.face)
        # 用伺服启动时的实时位置作为横向基准，而非 APPROACH 传入的历史位置
        _servo_start_pos = self._current_position()
        start_x = float(_servo_start_pos[0]) if _servo_start_pos is not None else (
            float(position[0]) if position else None
        )
        last_debug_capture_t = started
        _debug_interval = 0.5  # 周期性存图间隔（秒）

        while self._clock() < deadline:
            frames += 1
            try:
                frame = self.camera.read()
            except Exception:
                self._send_servo_command(0.0, z_target_m)
                return VisionServoResult(
                    False, z_target_m, frames, stable, reason="camera_read"
                )
            if frame is None:
                if frames >= config.min_lost_frames and self._clock() - last_seen > config.lost_timeout_s:
                    self._capture_servo_failure(last_frame, index, waypoint, "target_lost")
                    return VisionServoResult(
                        False, z_target_m, frames, stable, reason="target_lost"
                    )
                self._send_servo_command(0.0, z_target_m)
                self._sleep(self.config.scan_poll_s)
                continue

            try:
                try:
                    geometry = self.decoder.detect_geometry(frame, decode_content=False)
                except TypeError:
                    # Keep test doubles and older decoders that only accept
                    # the original one-argument API usable.
                    geometry = self.decoder.detect_geometry(frame)
                except AttributeError:
                    # Keep test doubles and older injected decoders usable;
                    # the real QRDecoder always provides detect_geometry().
                    geometry = self.decoder.detect(frame)
                aim = self.camera.laser_aim_point(frame)
            except Exception:
                self._send_servo_command(0.0, z_target_m)
                return VisionServoResult(
                    False, z_target_m, frames, stable, reason="vision_exception"
                )
            last_frame = frame
            if frames == 1 and self.vision_debug is not None:
                self.vision_debug.capture_scan(
                    frame,
                    {
                        "state": InventoryState.VISUAL_SERVO.value,
                        "capture": "servo_first",
                        "waypoint_index": index,
                        "slot_label": waypoint.slot_label,
                        "position": list(position),
                        "timestamp": time.time(),
                    },
                )
            if geometry is None:
                if frames >= config.min_lost_frames and self._clock() - last_seen > config.lost_timeout_s:
                    self._capture_servo_failure(last_frame, index, waypoint, "target_lost")
                    return VisionServoResult(
                        False, z_target_m, frames, stable, reason="target_lost"
                    )
                stable = 0
                self._send_servo_command(0.0, z_target_m)
            else:
                last_seen = self._clock()
                center = geometry.center
                error_x = center[0] - aim[0]
                error_y = center[1] - aim[1]
                if last_center is not None:
                    center_jump = math.hypot(
                        center[0] - last_center[0], center[1] - last_center[1]
                    )
                    if center_jump > config.max_center_jump_px:
                        # A second QR code or a rail artefact appeared. Hold
                        # position and wait for the previously tracked target;
                        # never steer toward an unbounded candidate jump.
                        stable = 0
                        self._send_servo_command(0.0, z_target_m)
                        self._sleep(self.config.scan_poll_s)
                        continue
                last_center = center
                last_error = (error_x, error_y)
                _cur_pos = self._current_position()

                centered = (
                    abs(error_x) <= config.center_tolerance_px
                    and abs(error_y) <= config.center_tolerance_px
                )
                if centered:
                    stable += 1
                else:
                    stable = 0
                x_cmd, z_target_m = servo_command(
                    config,
                    error_x,
                    error_y,
                    base_z_m,
                    z_target_m,
                    face_sign,
                )
                # 复用已读取的 _cur_pos，避免再次串口读取（Fix 中风险1）
                if start_x is not None and _cur_pos is not None:
                    if abs(_cur_pos[0] - start_x) > config.max_lateral_adjust_m:
                        return VisionServoResult(
                            False, z_target_m, frames, stable,
                            error_x, error_y, "lateral_limit"
                        )
                self._send_servo_command(0.0 if centered else x_cmd, z_target_m)

                # 每隔 _debug_interval 秒存一张伺服过程帧，移到指令下发后避免 I/O 阻塞（Fix 中风险2）
                _now_t = self._clock()
                if self.vision_debug is not None and (_now_t - last_debug_capture_t) >= _debug_interval:
                    last_debug_capture_t = _now_t
                    self.vision_debug.capture_scan(
                        frame,
                        {
                            "state": InventoryState.VISUAL_SERVO.value,
                            "capture": "servo_periodic",
                            "waypoint_index": index,
                            "slot_label": waypoint.slot_label,
                            "position": list(_cur_pos) if _cur_pos is not None else None,
                            "error_px": [round(error_x, 1), round(error_y, 1)],
                            "geometry_number": geometry.number,
                            "stable": stable,
                            "frame": frames,
                            "timestamp": time.time(),
                        },
                    )
                self.state_machine.sample(
                    waypoint_index=index,
                    slot_label=waypoint.slot_label,
                    position=list(_cur_pos) if _cur_pos is not None else None,
                    visual_servo_error_px=[round(error_x, 1), round(error_y, 1)],
                    visual_servo_centered=centered,
                    visual_servo_target_number=geometry.number,
                )
                if stable >= config.stable_frames:
                    if self.vision_debug is not None and last_frame is not None:
                        self.vision_debug.capture_scan(
                            last_frame,
                            {
                                "state": InventoryState.VISUAL_SERVO.value,
                                "capture": "servo_centered",
                                "waypoint_index": index,
                                "slot_label": waypoint.slot_label,
                                "error_px": [error_x, error_y],
                                "timestamp": time.time(),
                            },
                        )
                    self._send_servo_command(0.0, z_target_m)
                    return VisionServoResult(
                        True, z_target_m, frames, stable, error_x, error_y, "centered"
                    )
            self._sleep(self.config.scan_poll_s)

        return VisionServoResult(
            False, z_target_m, frames, stable,
            last_error[0], last_error[1], "timeout"
        )

    def _capture_servo_failure(self, frame, index, waypoint, reason):
        if self.vision_debug is None or frame is None:
            return
        self.vision_debug.capture_scan(
            frame,
            {
                "state": InventoryState.VISUAL_SERVO.value,
                "capture": "servo_" + reason,
                "waypoint_index": index,
                "slot_label": waypoint.slot_label,
                "timestamp": time.time(),
            },
        )

    def _current_position(self):
        realsense = getattr(self.driver, "realsense", None)
        if realsense is None:
            return None
        try:
            return tuple(float(value) for value in realsense.get_position())
        except Exception:
            return None

    def _send_servo_command(self, x_cmd, z_target_m):
        if self.driver is None:
            return
        yaw_cmd = getattr(getattr(self.driver, "_heading_status", None), "command_dps", 0)
        self.driver.set_speed(
            int(round(x_cmd)),
            0,
            int(round(yaw_cmd)),
            int(round(z_target_m * 100)),
        )

    def _hold_position(self, z_m):
        if self.driver is not None:
            self.driver.hold_position(z_m)

    def _publish_result(self, result):
        if self.ground_link is None:
            return
        try:
            self.ground_link.publish(
                GroundMessageType.INVENTORY_RESULT,
                {
                    "cargo_id": result.cargo_id,
                    "slot_label": result.slot_label,
                    "confidence": result.confidence,
                    "timestamp": result.timestamp,
                },
            )
        except Exception as exc:
            self.state_machine.trace.fault("ground_result_publish_failed", error=str(exc))

    def _abort(self, code, **fields):
        logger.error(f"盘点状态机故障: {code} {fields}")
        self.state_machine.fault(code, **fields)
        if self.driver is not None:
            self.driver.abort_to_land()
        if self.state_machine.state == InventoryState.RETURN:
            self._go(InventoryState.LAND, "fault_land", fault_code=code)
        return False


def default_inventory_config(base_dir=None):
    config = InventoryMissionConfig.from_env()
    if base_dir is None:
        return config
    mapping = Path(config.qr_mapping_file)
    if not mapping.is_absolute():
        mapping = Path(base_dir) / mapping
    return InventoryMissionConfig(
        camera_device=config.camera_device,
        camera_width=config.camera_width,
        camera_height=config.camera_height,
        camera_fps=config.camera_fps,
        camera_auto_exposure=config.camera_auto_exposure,
        camera_exposure=config.camera_exposure,
        camera_gain=config.camera_gain,
        camera_auto_focus=config.camera_auto_focus,
        camera_focus=config.camera_focus,
        camera_zoom=config.camera_zoom,
        camera_warmup_frames=config.camera_warmup_frames,
        qr_mapping_file=str(mapping),
        scan_timeout_s=config.scan_timeout_s,
        scan_poll_s=config.scan_poll_s,
        laser_aim_x_ratio=config.laser_aim_x_ratio,
        laser_aim_y_ratio=config.laser_aim_y_ratio,
        vision_servo=config.vision_servo,
        async_qr_scan=config.async_qr_scan,
        fov_precheck_enabled=config.fov_precheck_enabled,
        qr_decode_profile=config.qr_decode_profile,
    )


class InventoryFlightMission(FlightMission):
    """Flight driver subclass that invokes the inventory coordinator at stops."""

    def __init__(self, re_fc, se_fc, realsense, serial_fc, route, coordinator):
        super().__init__(
            re_fc,
            se_fc,
            realsense,
            serial_fc_ref=serial_fc,
            route_file=None,
        )
        self.inventory_route = list(route)
        self.targets = [waypoint.point.as_list() for waypoint in self.inventory_route]
        self.coordinator = coordinator
        self.coordinator.attach_driver(self)

    def start(self):
        if self.coordinator.state_machine.state == InventoryState.PREFLIGHT:
            self.coordinator._go(InventoryState.WARNING_5S, "preflight_passed")
        super().start()
        if self.task_running:
            self.coordinator._go(InventoryState.TAKEOFF, "flight_task_started")
        elif self.coordinator.state_machine.state == InventoryState.WARNING_5S:
            # The base driver can refuse to start after a T265 or warning-light
            # failure. Do not leave the inventory state machine looking ready.
            self.coordinator._abort("flight_start_failed")

    def takeoff(self):
        previous_state = self.state
        super().takeoff()
        if previous_state == "TAKEOFF" and self.state == "NAVIGATE":
            self.coordinator.on_takeoff_complete()

    def on_heading_recovery_started(self, source_state):
        if source_state != "SCAN":
            return
        self.coordinator.abort_scan_for_heading_recovery()
        self._scan_generation = None
        self.end_scan_hold()
        # 保留当前航点索引。恢复成功后重新到位，启动新generation扫码。
        self.state = "NAVIGATE"

    def heading_recovery_ready_to_resume(self):
        return not self.coordinator.scan_worker_active

    def on_heading_recovery_failed(self, reason):
        machine_state = self.coordinator.state_machine.state
        if InventoryState.LAND in ALLOWED_TRANSITIONS[machine_state]:
            self.coordinator._go(
                InventoryState.LAND,
                "heading_recovery_failed",
                error=str(reason),
            )
        elif machine_state != InventoryState.LAND:
            self.coordinator.state_machine.fault(
                "heading_recovery_failed",
                recover_to_return=False,
                error=str(reason),
            )
            if InventoryState.LAND in ALLOWED_TRANSITIONS[
                self.coordinator.state_machine.state
            ]:
                self.coordinator._go(
                    InventoryState.LAND,
                    "heading_recovery_failed_land",
                    error=str(reason),
                )

    def _advance_waypoint(self, reason, pos, target, arrival_distance):
        index = self.target_index
        if index >= len(self.inventory_route):
            return super()._advance_waypoint(reason, pos, target, arrival_distance)
        action = self.coordinator.on_waypoint_arrived(index, pos, reason)
        if action == WaypointArrivalAction.ENTER_SCAN:
            waypoint = self.inventory_route[index]
            self._scan_route_index = index
            self._scan_generation = self.coordinator.active_scan_generation
            self.begin_scan_hold(waypoint.point.as_list())
            return
        if action == WaypointArrivalAction.LAND:
            self.state = "LAND"
            return
        if action != WaypointArrivalAction.ADVANCE:
            return
        super()._advance_waypoint(reason, pos, target, arrival_distance)

    def on_flight_loop_exception(self, exc, failed_state):
        if failed_state == "SCAN":
            self.coordinator.cancel_scan("flight_loop_exception", join_timeout_s=0.0)
            self.end_scan_hold()
        super().on_flight_loop_exception(exc, failed_state)
        if failed_state == "LAND":
            return
        machine_state = self.coordinator.state_machine.state
        if machine_state in {InventoryState.LAND, InventoryState.END}:
            return
        if InventoryState.LAND in ALLOWED_TRANSITIONS[machine_state]:
            self.coordinator._go(
                InventoryState.LAND,
                "flight_loop_exception",
                error=str(exc),
                flight_state=failed_state,
            )
            return
        self.coordinator.state_machine.fault(
            "flight_loop_exception",
            recover_to_return=False,
            error=str(exc),
            flight_state=failed_state,
        )
        if InventoryState.LAND in ALLOWED_TRANSITIONS[self.coordinator.state_machine.state]:
            self.coordinator._go(
                InventoryState.LAND,
                "flight_loop_exception_land",
                flight_state=failed_state,
            )

    def on_scan_tick(self, pos, yaw, control):
        result = self.coordinator.poll_scan_result(self._scan_generation)
        if result is None:
            return
        consumed = self.coordinator.consume_scan_result(result, pos)
        if consumed.outcome == ScanConsumeOutcome.ADVANCE:
            self.end_scan_hold()
            self.state = "NAVIGATE"
            super()._advance_waypoint(
                "scan_complete",
                pos,
                self.targets[self.target_index],
                0.0,
            )
        elif consumed.outcome == ScanConsumeOutcome.RETURN:
            self.end_scan_hold()
            if consumed.return_route:
                self.replace_inventory_navigation_route(consumed.return_route, pos)
            else:
                # Duplicate/laser failures currently request RETURN without a
                # planned route. Never pass that empty tuple into the flight
                # navigator: stop XY/yaw and use the existing safe LAND path.
                logger.error(
                    "扫码请求返航但路线为空，直接安全降落: %s",
                    consumed.error_code or "scan_return_route_empty",
                )
                self.coordinator._abort(
                    "scan_return_route_empty",
                    error_code=consumed.error_code,
                )
        elif consumed.outcome == ScanConsumeOutcome.EMERGENCY_LAND:
            self.end_scan_hold()
            self.state = "LAND"

    def on_scan_tracking_lost(self, pos, yaw):
        self.coordinator.cancel_scan("tracking_lost", join_timeout_s=0.0)
        self.end_scan_hold()
        self.state = "LAND"

    def replace_inventory_navigation_route(self, route, current_pos):
        self.inventory_route = list(route)
        self.coordinator.route = list(route)
        generation = self.replace_navigation_targets(
            [waypoint.point.as_list() for waypoint in self.inventory_route],
            current_pos,
            purpose="return",
        )
        self.state = "NAVIGATE"
        return generation

    def hold_position(self, z_m):
        yaw_cmd = self._heading_status.command_dps
        self.set_speed(0, 0, yaw_cmd, int(float(z_m) * 100))

    def abort_to_land(self):
        self.set_speed(0, 0, 0, int(self._ramp_z_cm))
        self.state = "LAND"

    def land(self):
        previous_state = self.state
        super().land()
        if (
            previous_state == "LAND"
            and self.state == "END"
            and self.coordinator.state_machine.state == InventoryState.LAND
        ):
            self.coordinator._go(InventoryState.END, "land_confirmed")
