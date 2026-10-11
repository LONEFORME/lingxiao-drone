"""Cyber Camera D题独立调试入口。

默认把VS1输出到stdout；上板时用--serial指定连接Pi的UART。预览仅用于调试，
协议发送不依赖显示窗口。
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    from .camera_backend import create_capture
    from .detector import (
        AprilTagBlueFusionDetector, AprilTagDetector, BlueSquareDetector,
        FeatureFlag, PlatformDetector, RingCrossDetector,
    )
    from .display_backend import create_display
    from .protocol import BAUDRATE, encode, encode_control, parse_control
except ImportError:  # 允许直接python main.py运行
    from camera_backend import create_capture
    from detector import (
        AprilTagBlueFusionDetector, AprilTagDetector, BlueSquareDetector,
        FeatureFlag, PlatformDetector, RingCrossDetector,
    )
    from display_backend import create_display
    from protocol import BAUDRATE, encode, encode_control, parse_control


def _stream_id() -> int:
    value = (time.monotonic_ns() ^ id(object())) & 0xFFFF
    return value or 1


def _create_detector(
    target: str,
    tag_family: str = "tag36h11",
    tag_id: int = 0,
    tag_min_side_px: float = 18.0,
    tag_detect_scale: float = 0.75,
    tag_redetect_interval: int = 5,
    tag_max_flow_age_s: float = 0.5,
):
    if target == "formal":
        return PlatformDetector()
    if target == "ring_cross":
        return RingCrossDetector()
    if target == "blue_square":
        return BlueSquareDetector()
    if target == "apriltag":
        return AprilTagDetector(
            tag_family,
            tag_id,
            tag_min_side_px,
            tag_detect_scale,
            tag_redetect_interval,
            tag_max_flow_age_s,
        )
    if target == "apriltag_blue_fusion":
        return AprilTagBlueFusionDetector(
            tag_family,
            tag_id,
            tag_min_side_px,
            tag_detect_scale,
            tag_redetect_interval,
            tag_max_flow_age_s,
        )
    raise ValueError(f"未知目标类型: {target}")


def _annotate_frame(frame, result, target: str, processing_fps: float):
    annotated = frame.copy()
    h, w = annotated.shape[:2]
    frame_center = (w // 2, h // 2)
    cv2.drawMarker(
        annotated, frame_center, (255, 255, 255),
        cv2.MARKER_CROSS, 24, 2,
    )
    if result and result.found:
        flags = FeatureFlag(result.flags)
        if flags & FeatureFlag.COLOR_SHAPE_TRACKED:
            mode = "COLOR"
        elif flags & FeatureFlag.TEMPORAL_TRACKED:
            mode = "FLOW"
        else:
            mode = "PARTIAL" if flags & FeatureFlag.PARTIAL else "FULL"
        if result.debug_polygon:
            polygon = cv2.convexHull(
                np.asarray(result.debug_polygon, dtype=np.int32)
            )
            cv2.polylines(annotated, [polygon], True, (0, 255, 0), 2)
        cv2.circle(annotated, (result.cx, result.cy), 6, (0, 0, 255), 2)
        cv2.line(annotated, frame_center, (result.cx, result.cy), (0, 255, 255), 1)
        cv2.putText(
            annotated,
            f"{target} {mode} Q={result.quality} E=({result.cx-frame_center[0]},"
            f"{result.cy-frame_center[1]})",
            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2,
        )
    else:
        cv2.putText(
            annotated, "NO TARGET", (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 180, 255), 2,
        )
    cv2.putText(
        annotated, f"PROC {processing_fps:.1f} FPS",
        (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
    )
    return annotated


class DebugFrameRecorder:
    def __init__(self, enabled: bool, root_dir: str, interval_s: float = 1.0):
        if interval_s <= 0:
            raise ValueError("debug record interval must be greater than zero")
        self.enabled = bool(enabled)
        self.interval_s = float(interval_s)
        self.next_record_time = 0.0
        self.session_dir = None
        if self.enabled:
            session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.session_dir = Path(root_dir) / session_name
            self.session_dir.mkdir(parents=True, exist_ok=True)
            print(
                f"[DEBUG_RECORD] ON interval={self.interval_s:.2f}s "
                f"dir={self.session_dir}",
                file=sys.stderr,
                flush=True,
            )

    def maybe_record(
        self, frame, result, target: str, processing_fps: float,
        seq: int, capture_ms: int, now: float | None = None,
    ):
        if not self.enabled:
            return None
        current = time.monotonic() if now is None else float(now)
        if current < self.next_record_time:
            return None
        self.next_record_time = current + self.interval_s
        annotated = _annotate_frame(frame, result, target, processing_fps)
        output_path = self.session_dir / f"frame_{seq:08d}_{capture_ms:010d}.jpg"
        try:
            if not cv2.imwrite(str(output_path), annotated):
                raise RuntimeError("cv2.imwrite returned false")
        except Exception as exc:
            print(
                f"[DEBUG_RECORD] write failed: {output_path}: {exc}",
                file=sys.stderr,
                flush=True,
            )
            return None
        return output_path


class DuplexControlResponder:
    """Non-blocking VC1 responder; one complete control line per image loop."""

    def __init__(self, max_buffer_bytes: int = 512, read_bytes: int = 128):
        self.max_buffer_bytes = int(max_buffer_bytes)
        self.read_bytes = int(read_bytes)
        self.buffer = bytearray()
        self.parse_errors = 0
        self.write_errors = 0
        self.pongs_sent = 0

    def _limit_buffer(self) -> None:
        while len(self.buffer) > self.max_buffer_bytes:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                self.buffer.clear()
                self.parse_errors += 1
                return
            del self.buffer[:newline + 1]
            self.parse_errors += 1

    def service(self, serial_port) -> bool:
        try:
            data = serial_port.read(self.read_bytes)
        except Exception:
            self.parse_errors += 1
            return False
        if data:
            self.buffer.extend(data)
            self._limit_buffer()
        newline = self.buffer.find(b"\n")
        if newline < 0:
            return False
        line = bytes(self.buffer[:newline])
        del self.buffer[:newline + 1]
        try:
            command, seq = parse_control(line)
        except ValueError:
            self.parse_errors += 1
            return False
        if command != "PING":
            self.parse_errors += 1
            return False
        try:
            serial_port.write(encode_control("PONG", seq))
        except Exception:
            self.write_errors += 1
            return False
        self.pongs_sent += 1
        return True


def run(
    camera=0,
    serial_port: str | None = None,
    preview: bool = False,
    backend: str = "auto",
    width: int = 640,
    height: int = 480,
    hmirror: bool = False,
    vflip: bool = False,
    display_mode: str = "off",
    display_fps: float = 10.0,
    display_rotation: int = 0,
    target: str = "formal",
    debug_record: str = "off",
    debug_record_dir: str = "debug_frames",
    debug_record_interval: float = 1.0,
    tag_family: str = "tag36h11",
    tag_id: int = 0,
    tag_min_side_px: float = 18.0,
    tag_detect_scale: float = 0.75,
    tag_redetect_interval: int = 5,
    tag_max_flow_age_s: float = 0.5,
) -> int:
    if debug_record_interval <= 0:
        raise ValueError("debug record interval must be greater than zero")
    capture = create_capture(
        backend, camera, width, height, hmirror=hmirror, vflip=vflip
    )
    if not capture.isOpened():
        raise RuntimeError(f"无法打开摄像头: backend={backend}, source={camera}")
    display = create_display(display_mode, display_rotation)
    output = None
    if serial_port:
        import serial
        output = serial.Serial(serial_port, BAUDRATE, timeout=0, write_timeout=0.03)
    detector = _create_detector(
        target, tag_family, tag_id, tag_min_side_px, tag_detect_scale,
        tag_redetect_interval, tag_max_flow_age_s,
    )
    recorder = DebugFrameRecorder(
        debug_record == "on", debug_record_dir, debug_record_interval
    )
    stream_id = _stream_id()
    seq = 0
    next_display = 0.0
    fps_started = time.monotonic()
    fps_frames = 0
    processing_fps = 0.0
    control = DuplexControlResponder() if output is not None else None
    try:
        while True:
            ok, frame = capture.read()
            capture_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
            packet_seq = seq
            if not ok:
                result = None
                packet = encode(stream_id, seq, capture_ms, False, 0, 0, 0, 0, 0, 0, 0)
            else:
                result = detector.detect(frame)
                packet = encode(
                    stream_id, seq, capture_ms, result.found, result.cx, result.cy,
                    result.outer_px, result.inner_px, result.angle_cdeg,
                    result.quality, result.flags,
                )
            if output is None:
                sys.stdout.buffer.write(packet)
                sys.stdout.buffer.flush()
            else:
                output.write(packet)
                control.service(output)
            seq = (seq + 1) & 0xFFFFFFFF
            fps_frames += 1
            fps_now = time.monotonic()
            fps_elapsed = fps_now - fps_started
            if fps_elapsed >= 0.75:
                processing_fps = fps_frames / fps_elapsed
                fps_started = fps_now
                fps_frames = 0
            if ok:
                recorder.maybe_record(
                    frame, result, target, processing_fps, packet_seq, capture_ms
                )
            if display.enabled and ok and time.monotonic() >= next_display:
                annotated = _annotate_frame(frame, result, target, processing_fps)
                if not display.show(annotated):
                    break
                next_display = time.monotonic() + 1.0 / max(1.0, display_fps)
    finally:
        capture.release()
        display.close()
        if output is not None:
            output.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="0", help="摄像头索引或视频路径")
    parser.add_argument("--backend", choices=("auto", "csi", "opencv"), default="csi")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--hmirror", action="store_true")
    parser.add_argument("--vflip", action="store_true")
    parser.add_argument("--serial", default=None, help="连接Pi的UART设备")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--display", choices=("off", "builtin", "opencv"), default="off")
    parser.add_argument("--display-fps", type=float, default=10.0)
    parser.add_argument("--display-rotation", type=int, choices=(0, 90, 180, 270), default=0)
    parser.add_argument(
        "--target",
        choices=(
            "formal", "ring_cross", "blue_square", "apriltag",
            "apriltag_blue_fusion",
        ),
        default="formal",
    )
    parser.add_argument("--tag-family", default="tag36h11")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-min-side-px", type=float, default=18.0)
    parser.add_argument("--tag-detect-scale", type=float, default=0.75)
    parser.add_argument("--tag-redetect-interval", type=int, default=5)
    parser.add_argument("--tag-max-flow-age", type=float, default=0.5)
    parser.add_argument("--debug-record", choices=("on", "off"), default="off")
    parser.add_argument("--debug-record-dir", default="debug_frames")
    parser.add_argument("--debug-record-interval", type=float, default=1.0)
    args = parser.parse_args()
    camera = int(args.camera) if str(args.camera).isdigit() else args.camera
    display_mode = "opencv" if args.preview and args.display == "off" else args.display
    return run(
        camera, args.serial, args.preview, args.backend,
        args.width, args.height, args.hmirror, args.vflip,
        display_mode, args.display_fps, args.display_rotation, args.target,
        args.debug_record, args.debug_record_dir, args.debug_record_interval,
        args.tag_family, args.tag_id, args.tag_min_side_px,
        args.tag_detect_scale, args.tag_redetect_interval,
        args.tag_max_flow_age,
    )


if __name__ == "__main__":
    raise SystemExit(main())
