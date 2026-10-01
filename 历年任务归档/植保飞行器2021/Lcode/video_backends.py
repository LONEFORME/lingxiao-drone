"""Optional OpenCV capture and UDP-JPEG airborne publishing backends."""

from __future__ import annotations

import math
from pathlib import Path
import socket
import struct
import threading
import time
from typing import Optional
import zlib

from Lcode.video_source import (
    SnapshotResult,
    VideoFrame,
    VideoPublisher,
    VideoPublisherConfig,
    VideoSource,
    VideoSourceConfig,
    VideoSourceError,
    register_video_publisher_backend,
    register_video_source_backend,
)


UDP_JPEG_MAGIC = b"DJPG"
UDP_JPEG_VERSION = 1
UDP_JPEG_HEADER = struct.Struct("!4sBIHHHI")


def encode_udp_jpeg_packets(
    jpeg: bytes, frame_id: int, max_datagram: int = 1200
) -> tuple[bytes, ...]:
    if not jpeg:
        raise VideoSourceError("JPEG payload cannot be empty")
    if not UDP_JPEG_HEADER.size + 64 <= max_datagram <= 65507:
        raise VideoSourceError("UDP JPEG datagram size is invalid")
    chunk_size = max_datagram - UDP_JPEG_HEADER.size
    chunk_count = math.ceil(len(jpeg) / chunk_size)
    if chunk_count > 65535:
        raise VideoSourceError("JPEG frame requires too many UDP chunks")
    checksum = zlib.crc32(jpeg) & 0xFFFFFFFF
    packets = []
    for index in range(chunk_count):
        payload = jpeg[index * chunk_size : (index + 1) * chunk_size]
        header = UDP_JPEG_HEADER.pack(
            UDP_JPEG_MAGIC,
            UDP_JPEG_VERSION,
            int(frame_id) & 0xFFFFFFFF,
            index,
            chunk_count,
            len(payload),
            checksum,
        )
        packets.append(header + payload)
    return tuple(packets)


def decode_udp_jpeg_packet(packet: bytes) -> tuple[dict[str, int], bytes]:
    if len(packet) < UDP_JPEG_HEADER.size:
        raise VideoSourceError("UDP JPEG packet is truncated")
    magic, version, frame_id, index, count, size, checksum = UDP_JPEG_HEADER.unpack(
        packet[: UDP_JPEG_HEADER.size]
    )
    payload = packet[UDP_JPEG_HEADER.size :]
    if (
        magic != UDP_JPEG_MAGIC
        or version != UDP_JPEG_VERSION
        or count == 0
        or index >= count
        or size != len(payload)
    ):
        raise VideoSourceError("UDP JPEG packet header is invalid")
    return {
        "frame_id": frame_id,
        "chunk_index": index,
        "chunk_count": count,
        "frame_crc32": checksum,
    }, payload


class OpenCvVideoSource(VideoSource):
    def __init__(self, config: VideoSourceConfig):
        self.config = config
        self._cv2 = None
        self._capture = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._condition = threading.Condition()
        self._latest: Optional[VideoFrame] = None
        self._sequence = 0
        self._delivered_sequence = -1

    def start(self) -> bool:
        try:
            import cv2
        except ImportError as exc:
            raise VideoSourceError("OpenCV is not installed") from exc
        source = self.config.source
        if source.isdigit():
            source = int(source)
        capture = cv2.VideoCapture(source)
        options = self.config.options
        if options.get("width"):
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(options["width"]))
        if options.get("height"):
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(options["height"]))
        if options.get("fps"):
            capture.set(cv2.CAP_PROP_FPS, float(options["fps"]))
        if not capture.isOpened():
            capture.release()
            return False
        self._cv2 = cv2
        self._capture = capture
        self._thread = threading.Thread(
            target=self._capture_loop, name="opencv-capture", daemon=True
        )
        self._thread.start()
        return True

    def read_frame(self, timeout_s: float = 0.5) -> Optional[VideoFrame]:
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._condition:
            while (
                self._latest is None
                or self._latest.sequence == self._delivered_sequence
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop_event.is_set():
                    return None
                self._condition.wait(remaining)
            self._delivered_sequence = self._latest.sequence
            return self._latest

    def snapshot(
        self, point_id: str, output_dir: str | Path, timeout_s: float = 1.0
    ) -> SnapshotResult:
        frame = self.read_frame(timeout_s)
        if frame is None or self._cv2 is None:
            return SnapshotResult(point_id, None, None, "frame_timeout")
        output = Path(output_dir).resolve()
        output.mkdir(parents=True, exist_ok=True)
        final = output / f"{point_id}.jpg"
        temporary = output / f".{point_id}.{time.time_ns()}.tmp.jpg"
        try:
            quality = int(self.config.options.get("jpeg_quality", 55))
            ok = self._cv2.imwrite(
                str(temporary),
                frame.payload,
                [int(self._cv2.IMWRITE_JPEG_QUALITY), quality],
            )
            if not ok:
                return SnapshotResult(point_id, None, None, "jpeg_encode_failed")
            temporary.replace(final)
            return SnapshotResult(point_id, final, frame.captured_at)
        except (OSError, ValueError) as exc:
            return SnapshotResult(point_id, None, None, f"snapshot_write_failed:{exc}")
        finally:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass

    def is_running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def stop(self) -> None:
        self._stop_event.set()
        capture = self._capture
        if capture is not None:
            capture.release()
        with self._condition:
            self._condition.notify_all()
        thread = self._thread
        if thread is not None:
            thread.join(0.5)

    def _capture_loop(self) -> None:
        capture = self._capture
        if capture is None:
            return
        while not self._stop_event.is_set():
            ok, image = capture.read()
            if not ok:
                self._stop_event.wait(0.01)
                continue
            height, width = image.shape[:2]
            frame = VideoFrame(
                sequence=self._sequence,
                captured_at=time.time(),
                width=int(width),
                height=int(height),
                pixel_format="bgr8",
                payload=image,
            )
            self._sequence += 1
            with self._condition:
                self._latest = frame
                self._condition.notify_all()


class UdpJpegPublisher(VideoPublisher):
    def __init__(self, config: VideoPublisherConfig):
        self.config = config
        self._socket: Optional[socket.socket] = None
        self._target = _parse_target(config.target)
        self._frame_id = 0
        self.last_error: Optional[str] = None

    def start(self) -> bool:
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            return True
        except OSError as exc:
            self.last_error = str(exc)
            return False

    def publish_frame(self, frame: VideoFrame) -> bool:
        endpoint = self._socket
        if endpoint is None:
            return False
        try:
            jpeg = self._to_jpeg(frame)
            max_datagram = int(self.config.options.get("max_datagram", 1200))
            packets = encode_udp_jpeg_packets(jpeg, self._frame_id, max_datagram)
            self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
            for packet in packets:
                endpoint.sendto(packet, self._target)
            return True
        except (OSError, ValueError, VideoSourceError) as exc:
            self.last_error = str(exc)
            return False

    def is_running(self) -> bool:
        return self._socket is not None

    def stop(self) -> None:
        endpoint = self._socket
        self._socket = None
        if endpoint is not None:
            endpoint.close()

    def _to_jpeg(self, frame: VideoFrame) -> bytes:
        if isinstance(frame.payload, (bytes, bytearray, memoryview)):
            return bytes(frame.payload)
        try:
            import cv2
        except ImportError as exc:
            raise VideoSourceError("OpenCV required to encode non-JPEG frame") from exc
        quality = int(self.config.options.get("jpeg_quality", 55))
        ok, encoded = cv2.imencode(
            ".jpg",
            frame.payload,
            [int(cv2.IMWRITE_JPEG_QUALITY), quality],
        )
        if not ok:
            raise VideoSourceError("JPEG encode failed")
        return encoded.tobytes()


def register_builtin_video_backends() -> None:
    register_video_source_backend("opencv_capture", OpenCvVideoSource)
    register_video_source_backend("capture_device", OpenCvVideoSource)
    register_video_source_backend("network_stream", OpenCvVideoSource)
    register_video_publisher_backend("udp_jpeg", UdpJpegPublisher)


def _parse_target(target: str) -> tuple[str, int]:
    host, separator, port_text = target.rpartition(":")
    if not separator or not host:
        raise VideoSourceError("UDP JPEG target must be host:port")
    try:
        port = int(port_text)
    except ValueError as exc:
        raise VideoSourceError("UDP JPEG target port is invalid") from exc
    if not 1 <= port <= 65535:
        raise VideoSourceError("UDP JPEG target port is invalid")
    return host, port
