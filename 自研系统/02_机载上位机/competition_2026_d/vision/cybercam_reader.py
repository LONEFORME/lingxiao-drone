"""Threaded VS1 receiver and VC1 heartbeat owner for the Cyber Camera UART."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque

from .cybercam_protocol import ObservationGate, encode_control, parse_control, parse_line
from .link_status import LinkStatusPublisher
from .platform_observation import PlatformObservation


LOG = logging.getLogger(__name__)


class CyberCamReader:
    def __init__(
        self,
        port: str = "/dev/ttyS7",
        baudrate: int = 115200,
        timeout_s: float = 0.02,
        max_line_bytes: int = 256,
        serial_factory=None,
        status_publisher=None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.max_line_bytes = max_line_bytes
        self.serial_factory = serial_factory
        self._serial = None
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._gate = ObservationGate()
        self._latest: PlatformObservation | None = None
        self._lock = threading.Lock()
        self.parse_errors = 0
        self.control_errors = 0
        self.accepted_frames = 0
        self.last_received_monotonic: float | None = None
        self.last_pong_monotonic: float | None = None
        self.pings_sent = 0
        self.pongs_received = 0
        self.ping_write_errors = 0
        self._ping_seq = 0
        self._next_ping_monotonic = 0.0
        self._pending_pings: deque[tuple[int, float]] = deque(maxlen=4)
        self._status = status_publisher or LinkStatusPublisher()

    def is_running(self) -> bool:
        return self._running.is_set() and self._thread is not None and self._thread.is_alive()

    def stats(self) -> dict:
        with self._lock:
            latest = self._latest
            values = {
                "accepted_frames": self.accepted_frames,
                "last_received_monotonic": self.last_received_monotonic,
                "last_pong_monotonic": self.last_pong_monotonic,
                "pongs_received": self.pongs_received,
            }
        return {
            "running": self.is_running(),
            **values,
            "parse_errors": self.parse_errors,
            "control_errors": self.control_errors,
            "pings_sent": self.pings_sent,
            "ping_write_errors": self.ping_write_errors,
            "latest_stream_id": latest.stream_id if latest is not None else None,
            "latest_seq": latest.seq if latest is not None else None,
        }

    def start(self) -> bool:
        if self._running.is_set():
            return True
        try:
            factory = self.serial_factory
            if factory is None:
                import serial
                factory = serial.Serial
            try:
                self._serial = factory(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout_s,
                    write_timeout=0.03,
                )
            except TypeError:
                self._serial = factory(self.port, self.baudrate, timeout=self.timeout_s)
        except Exception as exc:
            LOG.error("Cyber Camera serial open failed (%s): %s", self.port, exc)
            return False
        self._next_ping_monotonic = time.monotonic()
        self._running.set()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="cybercam-vs1")
        self._thread.start()
        return True

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._publish_status(time.monotonic(), running=False, force=True)

    def latest(self, now: float | None = None, max_age_s: float = 0.15) -> PlatformObservation | None:
        timestamp = time.monotonic() if now is None else now
        with self._lock:
            observation = self._latest
        if observation is None or observation.age_s(timestamp) > max_age_s:
            return None
        return observation

    def _worker(self) -> None:
        buffer = bytearray()
        while self._running.is_set():
            self._maybe_send_ping(time.monotonic())
            try:
                data = self._serial.read(128)
                if data:
                    buffer.extend(data)
                    if len(buffer) > self.max_line_bytes * 2:
                        newline = buffer.rfind(b"\n")
                        buffer[:] = buffer[newline + 1:] if newline >= 0 else b""
                        self.parse_errors += 1
                    while b"\n" in buffer:
                        line, remainder = buffer.split(b"\n", 1)
                        buffer[:] = remainder
                        self._handle_line(line, time.monotonic())
            except Exception as exc:
                LOG.error("Cyber Camera serial read failed: %s", exc)
                self._running.clear()
            self._publish_status(time.monotonic(), running=self._running.is_set())
        self._publish_status(time.monotonic(), running=False, force=True)

    def _handle_line(self, line: bytes, received_monotonic: float) -> None:
        if not line or len(line) > self.max_line_bytes:
            self.parse_errors += 1
            return
        if line.startswith(b"VC1,"):
            try:
                command, seq = parse_control(line)
            except ValueError:
                self.control_errors += 1
                return
            if command != "PONG":
                self.control_errors += 1
                return
            for pending_seq, sent_at in tuple(self._pending_pings):
                if pending_seq == seq and 0.0 <= received_monotonic - sent_at <= 3.0:
                    self._pending_pings.remove((pending_seq, sent_at))
                    with self._lock:
                        self.last_pong_monotonic = received_monotonic
                        self.pongs_received += 1
                    return
            self.control_errors += 1
            return
        try:
            observation = parse_line(line, received_monotonic)
        except ValueError:
            self.parse_errors += 1
            return
        accepted = self._gate.accept(observation)
        if accepted is not None:
            with self._lock:
                self._latest = accepted
                self.accepted_frames += 1
                self.last_received_monotonic = accepted.received_monotonic

    def _maybe_send_ping(self, now: float) -> None:
        while self._pending_pings and now - self._pending_pings[0][1] > 3.0:
            self._pending_pings.popleft()
        if now < self._next_ping_monotonic:
            return
        self._next_ping_monotonic = now + 1.0
        seq = self._ping_seq
        self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
        try:
            self._serial.write(encode_control("PING", seq))
        except Exception as exc:
            self.ping_write_errors += 1
            LOG.warning("Cyber Camera PING send failed: %s", exc)
            return
        self._pending_pings.append((seq, now))
        self.pings_sent += 1

    def _publish_status(self, now: float, running: bool, force: bool = False) -> None:
        with self._lock:
            values = {
                "running": bool(running),
                "last_vs1_monotonic": self.last_received_monotonic,
                "last_pong_monotonic": self.last_pong_monotonic,
                "accepted_frames": self.accepted_frames,
                "pongs_received": self.pongs_received,
            }
        values.update({
            "pings_sent": self.pings_sent,
            "ping_write_errors": self.ping_write_errors,
            "parse_errors": self.parse_errors,
            "control_errors": self.control_errors,
        })
        published = self._status.publish(values, now=now, force=force)
        if force and not published:
            LOG.warning("VS1 OLED status snapshot write failed")
