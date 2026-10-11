"""Authenticated drone-side UDP link for events and preflight execute plans."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import hmac
import ipaddress
import json
import os
import queue
import secrets
import socket
import threading
import time
from pathlib import Path
from typing import Callable, Mapping, Optional
import zlib

from Lcode.mission_events import MissionEvent


PROTOCOL_VERSION = 1
COMMAND_TYPES = {"execute_plan"}
MODE_ACCEPT_PLAN = "ACCEPT_PLAN"
MODE_REPORT_ONLY = "REPORT_ONLY"


class DroneLinkError(RuntimeError):
    pass


@dataclass(frozen=True)
class DroneLinkConfig:
    enabled: bool = False
    required: bool = False
    bind_host: str = "0.0.0.0"
    bind_port: int = 5601
    remote_host: str = "127.0.0.1"
    remote_port: int = 5602
    allowed_host: str = "127.0.0.1"
    heartbeat_s: float = 1.0
    socket_timeout_s: float = 0.05
    stop_timeout_s: float = 0.5
    execute_plan_wait_s: float = 0.0
    timestamp_skew_s: float = 10.0
    max_datagram: int = 8192
    send_queue_size: int = 128

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "DroneLinkConfig":
        if raw is None:
            return cls()
        if not isinstance(raw, Mapping):
            raise DroneLinkError("drone_link must be an object")
        enabled = _bool(raw.get("enabled", False), "enabled")
        required = _bool(raw.get("required", False), "required")
        bind_host = _ipv4(raw.get("bind_host", "0.0.0.0"), "bind_host")
        remote_host = _ipv4(raw.get("remote_host", "127.0.0.1"), "remote_host")
        allowed_host = _ipv4(raw.get("allowed_host", remote_host), "allowed_host")
        config = cls(
            enabled=enabled,
            required=required,
            bind_host=bind_host,
            bind_port=_port(raw.get("bind_port", 5601), "bind_port"),
            remote_host=remote_host,
            remote_port=_port(raw.get("remote_port", 5602), "remote_port"),
            allowed_host=allowed_host,
            heartbeat_s=_positive_float(raw.get("heartbeat_s", 1.0), "heartbeat_s"),
            socket_timeout_s=_positive_float(
                raw.get("socket_timeout_s", 0.05), "socket_timeout_s"
            ),
            stop_timeout_s=_positive_float(
                raw.get("stop_timeout_s", 0.5), "stop_timeout_s"
            ),
            execute_plan_wait_s=_nonnegative_float(
                raw.get("execute_plan_wait_s", 0.0), "execute_plan_wait_s"
            ),
            timestamp_skew_s=_positive_float(
                raw.get("timestamp_skew_s", 10.0), "timestamp_skew_s"
            ),
            max_datagram=_bounded_int(
                raw.get("max_datagram", 8192), "max_datagram", 512, 65507
            ),
            send_queue_size=_bounded_int(
                raw.get("send_queue_size", 128), "send_queue_size", 1, 4096
            ),
        )
        if required and not enabled:
            raise DroneLinkError("required drone_link must also be enabled")
        return config

    @property
    def accepts_execute_plan(self) -> bool:
        return self.enabled and self.execute_plan_wait_s > 0

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "required": self.required,
            "bind_host": self.bind_host,
            "bind_port": self.bind_port,
            "remote_host": self.remote_host,
            "remote_port": self.remote_port,
            "allowed_host": self.allowed_host,
            "heartbeat_s": self.heartbeat_s,
            "socket_timeout_s": self.socket_timeout_s,
            "stop_timeout_s": self.stop_timeout_s,
            "execute_plan_wait_s": self.execute_plan_wait_s,
            "timestamp_skew_s": self.timestamp_skew_s,
            "max_datagram": self.max_datagram,
            "send_queue_size": self.send_queue_size,
        }


@dataclass(frozen=True)
class DroneMessage:
    message_type: str
    sequence: int
    timestamp: float
    run_id: str
    nonce: str
    payload: Mapping[str, object]


def load_drone_link_config(path: str | Path) -> DroneLinkConfig:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DroneLinkError(f"cannot load drone_link config: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise DroneLinkError("competition config root must be an object")
    return DroneLinkConfig.from_mapping(raw.get("drone_link"))


def encode_message(
    message_type: str,
    payload: Mapping[str, object],
    sequence: int,
    run_id: str,
    nonce: str,
    *,
    timestamp: Optional[float] = None,
    secret: Optional[str] = None,
) -> bytes:
    base = {
        "version": PROTOCOL_VERSION,
        "type": str(message_type),
        "sequence": int(sequence),
        "timestamp": float(time.time() if timestamp is None else timestamp),
        "run_id": str(run_id),
        "nonce": str(nonce),
        "payload": dict(payload),
    }
    if base["type"] in COMMAND_TYPES:
        if not secret:
            raise DroneLinkError("authenticated command requires a secret")
        base["hmac"] = hmac.new(
            secret.encode("utf-8"), _canonical(base), hashlib.sha256
        ).hexdigest()
    base["crc32"] = f"{zlib.crc32(_canonical(base)) & 0xFFFFFFFF:08x}"
    return _canonical(base)


def decode_message(
    data: bytes,
    *,
    max_datagram: int = 8192,
    secret: Optional[str] = None,
    expected_nonce: Optional[str] = None,
    timestamp_skew_s: float = 10.0,
    now: Optional[float] = None,
) -> DroneMessage:
    if not data or len(data) > max_datagram:
        raise DroneLinkError("invalid datagram length")
    try:
        raw = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise DroneLinkError("invalid JSON datagram") from exc
    if not isinstance(raw, dict):
        raise DroneLinkError("message root must be an object")
    received_crc = str(raw.pop("crc32", ""))
    expected_crc = f"{zlib.crc32(_canonical(raw)) & 0xFFFFFFFF:08x}"
    if not hmac.compare_digest(received_crc, expected_crc):
        raise DroneLinkError("CRC mismatch")
    try:
        version = int(raw["version"])
        message_type = str(raw["type"])
        sequence = int(raw["sequence"])
        timestamp_value = float(raw["timestamp"])
        run_id = str(raw["run_id"])
        nonce = str(raw["nonce"])
        payload = raw["payload"]
    except (KeyError, TypeError, ValueError) as exc:
        raise DroneLinkError("missing or invalid message fields") from exc
    if version != PROTOCOL_VERSION or sequence < 0:
        raise DroneLinkError("unsupported version or sequence")
    if not isinstance(payload, dict):
        raise DroneLinkError("payload must be an object")
    if message_type in COMMAND_TYPES:
        received_hmac = str(raw.pop("hmac", ""))
        if not secret:
            raise DroneLinkError("command secret unavailable")
        if expected_nonce is None or nonce != expected_nonce:
            raise DroneLinkError("nonce mismatch")
        expected_hmac = hmac.new(
            secret.encode("utf-8"), _canonical(raw), hashlib.sha256
        ).hexdigest()
        if not hmac.compare_digest(received_hmac, expected_hmac):
            raise DroneLinkError("HMAC mismatch")
        clock = time.time() if now is None else now
        if abs(clock - timestamp_value) > timestamp_skew_s:
            raise DroneLinkError("command timestamp outside allowed window")
    return DroneMessage(
        message_type,
        sequence,
        timestamp_value,
        run_id,
        nonce,
        dict(payload),
    )


class DroneLink:
    def __init__(
        self,
        config: DroneLinkConfig,
        run_id: str,
        *,
        secret: Optional[str] = None,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ):
        self.config = config
        self.run_id = str(run_id)
        self.secret = secret if secret is not None else os.getenv("DRONE_LINK_PSK")
        self.nonce = secrets.token_hex(16)
        self.socket_factory = socket_factory
        self._socket: Optional[socket.socket] = None
        self._send_queue: queue.Queue[bytes] = queue.Queue(config.send_queue_size)
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._plan_event = threading.Event()
        self._lock = threading.Lock()
        self._mode = (
            MODE_ACCEPT_PLAN if config.accepts_execute_plan else MODE_REPORT_ONLY
        )
        self._execute_plan: Optional[tuple[str, ...]] = None
        self._send_sequence = 0
        self._last_command_sequence = -1
        self._last_heartbeat = 0.0
        self.last_error: Optional[str] = None
        self._stats = {
            "sent": 0,
            "received": 0,
            "rejected": 0,
            "dropped": 0,
            "commands_accepted": 0,
            "commands_rejected": 0,
        }

    @property
    def ready(self) -> bool:
        thread = self._thread
        return self._socket is not None and thread is not None and thread.is_alive()

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    def start(self) -> bool:
        if not self.config.enabled:
            return False
        if self.config.accepts_execute_plan and not self.secret:
            self.last_error = "DRONE_LINK_PSK is required for execute_plan"
            return False
        try:
            endpoint = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            endpoint.settimeout(self.config.socket_timeout_s)
            endpoint.bind((self.config.bind_host, self.config.bind_port))
        except OSError as exc:
            self.last_error = f"socket_start_failed:{exc}"
            return False
        self._socket = endpoint
        self._thread = threading.Thread(
            target=self._run, name="drone-link", daemon=True
        )
        self._thread.start()
        self.publish(
            "hello",
            {
                "session_nonce": self.nonce,
                "mode": self.mode,
                "auth": "hmac-sha256" if self.config.accepts_execute_plan else "telemetry",
            },
        )
        return True

    def handle_event(self, event: MissionEvent) -> None:
        self.publish("mission_event", event.as_dict())

    def publish(self, message_type: str, payload: Mapping[str, object]) -> bool:
        with self._lock:
            sequence = self._send_sequence
            self._send_sequence += 1
        try:
            encoded = encode_message(
                message_type,
                payload,
                sequence,
                self.run_id,
                self.nonce,
            )
            if len(encoded) > self.config.max_datagram:
                raise DroneLinkError("outgoing datagram exceeds max_datagram")
            self._send_queue.put_nowait(encoded)
            return True
        except (DroneLinkError, queue.Full, TypeError, ValueError):
            with self._lock:
                self._stats["dropped"] += 1
            return False

    def wait_for_execute_plan(self, timeout_s: Optional[float] = None) -> tuple[str, ...]:
        wait_s = self.config.execute_plan_wait_s if timeout_s is None else timeout_s
        if not self._plan_event.wait(max(0.0, wait_s)):
            with self._lock:
                self._mode = MODE_REPORT_ONLY
            return ()
        with self._lock:
            return self._execute_plan or ()

    def set_report_only(self) -> None:
        with self._lock:
            self._mode = MODE_REPORT_ONLY

    def stop(self) -> bool:
        self._stop_event.set()
        endpoint = self._socket
        if endpoint is not None:
            try:
                endpoint.close()
            except OSError:
                pass
        thread = self._thread
        if thread is not None:
            thread.join(self.config.stop_timeout_s)
        self._socket = None
        return thread is None or not thread.is_alive()

    def stats(self) -> dict[str, int]:
        with self._lock:
            return {key: int(value) for key, value in self._stats.items()}

    def _run(self) -> None:
        endpoint = self._socket
        if endpoint is None:
            return
        remote = (self.config.remote_host, self.config.remote_port)
        while not self._stop_event.is_set():
            self._drain_one_send(endpoint, remote)
            now = time.monotonic()
            if now - self._last_heartbeat >= self.config.heartbeat_s:
                self._last_heartbeat = now
                self.publish(
                    "heartbeat",
                    {"mode": self.mode, "session_nonce": self.nonce},
                )
            try:
                data, address = endpoint.recvfrom(self.config.max_datagram + 1)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_event.is_set():
                    return
                continue
            self._receive(data, address)

    def _drain_one_send(
        self, endpoint: socket.socket, remote: tuple[str, int]
    ) -> None:
        try:
            payload = self._send_queue.get_nowait()
        except queue.Empty:
            return
        try:
            endpoint.sendto(payload, remote)
            with self._lock:
                self._stats["sent"] += 1
        except OSError:
            with self._lock:
                self._stats["dropped"] += 1
        finally:
            self._send_queue.task_done()

    def _receive(self, data: bytes, address: tuple[str, int]) -> None:
        if address[0] != self.config.allowed_host:
            self._reject(command=False)
            return
        try:
            message = decode_message(
                data,
                max_datagram=self.config.max_datagram,
                secret=self.secret,
                expected_nonce=self.nonce,
                timestamp_skew_s=self.config.timestamp_skew_s,
            )
        except DroneLinkError:
            self._reject(command=True)
            return
        with self._lock:
            self._stats["received"] += 1
            if message.message_type != "execute_plan":
                self._stats["rejected"] += 1
                return
            if self._mode != MODE_ACCEPT_PLAN:
                self._stats["commands_rejected"] += 1
                return
            if message.sequence <= self._last_command_sequence:
                self._stats["commands_rejected"] += 1
                return
            raw_points = message.payload.get("points")
            if not isinstance(raw_points, list):
                self._stats["commands_rejected"] += 1
                return
            points = tuple(str(point).strip() for point in raw_points)
            if not points or any(not point for point in points):
                self._stats["commands_rejected"] += 1
                return
            self._last_command_sequence = message.sequence
            self._execute_plan = points
            self._mode = MODE_REPORT_ONLY
            self._stats["commands_accepted"] += 1
            self._plan_event.set()

    def _reject(self, command: bool) -> None:
        with self._lock:
            self._stats["rejected"] += 1
            if command:
                self._stats["commands_rejected"] += 1


def _canonical(value: Mapping[str, object]) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def _bool(value: object, field: str) -> bool:
    if not isinstance(value, bool):
        raise DroneLinkError(f"drone_link.{field} must be boolean")
    return value


def _ipv4(value: object, field: str) -> str:
    text = str(value).strip()
    try:
        return str(ipaddress.IPv4Address(text))
    except ipaddress.AddressValueError as exc:
        raise DroneLinkError(f"drone_link.{field} must be an IPv4 address") from exc


def _port(value: object, field: str) -> int:
    return _bounded_int(value, field, 1, 65535)


def _bounded_int(value: object, field: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool):
        raise DroneLinkError(f"drone_link.{field} must be an integer")
    try:
        normalized = int(value)
    except (TypeError, ValueError) as exc:
        raise DroneLinkError(f"drone_link.{field} must be an integer") from exc
    if normalized != value or not minimum <= normalized <= maximum:
        raise DroneLinkError(
            f"drone_link.{field} must be in [{minimum}, {maximum}]"
        )
    return normalized


def _positive_float(value: object, field: str) -> float:
    normalized = _number(value, field)
    if normalized <= 0:
        raise DroneLinkError(f"drone_link.{field} must be positive")
    return normalized


def _nonnegative_float(value: object, field: str) -> float:
    normalized = _number(value, field)
    if normalized < 0:
        raise DroneLinkError(f"drone_link.{field} cannot be negative")
    return normalized


def _number(value: object, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise DroneLinkError(f"drone_link.{field} must be numeric") from exc
