"""Shared VS1 duplex status snapshot for the UART owner and OLED observer."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path


STATUS_PATH = Path("/dev/shm/competition_2026_d_vs1_status.json")
SNAPSHOT_TIMEOUT_S = 3.0
CAM_TO_RDK_TIMEOUT_S = 1.5
RDK_TO_CAM_TIMEOUT_S = 2.5


class LinkStatusPublisher:
    def __init__(self, path: Path | str = STATUS_PATH, interval_s: float = 0.5):
        self.path = Path(path)
        self.interval_s = float(interval_s)
        self.pid = os.getpid()
        self.started_monotonic = time.monotonic()
        self._last_publish = float("-inf")

    def publish(self, values: dict, now: float | None = None, force: bool = False) -> bool:
        timestamp = time.monotonic() if now is None else float(now)
        if not force and timestamp - self._last_publish < self.interval_s:
            return False
        payload = {
            "version": 1,
            "pid": self.pid,
            "started_monotonic": self.started_monotonic,
            "updated_monotonic": timestamp,
            **values,
        }
        temporary = self.path.with_name(f".{self.path.name}.{self.pid}.tmp")
        try:
            with temporary.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, separators=(",", ":"), allow_nan=False)
                handle.flush()
            os.replace(temporary, self.path)
        except (OSError, TypeError, ValueError):
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            return False
        self._last_publish = timestamp
        return True


def read_link_status(path: Path | str = STATUS_PATH) -> dict | None:
    try:
        with Path(path).open("r", encoding="utf-8") as handle:
            value = json.load(handle)
        if not isinstance(value, dict) or value.get("version") != 1:
            return None
        for key in ("pid", "started_monotonic", "updated_monotonic"):
            if key not in value:
                return None
        int(value["pid"])
        float(value["started_monotonic"])
        float(value["updated_monotonic"])
        return value
    except (OSError, TypeError, ValueError, json.JSONDecodeError):
        return None


class OledLinkEvaluator:
    """Convert snapshots to two debounced OLED state strings."""

    def __init__(self):
        self._identity: tuple[int, float] | None = None
        self._states = {"cam_to_rdk": "LOST", "rdk_to_cam": "LOST"}
        self._misses = {"cam_to_rdk": 0, "rdk_to_cam": 0}

    def evaluate(self, snapshot: dict | None, now: float | None = None) -> dict:
        timestamp = time.monotonic() if now is None else float(now)
        if snapshot is None:
            self._states = {"cam_to_rdk": "LOST", "rdk_to_cam": "LOST"}
            self._misses = {"cam_to_rdk": 0, "rdk_to_cam": 0}
            return dict(self._states)
        try:
            identity = (int(snapshot["pid"]), float(snapshot["started_monotonic"]))
            updated = float(snapshot["updated_monotonic"])
        except (KeyError, TypeError, ValueError):
            return self.evaluate(None, timestamp)
        if self._identity is None:
            self._identity = identity
        elif identity != self._identity:
            self._identity = identity
            self._states = {"cam_to_rdk": "RESTART", "rdk_to_cam": "RESTART"}
            self._misses = {"cam_to_rdk": 0, "rdk_to_cam": 0}
            return dict(self._states)

        snapshot_fresh = bool(snapshot.get("running")) and timestamp - updated <= SNAPSHOT_TIMEOUT_S
        raw = {
            "cam_to_rdk": snapshot_fresh and _recent(
                snapshot.get("last_vs1_monotonic"), timestamp, CAM_TO_RDK_TIMEOUT_S
            ),
            "rdk_to_cam": snapshot_fresh and _recent(
                snapshot.get("last_pong_monotonic"), timestamp, RDK_TO_CAM_TIMEOUT_S
            ),
        }
        for direction, online in raw.items():
            if online:
                self._states[direction] = "OK"
                self._misses[direction] = 0
            elif self._states[direction] == "OK":
                self._misses[direction] += 1
                if self._misses[direction] >= 2:
                    self._states[direction] = "LOST"
            else:
                self._states[direction] = "LOST"
                self._misses[direction] = 0
        return dict(self._states)


def _recent(value, now: float, timeout_s: float) -> bool:
    if value is None:
        return False
    try:
        age = now - float(value)
    except (TypeError, ValueError):
        return False
    return 0.0 <= age <= timeout_s
