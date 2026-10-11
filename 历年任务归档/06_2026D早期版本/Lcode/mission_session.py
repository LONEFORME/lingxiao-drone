"""Persistent two-flight competition session records."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import threading
from typing import Mapping, Optional
import uuid

from Lcode.mission_events import MissionEvent


class MissionSessionError(RuntimeError):
    pass


class MissionSession:
    def __init__(self, path: Path):
        self.path = path.resolve()
        self.path.mkdir(parents=True, exist_ok=True)
        (self.path / "snapshots").mkdir(exist_ok=True)
        self._lock = threading.Lock()

    @classmethod
    def create(
        cls, root: str | Path, requested: Optional[str | Path] = None
    ) -> "MissionSession":
        if requested:
            requested_path = Path(requested)
            path = requested_path if requested_path.is_absolute() else Path(root) / requested_path
        else:
            path = Path(root) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return cls(path)

    def begin(self, mission_name: str, phase: str, plan: Mapping[str, object]) -> str:
        now = datetime.now().astimezone().isoformat()
        run_id = uuid.uuid4().hex
        session_path = self.path / "session.json"
        existing: dict[str, object] = {}
        if session_path.exists():
            try:
                existing = json.loads(session_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise MissionSessionError(f"cannot read {session_path}: {exc}") from exc
        phases = dict(existing.get("phases", {}))
        phases[phase] = {
            "status": "planned",
            "updated_at": now,
            "run_id": run_id,
        }
        metadata = {
            **existing,
            "mission_name": mission_name,
            "created_at": existing.get("created_at", now),
            "updated_at": now,
            "phases": phases,
        }
        self._write_json(session_path, metadata)
        self._write_json(
            self.path / f"{phase}_plan.json", {**dict(plan), "run_id": run_id}
        )
        return run_id

    def finish(self, phase: str, status: str, **details: object) -> None:
        now = datetime.now().astimezone().isoformat()
        session_path = self.path / "session.json"
        try:
            metadata = json.loads(session_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise MissionSessionError(f"cannot update {session_path}: {exc}") from exc
        phases = dict(metadata.get("phases", {}))
        previous_phase = dict(phases.get(phase, {}))
        phases[phase] = {
            **previous_phase,
            "status": status,
            "updated_at": now,
            **details,
        }
        metadata["phases"] = phases
        metadata["updated_at"] = now
        self._write_json(session_path, metadata)
        self._write_json(
            self.path / f"{phase}_result.json",
            {
                "phase": phase,
                "status": status,
                "updated_at": now,
                **({"run_id": previous_phase["run_id"]} if "run_id" in previous_phase else {}),
                **details,
            },
        )

    def record_event(self, event: MissionEvent) -> None:
        line = json.dumps(event.as_dict(), ensure_ascii=False) + "\n"
        with self._lock:
            with (self.path / "events.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(line)

    @property
    def snapshots_dir(self) -> Path:
        return self.path / "snapshots"

    @staticmethod
    def _write_json(path: Path, value: Mapping[str, object]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary.replace(path)
