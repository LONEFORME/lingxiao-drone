"""Competition mission planning built on top of the stable ``basic`` flight core.

The planner deliberately knows nothing about cameras, radios, or the flight
controller.  It converts a field-point configuration into the XYZ waypoints
and per-point hold times consumed by :mod:`Mission_GPT`.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable, Mapping, Sequence


class CompetitionPlanError(ValueError):
    """Raised when a competition configuration or mission request is invalid."""


@dataclass(frozen=True)
class PointSpec:
    point_id: str
    x: float
    y: float
    z: float
    hold_s: float = 2.0
    action: str = "observe"

    @classmethod
    def from_mapping(
        cls, raw: Mapping[str, object], default_z: float
    ) -> "PointSpec":
        try:
            point_id = str(raw["id"]).strip()
            x = float(raw["x"])
            y = float(raw["y"])
            z = float(raw.get("z", default_z))
            hold_s = float(raw.get("hold_s", 2.0))
            action = str(raw.get("action", "observe")).strip() or "observe"
        except (KeyError, TypeError, ValueError) as exc:
            raise CompetitionPlanError(f"Invalid point definition: {raw!r}") from exc
        if not point_id:
            raise CompetitionPlanError("Point id cannot be empty")
        if z <= 0:
            raise CompetitionPlanError(f"Point {point_id} height must be positive")
        if hold_s < 0:
            raise CompetitionPlanError(f"Point {point_id} hold_s cannot be negative")
        return cls(point_id, x, y, z, hold_s, action)


@dataclass(frozen=True)
class CompetitionConfig:
    name: str
    home: PointSpec
    points: tuple[PointSpec, ...]
    scout_order: tuple[str, ...]
    home_hold_s: float = 1.0

    @property
    def point_map(self) -> dict[str, PointSpec]:
        return {point.point_id: point for point in self.points}


@dataclass(frozen=True)
class PlannedMission:
    phase: str
    point_ids: tuple[str, ...]
    waypoints: tuple[tuple[float, float, float], ...]
    hold_s: tuple[float, ...]
    actions: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "phase": self.phase,
            "point_ids": list(self.point_ids),
            "waypoints": [list(point) for point in self.waypoints],
            "hold_s": list(self.hold_s),
            "actions": list(self.actions),
        }


def load_competition_config(path: str | Path) -> CompetitionConfig:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CompetitionPlanError(f"Cannot load config {config_path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise CompetitionPlanError("Competition config root must be an object")

    try:
        name = str(raw.get("name", config_path.stem))
        cruise_height = float(raw.get("cruise_height_m", 1.0))
        home_hold_s = float(raw.get("home_hold_s", 1.0))
        home_raw = raw["home"]
        points_raw = raw["points"]
    except (KeyError, TypeError, ValueError) as exc:
        raise CompetitionPlanError("Config is missing valid home/points values") from exc
    if not isinstance(home_raw, dict) or not isinstance(points_raw, list):
        raise CompetitionPlanError("home must be an object and points must be a list")
    if home_hold_s < 0:
        raise CompetitionPlanError("home_hold_s cannot be negative")

    home_data = dict(home_raw)
    home_data.setdefault("id", "HOME")
    home = PointSpec.from_mapping(home_data, cruise_height)
    points = tuple(PointSpec.from_mapping(point, cruise_height) for point in points_raw)
    if not points:
        raise CompetitionPlanError("At least one competition point is required")

    ids = [point.point_id for point in points]
    if len(ids) != len(set(ids)):
        raise CompetitionPlanError("Competition point ids must be unique")
    if home.point_id in set(ids):
        raise CompetitionPlanError("Home id cannot duplicate a competition point id")

    order_raw = raw.get("scout_order", ids)
    if not isinstance(order_raw, list):
        raise CompetitionPlanError("scout_order must be a list")
    scout_order = tuple(str(point_id).strip() for point_id in order_raw)
    _resolve_points(points, scout_order)
    if len(scout_order) != len(set(scout_order)):
        raise CompetitionPlanError("scout_order cannot contain duplicate point ids")

    return CompetitionConfig(name, home, points, scout_order, home_hold_s)


def _resolve_points(
    points: Sequence[PointSpec], requested_ids: Iterable[str]
) -> tuple[PointSpec, ...]:
    point_map = {point.point_id: point for point in points}
    resolved = []
    for requested in requested_ids:
        point_id = str(requested).strip()
        if point_id not in point_map:
            raise CompetitionPlanError(f"Unknown competition point: {point_id}")
        resolved.append(point_map[point_id])
    return tuple(resolved)


def plan_mission(
    config: CompetitionConfig,
    phase: str,
    selected_ids: Iterable[str] = (),
) -> PlannedMission:
    normalized_phase = phase.strip().lower()
    if normalized_phase == "scout":
        selected = _resolve_points(config.points, config.scout_order)
    elif normalized_phase == "execute":
        requested = tuple(str(point_id).strip() for point_id in selected_ids if str(point_id).strip())
        if not requested:
            raise CompetitionPlanError("Execute phase requires at least one selected point")
        if len(requested) != len(set(requested)):
            raise CompetitionPlanError("Execute point list cannot contain duplicates")
        selected = _resolve_points(config.points, requested)
    else:
        raise CompetitionPlanError("phase must be scout or execute")

    route = (config.home, *selected, config.home)
    return PlannedMission(
        phase=normalized_phase,
        point_ids=tuple(point.point_id for point in route),
        waypoints=tuple((point.x, point.y, point.z) for point in route),
        hold_s=(config.home_hold_s, *(point.hold_s for point in selected), config.home_hold_s),
        actions=("depart", *(point.action for point in selected), "return"),
    )

