"""Static competition preflight checks that never command the aircraft."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import shutil
from typing import Iterable, Mapping

from Lcode.action_executor import ActionPolicy


class PreflightConfigError(ValueError):
    pass


@dataclass(frozen=True)
class PreflightConfig:
    min_free_mb: int = 128

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "PreflightConfig":
        if raw is None:
            return cls()
        if not isinstance(raw, Mapping):
            raise PreflightConfigError("preflight must be an object")
        value = raw.get("min_free_mb", 128)
        if isinstance(value, bool):
            raise PreflightConfigError("preflight.min_free_mb must be positive")
        try:
            normalized = int(value)
        except (TypeError, ValueError) as exc:
            raise PreflightConfigError(
                "preflight.min_free_mb must be positive"
            ) from exc
        if normalized <= 0 or normalized != value:
            raise PreflightConfigError("preflight.min_free_mb must be positive")
        return cls(normalized)

    def as_dict(self) -> dict[str, object]:
        return {"min_free_mb": self.min_free_mb}


@dataclass(frozen=True)
class ServiceReadiness:
    ready: bool
    required: bool
    detail: str = ""


@dataclass(frozen=True)
class PreflightCheck:
    name: str
    level: str
    detail: str

    def as_dict(self) -> dict[str, str]:
        return {"name": self.name, "level": self.level, "detail": self.detail}


@dataclass(frozen=True)
class PreflightReport:
    checks: tuple[PreflightCheck, ...]

    @property
    def ok(self) -> bool:
        return not any(check.level == "fail" for check in self.checks)

    @property
    def failures(self) -> tuple[PreflightCheck, ...]:
        return tuple(check for check in self.checks if check.level == "fail")

    @property
    def warnings(self) -> tuple[PreflightCheck, ...]:
        return tuple(check for check in self.checks if check.level == "warn")

    def as_dict(self) -> dict[str, object]:
        return {
            "ok": self.ok,
            "checks": [check.as_dict() for check in self.checks],
        }


def load_preflight_config(path: str | Path) -> PreflightConfig:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PreflightConfigError(f"cannot load preflight config: {exc}") from exc
    if not isinstance(raw, Mapping):
        raise PreflightConfigError("competition config root must be an object")
    return PreflightConfig.from_mapping(raw.get("preflight"))


class CompetitionPreflight:
    def __init__(self, config: PreflightConfig):
        self.config = config

    def run(
        self,
        point_ids: Iterable[str],
        waypoints: Iterable[Iterable[float]],
        actions: Iterable[str],
        action_policy: ActionPolicy,
        supported_actions: Iterable[str],
        session_dir: str | Path,
        services: Mapping[str, ServiceReadiness] | None = None,
    ) -> PreflightReport:
        checks: list[PreflightCheck] = []
        ids = tuple(str(point_id).strip() for point_id in point_ids)
        points = tuple(tuple(point) for point in waypoints)
        normalized_actions = tuple(str(action).strip().lower() for action in actions)
        supported = {str(action).strip().lower() for action in supported_actions}

        if ids and len(ids) == len(points) == len(normalized_actions):
            checks.append(PreflightCheck("route_shape", "pass", f"{len(ids)} points"))
        else:
            checks.append(
                PreflightCheck(
                    "route_shape",
                    "fail",
                    "point_ids/waypoints/actions must be non-empty and equal length",
                )
            )
        if ids and ids[0] == "HOME" and ids[-1] == "HOME":
            checks.append(PreflightCheck("home_endpoints", "pass", "HOME -> HOME"))
        else:
            checks.append(
                PreflightCheck("home_endpoints", "fail", "route must start/end HOME")
            )

        invalid_height = False
        for point in points:
            try:
                invalid_height = len(point) < 3 or float(point[2]) <= 0
            except (TypeError, ValueError):
                invalid_height = True
            if invalid_height:
                break
        checks.append(
            PreflightCheck(
                "positive_height",
                "fail" if invalid_height else "pass",
                "all configured heights positive"
                if not invalid_height
                else "invalid or non-positive height",
            )
        )

        unknown = sorted(set(normalized_actions) - supported)
        disallowed = sorted(set(normalized_actions) - set(action_policy.allowed_actions))
        action_errors = sorted(set(unknown + disallowed))
        checks.append(
            PreflightCheck(
                "actions_supported",
                "fail" if action_errors else "pass",
                (
                    f"unsupported actions: {','.join(action_errors)}"
                    if action_errors
                    else "all actions supported"
                ),
            )
        )

        directory = Path(session_dir).resolve()
        try:
            directory.mkdir(parents=True, exist_ok=True)
            probe = directory / ".preflight_write_probe.tmp"
            probe.write_bytes(b"ok")
            probe.unlink()
            checks.append(PreflightCheck("session_writable", "pass", str(directory)))
        except OSError as exc:
            checks.append(
                PreflightCheck("session_writable", "fail", f"{directory}: {exc}")
            )

        try:
            free_mb = shutil.disk_usage(directory).free // (1024 * 1024)
            checks.append(
                PreflightCheck(
                    "disk_space",
                    "pass" if free_mb >= self.config.min_free_mb else "fail",
                    f"{free_mb} MiB free; require {self.config.min_free_mb} MiB",
                )
            )
        except OSError as exc:
            checks.append(PreflightCheck("disk_space", "fail", str(exc)))

        for name, readiness in (services or {}).items():
            level = "pass" if readiness.ready else ("fail" if readiness.required else "warn")
            checks.append(
                PreflightCheck(
                    f"service:{name}",
                    level,
                    readiness.detail or ("ready" if readiness.ready else "unavailable"),
                )
            )

        checks.append(
            PreflightCheck(
                "geofence",
                "pass",
                "disabled by competition strategy; no deviation-triggered abort",
            )
        )
        return PreflightReport(tuple(checks))
