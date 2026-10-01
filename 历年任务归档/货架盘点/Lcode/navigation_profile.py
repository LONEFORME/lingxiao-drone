"""basic 的航点到达策略配置。"""

from collections.abc import Mapping
from dataclasses import dataclass
import os
from typing import Optional


@dataclass(frozen=True)
class NavigationProfileConfig:
    profile: str = "precision"
    precision_head: int = 1
    precision_tail: int = 1
    cruise_radius_m: float = 0.15
    cruise_confirm_cycles: int = 3
    cruise_require_z: bool = False
    cruise_timeout_base_s: float = 25.0
    cruise_min_progress_mps: float = 0.20
    cruise_timeout_margin_s: float = 5.0

    def __post_init__(self) -> None:
        if self.profile not in {"precision", "cruise"}:
            raise ValueError("DRONE_NAV_PROFILE 只能是 precision/cruise")
        if not 1 <= self.precision_head <= 100:
            raise ValueError("precision_head 必须在 [1, 100] 内")
        if not 1 <= self.precision_tail <= 100:
            raise ValueError("precision_tail 必须在 [1, 100] 内")
        if not 0.05 <= self.cruise_radius_m <= 1.0:
            raise ValueError("cruise_radius_m 必须在 [0.05, 1.0] 内")
        if not 2 <= self.cruise_confirm_cycles <= 20:
            raise ValueError("cruise_confirm_cycles 必须在 [2, 20] 内")
        if not 5.0 <= self.cruise_timeout_base_s <= 300.0:
            raise ValueError("cruise_timeout_base_s 必须在 [5, 300] 内")
        if not 0.05 <= self.cruise_min_progress_mps <= 0.4:
            raise ValueError("cruise_min_progress_mps 必须在 [0.05, 0.4] 内")
        if not 0.0 <= self.cruise_timeout_margin_s <= 60.0:
            raise ValueError("cruise_timeout_margin_s 必须在 [0, 60] 内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None) -> "NavigationProfileConfig":
        env = os.environ if environ is None else environ
        return cls(
            profile=env.get("DRONE_NAV_PROFILE", "precision").strip().lower(),
            precision_head=int(env.get("DRONE_CRUISE_PRECISION_HEAD", "1")),
            precision_tail=int(env.get("DRONE_CRUISE_PRECISION_TAIL", "1")),
            cruise_radius_m=float(env.get("DRONE_CRUISE_RADIUS_M", "0.15")),
            cruise_confirm_cycles=int(env.get("DRONE_CRUISE_CONFIRM_CYCLES", "3")),
            cruise_require_z=env.get("DRONE_CRUISE_REQUIRE_Z", "0").strip().lower()
            in {"1", "true", "yes", "on"},
            cruise_timeout_base_s=float(env.get("DRONE_CRUISE_TIMEOUT_S", "25")),
            cruise_min_progress_mps=float(env.get("DRONE_CRUISE_MIN_PROGRESS_MPS", "0.20")),
            cruise_timeout_margin_s=float(env.get("DRONE_CRUISE_TIMEOUT_MARGIN_S", "5")),
        )

    def waypoint_mode(self, target_index: int, target_count: int) -> str:
        if not 0 <= target_index < target_count:
            raise IndexError("target_index 超出航点范围")
        if self.profile == "precision":
            return "precision"
        if target_index < self.precision_head:
            return "precision"
        if target_index >= target_count - self.precision_tail:
            return "precision"
        return "cruise"

    def cruise_timeout_s(self, segment_distance_m: float) -> float:
        if segment_distance_m < 0:
            raise ValueError("segment_distance_m 不能为负")
        distance_budget = (
            self.cruise_timeout_margin_s
            + segment_distance_m / self.cruise_min_progress_mps
        )
        return max(self.cruise_timeout_base_s, distance_budget)
