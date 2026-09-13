from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag


class FeatureFlag(IntFlag):
    OUTER_VALID = 1 << 0
    INNER_VALID = 1 << 1
    CROSS_VALID = 1 << 2
    PARTIAL = 1 << 3
    TOO_CLOSE = 1 << 4
    AMBIGUOUS = 1 << 5
    SURROGATE_SQUARE = 1 << 6
    APRILTAG_VALID = 1 << 7
    TEMPORAL_TRACKED = 1 << 8
    COLOR_SHAPE_TRACKED = 1 << 9


@dataclass(frozen=True)
class PlatformObservation:
    stream_id: int
    seq: int
    capture_ms: int
    found: bool
    cx: int
    cy: int
    outer_px: int
    inner_px: int
    angle_cdeg: int
    quality: int
    flags: int
    received_monotonic: float

    def age_s(self, now: float) -> float:
        return max(0.0, float(now) - self.received_monotonic)

    def usable(
        self,
        now: float,
        max_age_s: float,
        min_quality: int,
        *,
        allow_surrogate: bool = False,
        target_source: str | None = None,
    ) -> bool:
        bad = FeatureFlag.TOO_CLOSE | FeatureFlag.AMBIGUOUS
        flags = FeatureFlag(self.flags)
        surrogate = bool(flags & FeatureFlag.SURROGATE_SQUARE)
        apriltag = bool(flags & FeatureFlag.APRILTAG_VALID)
        if surrogate and apriltag:
            return False
        if apriltag and flags & FeatureFlag.PARTIAL:
            return False
        if target_source is not None:
            if target_source not in ("apriltag", "blue_square"):
                raise ValueError(f"unsupported target source: {target_source}")
            source_allowed = apriltag if target_source == "apriltag" else surrogate
        else:
            source_allowed = allow_surrogate or not surrogate
        return (
            self.found
            and self.age_s(now) <= max_age_s
            and self.quality >= min_quality
            and not (flags & bad)
            and source_allowed
        )
