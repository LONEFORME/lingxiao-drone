"""Hardware-neutral video-link interfaces for the competition system.

This module intentionally has no OpenCV, GStreamer, or FFmpeg dependency.
Concrete UVC/RTSP/UDP backends can be added later without changing mission
planning or flight-control code.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import Any, Callable, Mapping, Optional


class VideoSourceError(RuntimeError):
    """Raised when a configured video source cannot be created or started."""


@dataclass(frozen=True)
class VideoSourceConfig:
    enabled: bool = False
    backend: str = "none"
    source: str = ""
    snapshot_dir: str = "snapshots"
    options: Mapping[str, object] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "VideoSourceConfig":
        if raw is None:
            return cls()
        if not isinstance(raw, Mapping):
            raise VideoSourceError("video config must be an object")
        enabled = bool(raw.get("enabled", False))
        backend = str(raw.get("backend", "none")).strip().lower() or "none"
        source = str(raw.get("source", "")).strip()
        snapshot_dir = str(raw.get("snapshot_dir", "snapshots")).strip()
        options = raw.get("options", {})
        if not isinstance(options, Mapping):
            raise VideoSourceError("video.options must be an object")
        if not snapshot_dir:
            raise VideoSourceError("video.snapshot_dir cannot be empty")
        if enabled and backend == "none":
            raise VideoSourceError("enabled video config requires a real backend")
        return cls(enabled, backend, source, snapshot_dir, dict(options))

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "backend": self.backend,
            "source": self.source,
            "snapshot_dir": self.snapshot_dir,
            "options": dict(self.options),
        }


@dataclass(frozen=True)
class VideoPublisherConfig:
    enabled: bool = False
    backend: str = "none"
    target: str = ""
    options: Mapping[str, object] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object] | None) -> "VideoPublisherConfig":
        if raw is None:
            return cls()
        if not isinstance(raw, Mapping):
            raise VideoSourceError("video publisher config must be an object")
        enabled = bool(raw.get("enabled", False))
        backend = str(raw.get("backend", "none")).strip().lower() or "none"
        target = str(raw.get("target", "")).strip()
        options = raw.get("options", {})
        if not isinstance(options, Mapping):
            raise VideoSourceError("video publisher options must be an object")
        if enabled and backend == "none":
            raise VideoSourceError("enabled video publisher requires a real backend")
        return cls(enabled, backend, target, dict(options))

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "backend": self.backend,
            "target": self.target,
            "options": dict(self.options),
        }


@dataclass(frozen=True)
class VideoProfileConfig:
    name: str
    receiver: VideoSourceConfig
    publisher: VideoPublisherConfig

    def as_dict(self) -> dict[str, object]:
        return {
            "receiver": self.receiver.as_dict(),
            "publisher": self.publisher.as_dict(),
        }


@dataclass(frozen=True)
class VideoCatalogConfig:
    active_profile: str
    profiles: Mapping[str, VideoProfileConfig]

    @property
    def active(self) -> Optional[VideoProfileConfig]:
        if self.active_profile == "none":
            return None
        return self.profiles.get(self.active_profile)

    def as_dict(self) -> dict[str, object]:
        return {
            "active_profile": self.active_profile,
            "profiles": {
                name: profile.as_dict() for name, profile in self.profiles.items()
            },
        }


@dataclass(frozen=True)
class VideoFrame:
    """One decoded frame returned by a concrete backend.

    ``payload`` is deliberately opaque. A UVC backend may return a NumPy BGR
    array, while a network backend may return RGB bytes or a backend-owned
    frame object.
    """

    sequence: int
    captured_at: float
    width: int
    height: int
    pixel_format: str
    payload: Any


@dataclass(frozen=True)
class SnapshotResult:
    point_id: str
    path: Optional[Path]
    captured_at: Optional[float]
    error: Optional[str] = None

    @property
    def ok(self) -> bool:
        return self.path is not None and self.error is None


class VideoSource(ABC):
    """Common contract for local cameras and wireless video receivers."""

    def __enter__(self) -> "VideoSource":
        if not self.start():
            raise VideoSourceError("video source failed to start")
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.stop()

    @abstractmethod
    def start(self) -> bool:
        """Open the receiver and begin decoding frames."""

    @abstractmethod
    def read_frame(self, timeout_s: float = 0.5) -> Optional[VideoFrame]:
        """Return the next decoded frame, or ``None`` on timeout."""

    @abstractmethod
    def snapshot(
        self, point_id: str, output_dir: str | Path, timeout_s: float = 1.0
    ) -> SnapshotResult:
        """Save a current frame for a mission point.

        Concrete backends must apply ``timeout_s`` to their lowest available
        camera/network read primitive, keep the final path inside
        ``output_dir``, and write via a same-directory temporary file followed
        by an atomic replace. Python-side elapsed-time checks cannot terminate
        a driver or FFI call that is already blocked.
        """

    @abstractmethod
    def is_running(self) -> bool:
        """Whether the source is currently able to receive frames."""

    @abstractmethod
    def stop(self) -> None:
        """Release receiver and decoder resources."""


class VideoPublisher(ABC):
    """Airborne-side interface used by the board-to-board video plan."""

    @abstractmethod
    def start(self) -> bool:
        """Open encoder/network resources and begin publishing."""

    @abstractmethod
    def publish_frame(self, frame: VideoFrame) -> bool:
        """Encode and publish one frame."""

    @abstractmethod
    def is_running(self) -> bool:
        """Whether the publisher can currently send frames."""

    @abstractmethod
    def stop(self) -> None:
        """Release encoder and network resources."""


class NullVideoSource(VideoSource):
    """Disabled video implementation used until hardware is selected."""

    def start(self) -> bool:
        return True

    def read_frame(self, timeout_s: float = 0.5) -> Optional[VideoFrame]:
        return None

    def snapshot(
        self, point_id: str, output_dir: str | Path, timeout_s: float = 1.0
    ) -> SnapshotResult:
        return SnapshotResult(
            point_id=point_id,
            path=None,
            captured_at=None,
            error="video_disabled",
        )

    def is_running(self) -> bool:
        return False

    def stop(self) -> None:
        return None


class NullVideoPublisher(VideoPublisher):
    """Publisher used by external hardware transmitters and disabled video."""

    def start(self) -> bool:
        return True

    def publish_frame(self, frame: VideoFrame) -> bool:
        return False

    def is_running(self) -> bool:
        return False

    def stop(self) -> None:
        return None


VideoSourceFactory = Callable[[VideoSourceConfig], VideoSource]
VideoPublisherFactory = Callable[[VideoPublisherConfig], VideoPublisher]
_SOURCE_FACTORIES: dict[str, VideoSourceFactory] = {}
_PUBLISHER_FACTORIES: dict[str, VideoPublisherFactory] = {}


def register_video_source_backend(name: str, factory: VideoSourceFactory) -> None:
    backend = name.strip().lower()
    if not backend or backend == "none":
        raise VideoSourceError("video source backend name is invalid")
    _SOURCE_FACTORIES[backend] = factory


def register_video_publisher_backend(name: str, factory: VideoPublisherFactory) -> None:
    backend = name.strip().lower()
    if not backend or backend == "none":
        raise VideoSourceError("video publisher backend name is invalid")
    _PUBLISHER_FACTORIES[backend] = factory


def load_video_catalog(path: str | Path) -> VideoCatalogConfig:
    config_path = Path(path)
    try:
        raw = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VideoSourceError(f"Cannot load video config {config_path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise VideoSourceError("competition config root must be an object")
    video_raw = raw.get("video")
    if video_raw is None:
        return VideoCatalogConfig("none", {})
    if not isinstance(video_raw, Mapping):
        raise VideoSourceError("video config must be an object")
    active_profile = str(video_raw.get("active_profile", "none")).strip() or "none"
    profiles_raw = video_raw.get("profiles", {})
    if not isinstance(profiles_raw, Mapping):
        raise VideoSourceError("video.profiles must be an object")

    snapshot_dir = str(video_raw.get("snapshot_dir", "snapshots")).strip()
    profiles: dict[str, VideoProfileConfig] = {}
    for name, profile_raw in profiles_raw.items():
        if not isinstance(profile_raw, Mapping):
            raise VideoSourceError(f"video profile {name!r} must be an object")
        receiver_raw = dict(profile_raw.get("receiver", {}))
        receiver_raw.setdefault("snapshot_dir", snapshot_dir)
        profiles[str(name)] = VideoProfileConfig(
            name=str(name),
            receiver=VideoSourceConfig.from_mapping(receiver_raw),
            publisher=VideoPublisherConfig.from_mapping(profile_raw.get("publisher")),
        )
    if active_profile != "none" and active_profile not in profiles:
        raise VideoSourceError(f"unknown active video profile: {active_profile}")
    return VideoCatalogConfig(active_profile, profiles)


def load_video_config(path: str | Path) -> VideoSourceConfig:
    catalog = load_video_catalog(path)
    return catalog.active.receiver if catalog.active else VideoSourceConfig()


def load_video_publisher_config(path: str | Path) -> VideoPublisherConfig:
    catalog = load_video_catalog(path)
    return catalog.active.publisher if catalog.active else VideoPublisherConfig()


def create_video_source(config: VideoSourceConfig) -> VideoSource:
    if not config.enabled or config.backend == "none":
        return NullVideoSource()
    factory = _SOURCE_FACTORIES.get(config.backend)
    if factory is None:
        raise VideoSourceError(
            f"Video source backend {config.backend!r} is reserved but not registered"
        )
    return factory(config)


def create_video_publisher(config: VideoPublisherConfig) -> VideoPublisher:
    if not config.enabled or config.backend == "none":
        return NullVideoPublisher()
    factory = _PUBLISHER_FACTORIES.get(config.backend)
    if factory is None:
        raise VideoSourceError(
            f"Video publisher backend {config.backend!r} is reserved but not registered"
        )
    return factory(config)
