import json

import pytest

from Lcode.video_source import (
    NullVideoSource,
    NullVideoPublisher,
    VideoFrame,
    VideoPublisherConfig,
    VideoSourceConfig,
    VideoSourceError,
    create_video_publisher,
    create_video_source,
    load_video_catalog,
    load_video_config,
    load_video_publisher_config,
    register_video_publisher_backend,
    register_video_source_backend,
)


def test_missing_video_config_defaults_to_disabled(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"home": {}, "points": []}), encoding="utf-8")

    config = load_video_config(path)

    assert config == VideoSourceConfig()
    assert isinstance(create_video_source(config), NullVideoSource)


def test_null_source_exposes_safe_empty_contract(tmp_path):
    source = NullVideoSource()

    assert source.start()
    assert source.read_frame(0) is None
    assert not source.snapshot("P1", tmp_path, 0).ok
    assert not source.is_running()
    source.stop()


def test_enabled_unimplemented_backend_fails_explicitly():
    config = VideoSourceConfig(enabled=True, backend="rtsp", source="rtsp://camera/live")

    with pytest.raises(VideoSourceError, match="not registered"):
        create_video_source(config)


def test_enabled_none_backend_is_invalid():
    with pytest.raises(VideoSourceError):
        VideoSourceConfig.from_mapping({"enabled": True, "backend": "none"})


def test_catalog_exposes_both_reserved_plans(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(
        json.dumps(
            {
                "video": {
                    "active_profile": "board_network",
                    "profiles": {
                        "capture_device": {
                            "receiver": {
                                "enabled": True,
                                "backend": "capture_device",
                                "source": "/dev/video0",
                            }
                        },
                        "board_network": {
                            "receiver": {
                                "enabled": True,
                                "backend": "network_stream",
                                "source": "rtsp://ground/live",
                            },
                            "publisher": {
                                "enabled": True,
                                "backend": "network_stream",
                                "target": "rtsp://0.0.0.0/live",
                            },
                        },
                    },
                }
            }
        ),
        encoding="utf-8",
    )

    catalog = load_video_catalog(path)

    assert set(catalog.profiles) == {"capture_device", "board_network"}
    assert load_video_config(path).backend == "network_stream"
    assert load_video_publisher_config(path).backend == "network_stream"


def test_backend_registries_allow_late_hardware_binding():
    source = NullVideoSource()
    publisher = NullVideoPublisher()
    register_video_source_backend("test_source", lambda config: source)
    register_video_publisher_backend("test_publisher", lambda config: publisher)

    assert create_video_source(
        VideoSourceConfig(enabled=True, backend="test_source")
    ) is source
    assert create_video_publisher(
        VideoPublisherConfig(enabled=True, backend="test_publisher")
    ) is publisher
    assert not publisher.publish_frame(VideoFrame(1, 0.0, 1, 1, "gray8", b"x"))
