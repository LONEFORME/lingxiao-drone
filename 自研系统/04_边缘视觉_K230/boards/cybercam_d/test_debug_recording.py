import numpy as np

from CyberCamera.boards.cybercam_d.detector import PlatformDetection
from CyberCamera.boards.cybercam_d.main import DebugFrameRecorder


def _frame():
    return np.zeros((48, 64, 3), dtype=np.uint8)


def test_debug_record_off_does_not_create_directory(tmp_path):
    root = tmp_path / "frames"
    recorder = DebugFrameRecorder(False, str(root), 1.0)
    result = recorder.maybe_record(
        _frame(), PlatformDetection(False), "blue_square", 35.0, 1, 100, now=0.0
    )
    assert result is None
    assert not root.exists()


def test_debug_record_writes_at_configured_interval(tmp_path):
    recorder = DebugFrameRecorder(True, str(tmp_path / "frames"), 1.0)
    detection = PlatformDetection(True, cx=30, cy=20, quality=90)

    first = recorder.maybe_record(
        _frame(), detection, "blue_square", 35.0, 1, 100, now=0.0
    )
    skipped = recorder.maybe_record(
        _frame(), detection, "blue_square", 35.0, 2, 200, now=0.5
    )
    second = recorder.maybe_record(
        _frame(), detection, "blue_square", 35.0, 3, 300, now=1.0
    )

    assert first is not None and first.exists()
    assert skipped is None
    assert second is not None and second.exists()
    assert len(list(recorder.session_dir.glob("*.jpg"))) == 2


def test_debug_record_rejects_non_positive_interval(tmp_path):
    try:
        DebugFrameRecorder(True, str(tmp_path), 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("zero interval must be rejected")
