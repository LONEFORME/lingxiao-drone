from types import SimpleNamespace

import numpy as np

from Lcode import qr_vision


class Mapping:
    content_to_number = {"near": 11, "far": 20}


def _result(content, left, top, size=80):
    return SimpleNamespace(
        data=content.encode("utf-8"),
        polygon=[
            (left, top),
            (left + size, top),
            (left + size, top + size),
            (left, top + size),
        ],
        rect=None,
    )


def test_target_roi_selects_nearest_qr_not_pyzbar_result_order(monkeypatch):
    # 1000x600 ROI centered at (640,360) → offset=(140,60).
    # local_target in ROI = (500,300).
    monkeypatch.setattr(
        qr_vision,
        "pyzbar_decode",
        lambda _image: [
            _result("far", 700, 260),
            _result("near", 460, 260),    # center (500,300) in ROI = (640,360) full-frame
        ],
    )
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    # "near" center ROI=(500,300) dist=0  ← nearest to local_target(500,300)
    # "far"  center ROI=(740,300) dist=240
    assert detection.number == 11
    assert detection.center == (640.0, 360.0)


def test_raw_profile_decodes_only_original_roi(monkeypatch):
    calls = []

    def decode(image):
        calls.append((image.ndim, image.shape[:2]))
        return []

    monkeypatch.setattr(qr_vision, "pyzbar_decode", decode)
    decoder = qr_vision.QRDecoder(Mapping(), decode_profile="raw")
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    assert decoder.detect(frame, target_point=(640.0, 360.0)) is None
    assert calls == [(3, (600, 1000))]


def test_invalid_decode_profile_is_rejected():
    try:
        qr_vision.QRDecoder(Mapping(), decode_profile="slow")
    except ValueError as exc:
        assert "raw/variants" in str(exc)
    else:
        raise AssertionError("invalid decode profile must fail closed")


def test_adaptive_three_x_variant_maps_polygon_back_to_full_frame(monkeypatch):
    calls = []

    def decode(image):
        calls.append((image.ndim, image.shape[:2]))
        # 1000x600 ROI → 3x adaptiveThreshold → (1800, 3000)
        if image.ndim == 2 and image.shape[:2] == (1800, 3000):
            return [_result("near", 1380, 780, size=240)]
        return []

    monkeypatch.setattr(qr_vision, "pyzbar_decode", decode)
    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    detection = decoder.detect(frame, target_point=(640.0, 360.0))

    assert detection.number == 11
    assert detection.center == (640.0, 360.0)
    assert (2, (1800, 3000)) in calls


def test_pyzbar_failure_in_airborne_path_does_not_run_opencv_fallback(monkeypatch):
    """Target-aware flight decode must remain bounded when pyzbar finds nothing."""
    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    def forbidden(*args, **kwargs):
        raise AssertionError("slow OpenCV fallback must not run in airborne path")

    monkeypatch.setattr(decoder, "_fast_geometry_search", forbidden)
    monkeypatch.setattr(decoder, "_decode_search", forbidden)

    result = decoder.detect(frame, target_point=(640.0, 360.0))

    assert result is None


def test_pyzbar_failure_with_no_geometry_does_not_trigger_decode_search(monkeypatch):
    """When both pyzbar and geometry search fail with a target_point, detect()
    must return None immediately without calling the slow _decode_search."""
    monkeypatch.setattr(qr_vision, "pyzbar_decode", lambda _image: [])

    decoder = qr_vision.QRDecoder(Mapping())
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)

    # Geometry search finds nothing (common when QR not in FOV / overexposed)
    monkeypatch.setattr(decoder, "_fast_geometry_search", lambda *a, **kw: None)

    decode_search_called = []
    monkeypatch.setattr(
        decoder,
        "_decode_search",
        lambda *a, **kw: decode_search_called.append(True) or None,
    )

    result = decoder.detect(frame, target_point=(640.0, 360.0))

    assert result is None
    assert not decode_search_called, "_decode_search must NOT be called in airborne scan loop"
