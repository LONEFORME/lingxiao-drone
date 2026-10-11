"""Cyber Camera当前启用的外接CSI0摄像头与UVC的统一采集接口。"""

from __future__ import annotations

import cv2


class WalnutPiCSICapture:
    """包装当前设备树选中的CSI0相机，提供OpenCV风格接口。"""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        *,
        hmirror: bool = False,
        vflip: bool = False,
        sensor_factory=None,
    ) -> None:
        if sensor_factory is None:
            from walnutpi import Sensor
            sensor_factory = Sensor.Sensor
        self._sensor = sensor_factory(int(width), int(height))
        if hmirror:
            self._sensor.set_hmirror(1)
        if vflip:
            self._sensor.set_vflip(1)

    def isOpened(self) -> bool:
        return bool(self._sensor.isOpened())

    def read(self):
        return self._sensor.read()

    def release(self) -> None:
        release = getattr(self._sensor, "release", None)
        if callable(release):
            release()


class OpenCVCapture:
    def __init__(self, source=0, width: int = 640, height: int = 480, capture_factory=None) -> None:
        factory = capture_factory or cv2.VideoCapture
        self._capture = factory(source)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

    def isOpened(self) -> bool:
        return bool(self._capture.isOpened())

    def read(self):
        return self._capture.read()

    def release(self) -> None:
        self._capture.release()


def create_capture(
    backend: str,
    source=0,
    width: int = 640,
    height: int = 480,
    *,
    hmirror: bool = False,
    vflip: bool = False,
):
    backend = backend.lower()
    if backend == "csi":
        return WalnutPiCSICapture(
            width, height, hmirror=hmirror, vflip=vflip
        )
    if backend == "opencv":
        return OpenCVCapture(source, width, height)
    if backend != "auto":
        raise ValueError("backend只能是auto/csi/opencv")
    try:
        capture = WalnutPiCSICapture(
            width, height, hmirror=hmirror, vflip=vflip
        )
        if capture.isOpened():
            return capture
        capture.release()
    except (ImportError, OSError, RuntimeError):
        pass
    return OpenCVCapture(source, width, height)
