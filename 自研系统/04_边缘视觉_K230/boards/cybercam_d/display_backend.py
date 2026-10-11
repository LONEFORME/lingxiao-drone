"""调试显示后端；飞行模式使用NullDisplay完全跳过显示。"""

from __future__ import annotations


class NullDisplay:
    enabled = False

    def show(self, _frame) -> bool:
        return True

    def close(self) -> None:
        pass


class BuiltinDisplay:
    enabled = True

    def __init__(self, rotation: int = 0) -> None:
        import Display

        self._display = Display
        self._display.init()
        rotations = {
            0: Display.ROTATION_0,
            90: Display.ROTATION_90,
            180: Display.ROTATION_180,
            270: Display.ROTATION_270,
        }
        if rotation not in rotations:
            raise ValueError("屏幕旋转只能是0/90/180/270")
        self._display.set_rotation(rotations[rotation])

    def show(self, frame) -> bool:
        self._display.show(frame)
        return True

    def close(self) -> None:
        self._display.flush()


class OpenCVDisplay:
    enabled = True

    def __init__(self, window_name: str = "cybercam_d") -> None:
        import cv2

        self._cv2 = cv2
        self._window_name = window_name

    def show(self, frame) -> bool:
        self._cv2.imshow(self._window_name, frame)
        return self._cv2.waitKey(1) & 0xFF not in (27, ord("q"))

    def close(self) -> None:
        self._cv2.destroyWindow(self._window_name)


def create_display(mode: str, rotation: int = 0):
    mode = mode.lower()
    if mode == "off":
        return NullDisplay()
    if mode == "builtin":
        return BuiltinDisplay(rotation)
    if mode == "opencv":
        return OpenCVDisplay()
    raise ValueError("display只能是off/builtin/opencv")
