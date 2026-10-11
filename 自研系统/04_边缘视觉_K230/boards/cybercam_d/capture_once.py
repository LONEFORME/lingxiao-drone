"""外接CSI0单帧采集工具，用于检测前确认画面、方向和曝光。"""

from __future__ import annotations

import argparse

import cv2

try:
    from .camera_backend import create_capture
except ImportError:
    from camera_backend import create_capture


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("csi", "opencv"), default="csi")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    source = int(args.camera) if str(args.camera).isdigit() else args.camera
    capture = create_capture(args.backend, source, args.width, args.height)
    try:
        if not capture.isOpened():
            raise RuntimeError("摄像头未打开")
        for _ in range(max(0, args.warmup_frames)):
            capture.read()
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError("读取单帧失败")
        if not cv2.imwrite(args.output, frame):
            raise RuntimeError("保存单帧失败")
        print(f"saved={args.output} shape={frame.shape}")
    finally:
        capture.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
