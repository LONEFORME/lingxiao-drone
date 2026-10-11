"""
calib.py — CyberCAM 焦距标定工具

用法：
  python calib.py                    # 连接摄像头，手动标定
  python calib.py --width 640        # 更低分辨率快速标定

标定原理：
  1. 打印已知尺寸的黑色方块（默认 30×30cm）
  2. 放在相机正下方已知距离（默认 1.0m）
  3. 脚本检测方块在画面中的像素宽度
  4. focal_length_px = pixel_width * distance_m / real_width_m
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np

from detector import SquareDetector


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="CyberCAM focal length calibrator")
    p.add_argument("--width", type=int, default=1920)
    p.add_argument("--height", type=int, default=1080)
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--square-size", type=float, default=0.30,
                   help="Known square size in meters (default 0.30m)")
    p.add_argument("--distance", type=float, default=1.0,
                   help="Measured distance to square in meters (default 1.0m)")
    return p


def main() -> int:
    args = build_parser().parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("[ERR] Cannot open camera")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    detector = SquareDetector(img_w=args.width, img_h=args.height)
    print(f"[CALIB] Place {args.square_size}m black square at {args.distance}m")
    print("[CALIB] Press SPACE to capture a frame, ESC to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        result = detector.detect(frame)
        display = frame.copy()

        if result.found:
            cx = result.dx + detector._cx0
            cy = result.dy + detector._cy0
            cv2.circle(display, (cx, cy), 8, (0, 0, 255), -1)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        if key == 27:  # ESC
            break
        elif key == 32 and result.found:  # SPACE
            # 估算方块像素宽度（从面积反推，假设为正方形）
            area_total = detector.W * detector.H
            r = int(((result.area_ratio * area_total) ** 0.5))
            pixel_width = r * 2
            if pixel_width > 0:
                focal = (
                    pixel_width * args.distance / args.square_size
                )
                print(f"\n=== Calibration Result ===")
                print(f"  Captured @ {args.width}×{args.height}")
                print(f"  Square: {args.square_size}m @ {args.distance}m")
                print(f"  Pixel width: {pixel_width} px")
                print(f"  focal_length_px = {focal:.1f}")
                print(f"==========================\n")
            else:
                print("[CALIB] Square too small, move closer")
        elif key == 32:
            print("[CALIB] No target detected, adjust position")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
