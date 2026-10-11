"""
main.py — CyberCAM 视觉伺服主程序

运行在 CyberCAM（核桃派）板端：
  1. 打开摄像头 1920×1080 @ 30fps
  2. 每帧检测黑色实心方块
  3. 通过 UART 发送 (dx, dy, found) 给 Pi

用法：
  python main.py                    # 默认 UART /dev/ttyS0
  python main.py --port /dev/ttyS1  # 指定串口
  python main.py --preview          # 打开显示窗口（桌面调试用）
"""

from __future__ import annotations

import argparse
import time

import cv2

from detector import SquareDetector, DetResult
from protocol import encode


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="CyberCAM visual servo node")
    p.add_argument("--port", default="/dev/ttyS0", help="UART device")
    p.add_argument(
        "--baud", type=int, default=115200, help="UART baud rate"
    )
    p.add_argument(
        "--camera", type=int, default=0, help="Camera device index"
    )
    p.add_argument(
        "--width", type=int, default=1920, help="Camera capture width",
    )
    p.add_argument(
        "--height", type=int, default=1080, help="Camera capture height",
    )
    p.add_argument(
        "--preview", action="store_true", help="Show preview window",
    )
    p.add_argument(
        "--no-uart", action="store_true",
        help="Run without UART (print to stdout)",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()

    # ── 串口 ──────────────────────────────────────────────────────── #
    uart = None
    if not args.no_uart:
        import serial
        try:
            uart = serial.Serial(
                args.port, args.baud, timeout=0.01,
                write_timeout=0.01,
            )
            print(f"[UART] opened {args.port} @ {args.baud}")
        except Exception as e:
            print(f"[UART] failed: {e}, running no-UART mode")
            args.no_uart = True

    # ── 摄像头 ────────────────────────────────────────────────────── #
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[CAM] cannot open camera {args.camera}")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    # 关闭自动曝光，保持固定参数（防止光照变化导致检测抖动）
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 部分驱动 0.25=off
    print(f"[CAM] opened camera {args.camera} @ {args.width}×{args.height}")

    detector = SquareDetector(img_w=args.width, img_h=args.height)

    frame_count = 0
    fps_t0 = time.monotonic()
    preview = args.preview

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # ── 检测 ────────────────────────────────────────────── #
            result: DetResult = detector.detect(frame)
            payload = encode(result.dx, result.dy, result.found)

            # ── 发送 ────────────────────────────────────────────── #
            if uart is not None and uart.is_open:
                try:
                    uart.write(payload)
                except Exception as e:
                    print(f"[UART] write error: {e}")
            else:
                # no-UART 模式：输出到终端（调试）
                print(payload.decode("ascii").strip())

            # ── 预览 ────────────────────────────────────────────── #
            if preview:
                if result.found:
                    cx = result.dx + detector._cx0
                    cy = result.dy + detector._cy0
                    cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                    r = int((result.area_ratio * detector.W * detector.H) ** 0.5)
                    cv2.rectangle(
                        frame,
                        (cx - r, cy - r), (cx + r, cy + r),
                        (0, 255, 0), 2,
                    )
                cv2.imshow("CyberCAM Visual Servo", frame)
                if cv2.waitKey(1) == ord("q"):
                    break

            # FPS 统计
            frame_count += 1
            now = time.monotonic()
            if now - fps_t0 >= 1.0:
                fps = frame_count / (now - fps_t0)
                print(f"[FPS] {fps:.1f}")
                frame_count = 0
                fps_t0 = now

    except KeyboardInterrupt:
        print("\n[EXIT] user interrupt")
    finally:
        if uart is not None and uart.is_open:
            uart.close()
        cap.release()
        if preview:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
