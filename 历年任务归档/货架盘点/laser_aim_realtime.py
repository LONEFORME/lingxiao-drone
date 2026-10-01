#!/usr/bin/env python3
"""
激光实时标定工具
================
实时显示摄像头画面 + 中心十字标线 + 激光常亮，方便调整物理支架。

用法（板子上，需要图形界面）：
    cd ~/Desktop/FJJ/warehouse_inventory
    python3 laser_aim_realtime.py

    # 如果远程 SSH 需要 X11 转发（板子上开启 X11Forwarding）
    ssh -X ubuntu-pi
    cd ~/Desktop/FJJ/warehouse_inventory
    python3 laser_aim_realtime.py

操作：
    - 画面会显示红色中心十字（软件激光点）
    - 激光会持续点亮
    - 调整物理支架，让激光光斑对准红色十字
    - 按 'q' 或 ESC 退出

注意：
    - 需要 X11 图形环境（cv2.imshow）
    - 激光会持续点亮，注意用眼安全
"""

import sys
import cv2

PIN = 19
DEVICE = "/dev/video0"
WIDTH = 1280
HEIGHT = 720
WINDOW_NAME = "Laser Alignment (press 'q' to quit)"


def main():
    GPIO = None
    cap = None
    
    try:
        # ── 1. GPIO 初始化 ────────────────────────────────────────────
        try:
            import Hobot.GPIO as GPIO_module
            GPIO = GPIO_module
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(PIN, GPIO.OUT)
            GPIO.output(PIN, GPIO.LOW)
            print(f"[OK] GPIO 初始化完成")
        except Exception as e:
            print(f"[ERROR] GPIO 初始化失败: {e}")
            return 1

        # ── 2. 摄像头初始化 ───────────────────────────────────────────
        cap = cv2.VideoCapture(DEVICE)
        if not cap.isOpened():
            print(f"[ERROR] 无法打开摄像头 {DEVICE}")
            return 1

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print(f"[OK] 摄像头已打开: {DEVICE} ({WIDTH}x{HEIGHT})")

        # ── 3. 点亮激光（常亮）────────────────────────────────────────
        GPIO.output(PIN, GPIO.HIGH)
        print(f"[OK] 激光已点亮 (BCM{PIN} = HIGH)")
        print()
        print("=" * 60)
        print("  实时标定窗口已打开")
        print("  - 红色十字 = 软件激光点（画面中心）")
        print("  - 调整物理支架，使激光光斑对准红色十字")
        print("  - 按 'q' 或 ESC 退出")
        print("=" * 60)
        print()

        # ── 4. 实时显示循环 ───────────────────────────────────────────
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 960, 540)  # 缩放窗口便于观察

        frame_count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] 读帧失败，跳过")
                continue

            frame_count += 1
            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2

            # 绘制中心十字（红色，粗线）
            ARM = 80
            THICK = 3
            RED = (0, 0, 255)

            cv2.line(frame, (cx - ARM, cy), (cx + ARM, cy), RED, THICK)
            cv2.line(frame, (cx, cy - ARM), (cx, cy + ARM), RED, THICK)
            cv2.circle(frame, (cx, cy), 8, RED, THICK)

            # 文字标注（画面中心坐标 + 帧数）
            cv2.putText(
                frame,
                f"Center ({cx}, {cy})  Frame {frame_count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, RED, 2,
            )

            # 提示文字（底部）
            cv2.putText(
                frame,
                "Press 'q' or ESC to quit",
                (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )

            cv2.imshow(WINDOW_NAME, frame)

            # 等待按键（1ms 刷新间隔）
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' 或 ESC
                print("\n[OK] 用户退出")
                break

    except KeyboardInterrupt:
        print("\n[OK] Ctrl+C 退出")
    except Exception as e:
        print(f"\n[ERROR] 异常: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # ── 5. 清理资源 ──────────────────────────────────────────────
        if cap is not None:
            cap.release()
            print("[OK] 摄像头已关闭")

        if GPIO is not None:
            GPIO.output(PIN, GPIO.LOW)
            GPIO.cleanup()
            print("[OK] 激光已关闭，GPIO 已清理")

        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
