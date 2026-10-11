#!/usr/bin/env python3
"""
激光位置标定工具
================
点亮 BCM19 激光（常亮），拍摄一帧图片，叠加画面中心十字后保存。
通过对比激光光斑与红色十字的位置，量化激光偏移量（单位：像素）。

用法（板子上）：
    cd ~/Desktop/FJJ/warehouse_inventory
    python3 laser_aim_check.py

结果：桌面 ~/Desktop/laser_aim_check.jpg
"""

import sys
import time

PIN = 19                                    # BCM GPIO 引脚（激光）
DEVICE = "/dev/video0"                      # 摄像头设备
WIDTH = 1280
HEIGHT = 720
WARMUP_FRAMES = 20                          # 热身帧数（等曝光稳定）
OUTPUT = "/home/sunrise/Desktop/laser_aim_check.jpg"


def main():
    # ── 1. GPIO 初始化 ────────────────────────────────────────────────────────
    try:
        import Hobot.GPIO as GPIO
    except ImportError as e:
        print(f"[ERROR] Hobot.GPIO 不可用: {e}")
        sys.exit(1)

    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PIN, GPIO.OUT)
        GPIO.output(PIN, GPIO.LOW)   # 初始低电平，确保激光关闭
    except Exception as e:
        print(f"[ERROR] GPIO 初始化失败: {e}")
        sys.exit(1)

    # ── 2. 摄像头初始化 ───────────────────────────────────────────────────────
    try:
        import cv2
    except ImportError:
        GPIO.output(PIN, GPIO.LOW)
        GPIO.cleanup()
        print("[ERROR] cv2 未安装")
        sys.exit(1)

    cap = cv2.VideoCapture(DEVICE)
    if not cap.isOpened():
        GPIO.output(PIN, GPIO.LOW)
        GPIO.cleanup()
        print(f"[ERROR] 无法打开摄像头 {DEVICE}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # ── 3. 点亮激光（常亮）───────────────────────────────────────────────────
    GPIO.output(PIN, GPIO.HIGH)
    print(f"[OK] 激光已点亮 (BCM{PIN} = HIGH)")

    # ── 4. 热身：丢弃前 N 帧等曝光稳定 ───────────────────────────────────────
    print(f"[...] 热身 {WARMUP_FRAMES} 帧，等待曝光稳定...")
    for i in range(WARMUP_FRAMES):
        ret, _ = cap.read()
        if not ret:
            print(f"[WARN] 热身第 {i + 1} 帧读取失败，跳过")

    # ── 5. 抓取目标帧 ─────────────────────────────────────────────────────────
    ret, frame = cap.read()
    cap.release()

    # ── 6. 关闭激光 ──────────────────────────────────────────────────────────
    GPIO.output(PIN, GPIO.LOW)
    GPIO.cleanup()
    print("[OK] 激光已关闭")

    if not ret or frame is None:
        print("[ERROR] 图片抓取失败")
        sys.exit(1)

    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # ── 7. 标注：画面中心红色十字 ─────────────────────────────────────────────
    ARM = 60       # 十字臂长（像素）
    THICK = 2
    RED = (0, 0, 255)

    cv2.line(frame, (cx - ARM, cy), (cx + ARM, cy), RED, THICK)
    cv2.line(frame, (cx, cy - ARM), (cx, cy + ARM), RED, THICK)

    # 中心小圆（方便对比光斑中心）
    cv2.circle(frame, (cx, cy), 6, RED, THICK)

    # 文字标注
    cv2.putText(
        frame,
        f"software center ({cx}, {cy})",
        (cx + 15, cy - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2,
    )

    # ── 8. 保存 ───────────────────────────────────────────────────────────────
    cv2.imwrite(OUTPUT, frame)
    print(f"[OK] 图片已保存: {OUTPUT}  ({w} x {h})")
    print()
    print("=" * 50)
    print("  请观察图片中：")
    print(f"    红色十字中心  = 软件激光点 ({cx}, {cy})")
    print("    激光光斑位置  = 物理激光实际落点")
    print()
    print("  偏移量（估算）：")
    print("    dx = 光斑x - 红色十字x   (正 = 光斑在右)")
    print("    dy = 光斑y - 红色十字y   (正 = 光斑在下)")
    print()
    print("  之后用环境变量补偿（若不物理调正）：")
    print(f"    DRONE_LASER_AIM_X_RATIO = (光斑x) / {w}")
    print(f"    DRONE_LASER_AIM_Y_RATIO = (光斑y) / {h}")
    print("=" * 50)


if __name__ == "__main__":
    main()
