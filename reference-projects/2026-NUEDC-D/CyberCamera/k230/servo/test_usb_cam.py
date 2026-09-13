"""
CyberCAM USB 摄像头测试：从 /dev/video5 捕获视频并显示到屏幕。

Display 库接受 BGR888 格式（OpenCV 原生），零转换直接显示。
"""

import cv2
import time
import sys

# ── 添加 Display 库路径 ──
sys.path.insert(0, "/usr/lib/walnutpi/k230_libdisplay/py_lib")

import Display


def main():
    # 初始化屏幕
    Display.init()
    print(f"📺 屏幕尺寸: {Display.get_width()}x{Display.get_height()}")

    # 打开 USB 摄像头 (设备节点 /dev/video5)
    cap = cv2.VideoCapture("/dev/video5", cv2.CAP_V4L2)

    if not cap.isOpened():
        print("❌ 无法打开 /dev/video5")
        return

    # 设置 MJPG 格式和分辨率
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✅ USB 摄像头已打开: {actual_w}x{actual_h}")

    # 先读几帧暖身
    for _ in range(5):
        ret, frame = cap.read()
        if ret:
            break
        time.sleep(0.1)

    if not ret:
        print("❌ 无法读取帧")
        cap.release()
        return

    print("🎥 开始实时显示（按 Ctrl+C 退出）...")

    fps_count = 0
    fps_timer = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        # Display 接受 BGR888 (OpenCV 原生格式，零转换)
        Display.show(frame)

        # FPS 统计
        fps_count += 1
        elapsed = time.time() - fps_timer
        if elapsed >= 1.0:
            print(f"   FPS: {fps_count}")
            fps_count = 0
            fps_timer = time.time()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋 用户中断")
    except Exception as e:
        print(f"❌ 异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        Display.flush()
        print("已释放资源")
