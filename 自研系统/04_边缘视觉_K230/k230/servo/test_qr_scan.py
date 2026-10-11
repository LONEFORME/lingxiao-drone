"""
USB 摄像头二维码扫描 + 货架号映射。
扫码得到 URL → 查 qr_mapping.txt → 显示货架编号 (1~24)。
视频全速显示，QR 解码降频（~1次/秒），OpenCV QRCodeDetector 在 RISC-V 上约 210ms/帧。
"""

import sys
import time
import cv2
import numpy as np
from pathlib import Path

sys.path.insert(0, "/usr/lib/walnutpi/k230_libdisplay/py_lib")
import Display


def load_mapping(mapping_file: str):
    """加载 qr_mapping.txt，返回 content_to_number 字典。"""
    path = Path(mapping_file)
    if not path.exists():
        print(f"⚠️ 映射文件不存在: {path}，将显示原始 URL")
        return {}

    content_to_number = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split("\t", 1)
        if len(parts) != 2:
            continue
        try:
            number = int(parts[0])
            content = parts[1].strip()
            content_to_number[content] = number
        except ValueError:
            continue

    print(f"📋 已加载 {len(content_to_number)} 个货架映射")
    return content_to_number


def draw_status(img: np.ndarray, shelf: str, raw_url: str,
                got_qr: bool, fps: float) -> np.ndarray:
    """在图像上绘制货架号和状态信息。"""
    h, w = img.shape[:2]
    out = img.copy()

    if got_qr and shelf:
        # ── 找到二维码：绿色醒目显示 ──
        overlay = out.copy()
        cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
        out = cv2.addWeighted(overlay, 0.5, out, 0.5, 0)

        cv2.putText(out, f"🏷️  货架 #{shelf}", (16, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(out, f"URL: {raw_url[:48]}", (16, 66),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 255, 180), 1)
    else:
        cv2.putText(out, "扫描中...", (16, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)

    # 右下角 FPS
    cv2.putText(out, f"{fps:.0f} FPS", (w - 80, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return out


def main():
    Display.init()
    print(f"📺 屏幕: {Display.get_width()}x{Display.get_height()}", flush=True)

    # ── 加载映射 ──
    mapping = load_mapping("qr_mapping.txt")

    # ── USB 摄像头 ──
    cap = cv2.VideoCapture("/dev/video5", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    for _ in range(5):
        cap.read()
        time.sleep(0.05)

    qr = cv2.QRCodeDetector()

    # ── 状态变量 ──
    last_url = ""
    display_url = ""
    display_shelf = ""
    got_qr = False
    decode_interval = 1.0
    last_decode_time = 0.0

    frame_count = 0
    fps_timer = time.time()
    current_fps = 0.0

    print("🎥 扫码中（二维码对准摄像头，Ctrl+C 退出）")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        now = time.time()
        frame_count += 1

        # ── FPS 统计 ──
        if now - fps_timer >= 2.0:
            current_fps = frame_count / (now - fps_timer)
            frame_count = 0
            fps_timer = now

        # ── QR 解码（降频，~1次/秒） ──
        if now - last_decode_time >= decode_interval:
            last_decode_time = now
            data, bbox, _ = qr.detectAndDecode(frame)

            if data:
                shelf_number = mapping.get(data, None)
                if shelf_number:
                    display_shelf = str(shelf_number)
                    display_url = data
                    if data != last_url:
                        print(f"✅ 货架 #{shelf_number}  URL: {data}")
                        last_url = data
                    got_qr = True
                else:
                    # 扫到的 URL 不在映射表中，显示原始内容
                    display_shelf = f"?? ({data[:20]})"
                    display_url = data
                    if data != last_url:
                        print(f"❓ 未映射的二维码: {data}")
                        last_url = data
                    got_qr = True
            else:
                if got_qr and now - last_decode_time > 3.0:
                    display_shelf = ""
                    display_url = ""
                    got_qr = False

        # ── 显示 ──
        annotated = draw_status(frame, display_shelf, display_url,
                                got_qr, current_fps)
        Display.show(annotated)


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
