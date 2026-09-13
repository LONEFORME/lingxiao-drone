"""
QR 视觉伺服：首次解码后提取 QR 内部 50+ 特征点，光流跟踪整体偏移。

策略：
  第1帧: detectAndDecode → 货架号 + QR 四角坐标 + 精确中心
  之后:  提取 QR 区域内 Shi-Tomasi 特征点 → 光流跟踪 → 中位数偏移 → 更新中心
  每30帧: 重新 decode 校正累计漂移

比跟踪4个角点的方案更鲁棒：50个特征点互相校验，单点丢失不影响整体。
"""

import sys
import time
import cv2
import numpy as np
from pathlib import Path

sys.path.insert(0, "/usr/lib/walnutpi/k230_libdisplay/py_lib")
import Display


# ── 参数 ──
REFINE_INTERVAL = 30    # 每 N 帧重新 decode 校正漂移
MAX_FEATURES = 60       # 最大特征点数
MIN_FEATURES = 8        # 最少有效特征点数
QUALITY_LEVEL = 0.02    # Shi-Tomasi 质量阈值
MIN_DISTANCE = 5        # 特征点最小间距
LK_WIN_SIZE = (21, 21)
LK_MAX_LEVEL = 3
MAX_LOST = 30           # 彻底丢失阈值（帧数）


def load_mapping(mapping_file: str):
    path = Path(mapping_file)
    if not path.exists():
        print("⚠️ 无映射文件")
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
            content_to_number[parts[1].strip()] = int(parts[0])
        except ValueError:
            continue
    print(f"📋 映射: {len(content_to_number)} 个货架")
    return content_to_number


def extract_features(gray, corners):
    """在 QR 四边形区域内提取 Shi-Tomasi 特征点。"""
    mask = np.zeros_like(gray, dtype=np.uint8)
    cv2.fillPoly(mask, [corners.astype(np.int32)], 255)
    features = cv2.goodFeaturesToTrack(
        gray, maxCorners=MAX_FEATURES,
        qualityLevel=QUALITY_LEVEL,
        minDistance=MIN_DISTANCE,
        mask=mask,
    )
    if features is None:
        return None
    return features.reshape(-1, 2).astype(np.float32)


def draw_servo(img, corners, features, area, shelf, fps, dx, dy):
    """绘制视觉伺服状态。"""
    h, w = img.shape[:2]
    out = img.copy()
    cx_img, cy_img = w // 2, h // 2

    if corners is not None and len(corners) == 4:
        # ── QR 四边形 ──
        pts = corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(out, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # ── QR 中心十字 ──
        cx, cy = float(corners[:, 0].mean()), float(corners[:, 1].mean())
        cv2.line(out, (int(cx) - 18, int(cy)), (int(cx) + 18, int(cy)),
                 (0, 255, 0), 2)
        cv2.line(out, (int(cx), int(cy) - 18), (int(cx), int(cy) + 18),
                 (0, 255, 0), 2)
        cv2.line(out, (cx_img, cy_img), (int(cx), int(cy)),
                 (0, 255, 255), 1)

        # ── 跟踪的特征点 ──
        if features is not None:
            for p in features:
                cv2.circle(out, (int(p[0]), int(p[1])), 2, (0, 180, 255), -1)

        # ── 顶栏 ──
        overlay = out.copy()
        cv2.rectangle(overlay, (0, 0), (w, 90), (0, 0, 0), -1)
        out = cv2.addWeighted(overlay, 0.5, out, 0.5, 0)

        cv2.putText(out, f"🏷️  #{shelf}  dx={dx:+d}  dy={dy:+d}",
                    (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.putText(out, f"area={area:.0f}px  ftr={len(features) if features is not None else 0}  {fps:.0f}FPS",
                    (16, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 255, 180), 1)
    else:
        cv2.putText(out, "扫描中...", (16, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)

    cv2.putText(out, f"{fps:.0f} FPS", (w - 80, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    return out


def main():
    Display.init()
    screen_w, screen_h = Display.get_size()
    print(f"📺 屏幕: {screen_w}x{screen_h}")

    mapping = load_mapping("qr_mapping.txt")

    cap = cv2.VideoCapture("/dev/video5", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    for _ in range(10):
        cap.read()
        time.sleep(0.03)

    qr = cv2.QRCodeDetector()

    # ── 状态 ──
    shelf = "?"
    ref_corners = None       # 参考四角 (来自最近一次 decode)
    ref_features = None      # 参考帧特征点
    ref_gray = None          # 参考帧灰度图
    qr_corners = None        # 当前四角（显示用）
    cur_features = None      # 当前特征点（显示用）
    area = 0.0
    state = "SEARCH"
    lost_count = 0
    frame_since_refine = 0

    # 用于伺服输出的平滑值
    smooth_dx = 0.0
    smooth_dy = 0.0

    frame_count = 0
    fps_timer = time.time()
    current_fps = 0.0

    print("🎥 QR 视觉伺服（50+特征点跟踪偏移）")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        now = time.time()
        frame_count += 1
        frame_since_refine += 1

        if now - fps_timer >= 2.0:
            current_fps = frame_count / (now - fps_timer)
            frame_count = 0
            fps_timer = now

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h_img, w_img = gray.shape
        cx_img, cy_img = w_img // 2, h_img // 2
        display_dx = 0
        display_dy = 0

        # ── 策略 ──
        if state == "SEARCH":
            # ── 全帧 QR detectAndDecode ──
            data, bbox, _ = qr.detectAndDecode(frame)

            if data and bbox is not None and len(bbox) > 0:
                shelf_num = mapping.get(data, None)
                shelf = str(shelf_num) if shelf_num else f"??{data[:12]}"

                ref_corners = bbox[0].astype(np.float32)
                qr_corners = ref_corners.copy()
                area = cv2.contourArea(bbox[0])
                ref_gray = gray.copy()
                frame_since_refine = 0

                # 提取 QR 内部特征点
                ref_features = extract_features(gray, ref_corners)
                cur_features = ref_features.copy() if ref_features is not None else None
                lost_count = 0
                state = "TRACK"

                cx = float(ref_corners[:, 0].mean())
                cy = float(ref_corners[:, 1].mean())
                dx = int(cx - cx_img)
                dy = int(cy - cy_img)
                print(f"✅ 货架 #{shelf}  dx={dx:+d} dy={dy:+d}"
                      f"  area={area:.0f}px"
                      f"  features={len(ref_features) if ref_features is not None else 0}",
                      flush=True)

        else:  # TRACK
            # ── 光流跟踪特征点 ──
            track_ok = False

            if (ref_features is not None and ref_gray is not None
                    and len(ref_features) >= MIN_FEATURES):

                next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    ref_gray, gray, ref_features, None,
                    winSize=LK_WIN_SIZE, maxLevel=LK_MAX_LEVEL)

                good = next_pts[status.flatten() == 1]

                if len(good) >= MIN_FEATURES:
                    # 计算整体偏移（中位数，抗离群点）
                    tracked = good.reshape(-1, 2).astype(np.float32)
                    delta = tracked - ref_features[status.flatten() == 1]
                    dx_offset = float(np.median(delta[:, 0]))
                    dy_offset = float(np.median(delta[:, 1]))

                    # 更新参考特征点到当前帧（为下一帧做准备）
                    ref_features = tracked.copy()
                    cur_features = tracked.copy()
                    ref_gray = gray.copy()

                    # 更新 QR 四角 = 参考四角 + 整体偏移
                    qr_corners = ref_corners.copy()
                    qr_corners[:, 0] += dx_offset
                    qr_corners[:, 1] += dy_offset

                    # 面积用当前四角算
                    if len(qr_corners) >= 3:
                        area = cv2.contourArea(qr_corners.reshape(-1, 1, 2))

                    lost_count = 0
                    track_ok = True

                    # 伺服输出
                    cx = float(qr_corners[:, 0].mean())
                    cy = float(qr_corners[:, 1].mean())
                    display_dx = int(cx - cx_img)
                    display_dy = int(cy - cy_img)

                    # 平滑
                    smooth_dx = smooth_dx * 0.7 + display_dx * 0.3
                    smooth_dy = smooth_dy * 0.7 + display_dy * 0.3

            if not track_ok:
                lost_count += 1
                if lost_count > MAX_LOST:
                    state = "SEARCH"
                    qr_corners = None
                    ref_features = None
                    print("👋 QR 丢失，重新搜索", flush=True)

            # ── 周期性 refine ──
            if track_ok and frame_since_refine >= REFINE_INTERVAL:
                frame_since_refine = 0
                data, bbox, _ = qr.detectAndDecode(frame)
                if data and bbox is not None and len(bbox) > 0:
                    # 用新的检测结果校正参考角点
                    ref_corners = bbox[0].astype(np.float32)
                    qr_corners = ref_corners.copy()
                    area = cv2.contourArea(bbox[0])
                    ref_gray = gray.copy()
                    ref_features = extract_features(gray, ref_corners)
                    cur_features = ref_features.copy() if ref_features is not None else None
                    print(f"🔄 refine  dx={display_dx:+d}", flush=True)

        # ── 显示 ──
        annotated = draw_servo(frame, qr_corners, cur_features, area,
                               shelf, current_fps, display_dx, display_dy)
        Display.show(annotated)

        # ── 控制量输出 ──
        if qr_corners is not None and len(qr_corners) == 4:
            cx = float(qr_corners[:, 0].mean())
            cy = float(qr_corners[:, 1].mean())
            dx = cx - cx_img
            dy = cy - cy_img
            # UART: uart.write(f"V{shelf},{dx:+d},{dy:+d}\n".encode())


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
