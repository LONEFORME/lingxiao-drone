#!/usr/bin/env python3
"""
ROI 标注工具 — 在视觉调试图片上叠加解码区域和检测结果
======================================================

用法：
    # 标注 vision_debug 目录下的所有图片
    python annotate_roi.py --input <vision_debug_目录> --output <输出目录>

    # 单张图片
    python annotate_roi.py --input image.jpg --output out.png

    # 覆盖默认 ROI 尺寸（默认 560x600，中心 640,360）
    python annotate_roi.py --input <目录> --roi-w 560 --roi-h 600 --cx 640 --cy 360

输出：
    - 每张图片输出为 PNG 标注图（在原目录下创建 _annotated/ 子目录）
    - ROI 用绿色框标出，排除区半透暗化
    - 检测到的 QR 码用蓝色多边形和编号标注
"""

import argparse
import json
import os
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    print("[ERROR] 需要 opencv-python: pip install opencv-python")
    sys.exit(1)


def annotate_image(img, roi_w, roi_h, cx, cy, qr_data=None):
    """
    在单帧上叠加 ROI 标注。

    参数:
        img: OpenCV BGR 图像
        roi_w, roi_h: ROI 宽度和高度（像素）
        cx, cy: 激光点/画面中心坐标
        qr_data: list of dict, 每项 {"corners": [(x,y),...], "number": int, "content": str}

    返回:
        标注后的图像副本
    """
    h, w = img.shape[:2]
    rx1 = max(0, cx - roi_w // 2)
    ry1 = max(0, cy - roi_h // 2)
    rx2 = min(w, rx1 + roi_w)
    ry2 = min(h, ry1 + roi_h)
    # 重新计算实际尺寸（防止边缘溢出）
    actual_roi_w = rx2 - rx1
    actual_roi_h = ry2 - ry1

    out = img.copy()

    # ROI 外部暗化
    overlay = out.copy()
    overlay[:, :rx1] = (overlay[:, :rx1] * 0.35).astype(np.uint8)
    overlay[:, rx2:] = (overlay[:, rx2:] * 0.35).astype(np.uint8)
    overlay[:ry1, rx1:rx2] = (overlay[:ry1, rx1:rx2] * 0.35).astype(np.uint8)
    overlay[ry2:, rx1:rx2] = (overlay[ry2:, rx1:rx2] * 0.35).astype(np.uint8)
    out = overlay

    # ROI 边框（绿色）
    cv2.rectangle(out, (rx1, ry1), (rx2, ry2), (0, 230, 100), 2)
    cv2.putText(out, f"Decode ROI {actual_roi_w}x{actual_roi_h}",
                (rx1 + 5, ry1 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 230, 100), 2)
    cv2.putText(out, f"x[{rx1},{rx2}] y[{ry1},{ry2}]",
                (rx1 + 5, ry1 + actual_roi_h + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 80), 2)

    # 中心十字（红色）
    cv2.line(out, (cx - 60, cy), (cx + 60, cy), (0, 0, 255), 2)
    cv2.line(out, (cx, cy - 60), (cx, cy + 60), (0, 0, 255), 2)
    cv2.circle(out, (cx, cy), 8, (0, 0, 255), 2)
    cv2.putText(out, f"laser ({cx},{cy})", (cx + 15, cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # 排除区标注
    cv2.putText(out, f"EXCLUDED {rx1}px", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
    cv2.putText(out, f"EXCLUDED {w - rx2}px", (w - 160, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

    # 顶部排除区高度
    cv2.putText(out, f"EXCLUDED {ry1}px", (cx - 60, ry1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

    # 底部排除区
    if ry2 < h - 2:
        cv2.putText(out, f"EXCLUDED {h - ry2}px", (cx - 60, ry2 + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

    # QR 检测结果（如果有）
    if qr_data:
        for qr in qr_data:
            if "corners" in qr and qr["corners"]:
                pts = np.array(qr["corners"], dtype=np.int32)
                cv2.polylines(out, [pts], True, (255, 180, 0), 2)
                cy_qr = int(sum(p[1] for p in qr["corners"]) / 4)
                cx_qr = int(sum(p[0] for p in qr["corners"]) / 4)
                number = qr.get("number", "?")
                cv2.putText(out, f"QR#{number}", (cx_qr + 5, cy_qr - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 180, 0), 2)

    return out


def detect_qrs(img):
    """用 pyzbar (+ OpenCV 回退) 检测图中的 QR 码，返回坐标和内容。"""
    results = []
    # pyzbar
    try:
        from pyzbar.pyzbar import decode as pyzbar_decode
        detections = pyzbar_decode(img)
        for d in detections:
            if d.polygon:
                pts = [(p.x, p.y) for p in d.polygon]
                raw = d.data.decode("utf-8", errors="replace").strip()
                results.append({"corners": pts, "content": raw, "number": "?"})
            elif d.rect:
                x, y, w, h = d.rect
                pts = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                results.append({"corners": pts, "content": d.data, "number": "?"})
    except ImportError:
        pass

    # OpenCV 回退
    if not results:
        detector = cv2.QRCodeDetector()
        ret, points, decoded = detector.detectAndDecode(img)
        if ret and points is not None and len(points) > 0:
            pts = [(float(p[0]), float(p[1])) for p in points[0]]
            results.append({"corners": pts, "content": str(decoded or ""), "number": "?"})

    return results


def process_single(img_path, output_path, roi_w, roi_h, cx, cy, detect=True):
    """处理单张图片。"""
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"  [SKIP] 无法读取: {img_path}")
        return False

    qr_data = detect_qrs(img) if detect else None
    annotated = annotate_image(img, roi_w, roi_h, cx, cy, qr_data)

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    cv2.imwrite(str(output_path), annotated)

    info = f"  -> {output_path}"
    if qr_data:
        info += f"  [{len(qr_data)} QR]"
    print(info)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="在视觉调试图片上叠加解码ROI和检测结果",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--input", "-i", required=True,
                        help="输入图片或目录（vision_debug 目录）")
    parser.add_argument("--output", "-o", default=None,
                        help="输出文件或目录（默认: <input>_annotated/）")
    parser.add_argument("--roi-w", type=int, default=560,
                        help="ROI 宽度 (默认: 560)")
    parser.add_argument("--roi-h", type=int, default=600,
                        help="ROI 高度 (默认: 600)")
    parser.add_argument("--cx", type=int, default=640,
                        help="中心x (默认: 640)")
    parser.add_argument("--cy", type=int, default=360,
                        help="中心y (默认: 360)")
    parser.add_argument("--no-detect", action="store_true",
                        help="不进行 QR 检测（只画 ROI）")
    args = parser.parse_args()

    input_path = Path(args.input)

    # 单文件
    if input_path.is_file():
        out = args.output or (input_path.stem + "_annotated.png")
        process_single(input_path, out, args.roi_w, args.roi_h,
                       args.cx, args.cy, detect=not args.no_detect)
        return

    # 目录
    if not input_path.is_dir():
        print(f"[ERROR] 输入不存在: {input_path}")
        sys.exit(1)

    output_root = Path(args.output) if args.output else (
        input_path.parent / (input_path.name + "_annotated")
    )
    output_root.mkdir(parents=True, exist_ok=True)

    # 收集图片文件
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"}
    images = sorted([p for p in input_path.iterdir() if p.suffix.lower() in exts])
    if not images:
        print(f"[ERROR] {input_path} 中未找到图片文件")
        sys.exit(1)

    print(f"共 {len(images)} 张图片，处理中...")
    for img_path in images:
        out_path = output_root / (img_path.stem + "_annotated.png")
        process_single(img_path, out_path, args.roi_w, args.roi_h,
                       args.cx, args.cy, detect=not args.no_detect)

    print(f"\n完成！结果在: {output_root}")
    print("图例: 绿色框=ROI 红色十字=激光点 灰色=排除区 蓝色=QR检测")


if __name__ == "__main__":
    main()
