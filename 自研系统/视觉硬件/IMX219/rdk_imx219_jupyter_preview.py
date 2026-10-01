import json
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    import ipywidgets as widgets
    from IPython.display import display
except ImportError:
    widgets = None
    display = None


DEVICE = "/dev/video6"
SENSOR_SUBDEVICE = "/dev/v4l-subdev1"
PRODUCER = "/app/cdev_demo/v4l2/rdk_imx219_stream"

RAW_WIDTH = 1920
RAW_HEIGHT = 1080
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 540
FRAME_BYTES = RAW_WIDTH * RAW_HEIGHT * 3 // 2
JPEG_QUALITY = 65

__all__ = [
    "VisionSystem",
    "IMX219VisionSystem",
    "run_preview",
    "capture_frame",
    "set_camera_controls",
    "apply_color_correction",
    "apply_fast_denoise",
    "draw_color_detections",
    "draw_red_square_detections",
]


def _require_jupyter_display():
    if widgets is None or display is None:
        raise RuntimeError(
            "Jupyter preview requires ipywidgets and IPython; "
            "camera capture and detection can run without them"
        )


def set_camera_controls(exposure=3000, analogue_gain=0, vertical_blanking=2446):
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            SENSOR_SUBDEVICE,
            f"--set-ctrl=vertical_blanking={vertical_blanking}",
        ],
        check=True,
    )
    subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            SENSOR_SUBDEVICE,
            f"--set-ctrl=exposure={exposure},analogue_gain={analogue_gain}",
        ],
        check=True,
    )


def read_exact(stream, size):
    data = bytearray(size)
    view = memoryview(data)
    offset = 0

    while offset < size:
        read_count = stream.readinto(view[offset:])
        if not read_count:
            raise EOFError("The V4L2 producer stopped before a complete frame arrived")
        offset += read_count

    return data


def apply_color_correction(
    image,
    blue_gain,
    green_gain,
    red_gain,
    saturation,
):
    corrected = image.astype(np.float32)
    corrected[:, :, 0] *= blue_gain
    corrected[:, :, 1] *= green_gain
    corrected[:, :, 2] *= red_gain
    corrected = np.clip(corrected, 0, 255).astype(np.uint8)

    hsv = cv2.cvtColor(corrected, cv2.COLOR_BGR2HSV)
    hsv[:, :, 1] = np.clip(
        hsv[:, :, 1].astype(np.float32) * saturation,
        0,
        255,
    ).astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def apply_fast_denoise(image, luma_kernel, chroma_kernel):
    ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    luma, red_chroma, blue_chroma = cv2.split(ycrcb)

    if luma_kernel >= 3:
        luma = cv2.GaussianBlur(luma, (luma_kernel, luma_kernel), 0)
    if chroma_kernel >= 3:
        red_chroma = cv2.GaussianBlur(
            red_chroma,
            (chroma_kernel, chroma_kernel),
            0,
        )
        blue_chroma = cv2.GaussianBlur(
            blue_chroma,
            (chroma_kernel, chroma_kernel),
            0,
        )

    return cv2.cvtColor(
        cv2.merge((luma, red_chroma, blue_chroma)),
        cv2.COLOR_YCrCb2BGR,
    )


def draw_color_detections(image, minimum_area=600):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    color_definitions = [
        (
            "RED",
            [((0, 65, 45), (12, 255, 255)), ((165, 65, 45), (179, 255, 255))],
            (40, 40, 230),
        ),
        ("YELLOW", [((15, 65, 55), (38, 255, 255))], (20, 210, 240)),
        ("GREEN", [((40, 55, 40), (90, 255, 255))], (50, 200, 70)),
        ("BLUE", [((90, 60, 40), (135, 255, 255))], (230, 120, 40)),
    ]
    detections = []

    for color_name, ranges, box_color in color_definitions:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                ),
            )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < minimum_area:
                continue

            left, top, width, height = cv2.boundingRect(contour)
            detections.append(
                {
                    "label": color_name,
                    "area": int(area),
                    "box": (left, top, width, height),
                }
            )
            cv2.rectangle(
                image,
                (left, top),
                (left + width, top + height),
                box_color,
                2,
            )
            label = f"{color_name} {int(area)}"
            text_size, baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                1,
            )
            label_top = max(0, top - text_size[1] - baseline - 6)
            cv2.rectangle(
                image,
                (left, label_top),
                (left + text_size[0] + 8, label_top + text_size[1] + baseline + 6),
                box_color,
                -1,
            )
            cv2.putText(
                image,
                label,
                (left + 4, label_top + text_size[1] + 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    return detections


def _corner_cosine(point_before, vertex, point_after):
    first_vector = point_before.astype(np.float32) - vertex.astype(np.float32)
    second_vector = point_after.astype(np.float32) - vertex.astype(np.float32)
    denominator = np.linalg.norm(first_vector) * np.linalg.norm(second_vector)
    if denominator < 1e-6:
        return 1.0
    return float(np.dot(first_vector, second_vector) / denominator)


def draw_red_square_detections(
    image,
    minimum_area=600,
    maximum_side_ratio=1.35,
    maximum_corner_cosine=0.45,
):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red = cv2.inRange(
        hsv,
        np.array((0, 65, 45), dtype=np.uint8),
        np.array((12, 255, 255), dtype=np.uint8),
    )
    upper_red = cv2.inRange(
        hsv,
        np.array((165, 65, 45), dtype=np.uint8),
        np.array((179, 255, 255), dtype=np.uint8),
    )
    mask = cv2.bitwise_or(lower_red, upper_red)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    detections = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < minimum_area:
            continue

        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            continue

        points = polygon.reshape(4, 2)
        corner_cosines = [
            abs(
                _corner_cosine(
                    points[(index - 1) % 4],
                    points[index],
                    points[(index + 1) % 4],
                )
            )
            for index in range(4)
        ]
        if max(corner_cosines) > maximum_corner_cosine:
            continue

        _, (rotated_width, rotated_height), _ = cv2.minAreaRect(contour)
        shorter_side = min(rotated_width, rotated_height)
        longer_side = max(rotated_width, rotated_height)
        if shorter_side < 1 or longer_side / shorter_side > maximum_side_ratio:
            continue

        rectangle_area = rotated_width * rotated_height
        if rectangle_area <= 0 or area / rectangle_area < 0.65:
            continue

        left, top, width, height = cv2.boundingRect(polygon)
        center_x = left + width // 2
        center_y = top + height // 2
        detections.append(
            {
                "label": "RED_SQUARE",
                "area": int(area),
                "center": (center_x, center_y),
                "box": (left, top, width, height),
                "side_ratio": round(float(longer_side / shorter_side), 3),
            }
        )

        cv2.polylines(image, [polygon], True, (0, 255, 0), 3)
        cv2.circle(image, (center_x, center_y), 5, (0, 255, 255), -1)
        label = f"RED SQUARE ({center_x},{center_y})"
        label_y = max(24, top - 8)
        cv2.putText(
            image,
            label,
            (left, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return detections


def run_preview(
    exposure=3000,
    analogue_gain=0,
    vertical_blanking=2446,
    device=DEVICE,
    raw_width=RAW_WIDTH,
    raw_height=RAW_HEIGHT,
    display_width=DISPLAY_WIDTH,
    display_height=DISPLAY_HEIGHT,
    blue_gain=1.0,
    green_gain=1.0,
    red_gain=1.0,
    saturation=1.0,
    luma_denoise=0,
    chroma_denoise=0,
    temporal_denoise=0.0,
    detect_colors=False,
    minimum_color_area=600,
    detect_red_squares=False,
    minimum_square_area=600,
):
    _require_jupyter_display()
    frame_bytes = raw_width * raw_height * 3 // 2
    set_camera_controls(
        exposure=exposure,
        analogue_gain=analogue_gain,
        vertical_blanking=vertical_blanking,
    )

    blank = np.zeros((display_height, display_width, 3), dtype=np.uint8)
    encoded, blank_jpeg = cv2.imencode(".jpg", blank)
    if not encoded:
        raise RuntimeError("Failed to create the initial JPEG image")

    image_widget = widgets.Image(
        value=blank_jpeg.tobytes(),
        format="jpeg",
        layout=widgets.Layout(
            width=f"{display_width}px",
            height=f"{display_height}px",
        ),
    )
    status = widgets.Label(value="Starting continuous V4L2 preview...")
    display(status, image_widget)

    producer = subprocess.Popen(
        [PRODUCER, device, str(raw_width), str(raw_height)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )

    frame_count = 0
    start_time = time.time()
    previous_preview = None

    try:
        while True:
            frame_data = read_exact(producer.stdout, frame_bytes)
            nv12 = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (raw_height * 3 // 2, raw_width)
            )
            bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
            preview = cv2.resize(
                bgr,
                (display_width, display_height),
                interpolation=cv2.INTER_AREA,
            )
            if luma_denoise >= 3 or chroma_denoise >= 3:
                preview = apply_fast_denoise(
                    preview,
                    luma_kernel=luma_denoise,
                    chroma_kernel=chroma_denoise,
                )
            if (
                blue_gain != 1.0
                or green_gain != 1.0
                or red_gain != 1.0
                or saturation != 1.0
            ):
                preview = apply_color_correction(
                    preview,
                    blue_gain=blue_gain,
                    green_gain=green_gain,
                    red_gain=red_gain,
                    saturation=saturation,
                )

            if previous_preview is not None and temporal_denoise > 0:
                preview = cv2.addWeighted(
                    preview,
                    1.0 - temporal_denoise,
                    previous_preview,
                    temporal_denoise,
                    0,
                )
            previous_preview = preview.copy()

            detections = []
            if detect_colors:
                detections = draw_color_detections(
                    preview,
                    minimum_area=minimum_color_area,
                )
            if detect_red_squares:
                detections.extend(
                    draw_red_square_detections(
                        preview,
                        minimum_area=minimum_square_area,
                    )
                )

            encoded, jpeg = cv2.imencode(
                ".jpg",
                preview,
                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
            )
            if not encoded:
                continue

            image_widget.value = jpeg.tobytes()
            frame_count += 1

            if frame_count % 10 == 0:
                elapsed = max(time.time() - start_time, 1e-6)
                mean_y = float(nv12[:raw_height, :].mean())
                detection_text = ""
                if detect_colors or detect_red_squares:
                    counts = {}
                    for detection in detections:
                        label = detection["label"]
                        counts[label] = counts.get(label, 0) + 1
                    detection_text = f", detections={counts}"
                status.value = (
                    f"frames={frame_count}, preview={frame_count / elapsed:.1f} fps, "
                    f"meanY={mean_y:.1f}, exposure={exposure}, gain={analogue_gain}, "
                    f"BGR gains={blue_gain:.2f}/{green_gain:.2f}/{red_gain:.2f}"
                    f"{detection_text}"
                )

    except KeyboardInterrupt:
        status.value = "Stopped by user"
    except EOFError as error:
        producer_error = producer.stderr.read().decode("utf-8", errors="replace")
        status.value = str(error)
        if producer_error:
            print(producer_error)
    finally:
        if producer.poll() is None:
            producer.terminate()
            try:
                producer.wait(timeout=3)
            except subprocess.TimeoutExpired:
                producer.kill()
                producer.wait()

        status.value += " | camera closed"


def _query_camera_controls():
    result = subprocess.run(
        [
            "v4l2-ctl",
            "-d",
            SENSOR_SUBDEVICE,
            "--get-ctrl=vertical_blanking,exposure,analogue_gain,test_pattern,"
            "horizontal_flip,vertical_flip",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    values = {}
    for line in result.stdout.splitlines():
        name, separator, value = line.partition(":")
        if not separator:
            continue
        value = value.strip()
        try:
            values[name.strip()] = int(value.split()[0])
        except (ValueError, IndexError):
            values[name.strip()] = value
    return values, result.stdout.strip()


def _query_video_format(device):
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, "--get-fmt-video"],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        return result.stdout.strip()
    return result.stderr.strip()


def _summarize_bgr(image):
    height, width = image.shape[:2]
    center = image[
        height // 3 : height * 2 // 3,
        width // 3 : width * 2 // 3,
    ]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blue_mean, green_mean, red_mean = cv2.mean(image)[:3]
    center_blue, center_green, center_red = cv2.mean(center)[:3]

    return {
        "bgr_mean": {
            "blue": round(float(blue_mean), 3),
            "green": round(float(green_mean), 3),
            "red": round(float(red_mean), 3),
        },
        "center_bgr_mean": {
            "blue": round(float(center_blue), 3),
            "green": round(float(center_green), 3),
            "red": round(float(center_red), 3),
        },
        "center_bgr_stddev": {
            "blue": round(float(center[:, :, 0].std()), 3),
            "green": round(float(center[:, :, 1].std()), 3),
            "red": round(float(center[:, :, 2].std()), 3),
        },
        "saturation": {
            "mean": round(float(hsv[:, :, 1].mean()), 3),
            "median": round(float(np.percentile(hsv[:, :, 1], 50)), 3),
            "p95": round(float(np.percentile(hsv[:, :, 1], 95)), 3),
        },
        "focus_laplacian_variance": round(
            float(cv2.Laplacian(gray, cv2.CV_64F).var()),
            3,
        ),
    }


def _summarize_luma(luma):
    return {
        "mean": round(float(luma.mean()), 3),
        "stddev": round(float(luma.std()), 3),
        "minimum": int(luma.min()),
        "p01": round(float(np.percentile(luma, 1)), 3),
        "median": round(float(np.percentile(luma, 50)), 3),
        "p99": round(float(np.percentile(luma, 99)), 3),
        "maximum": int(luma.max()),
        "limited_range_shadow_percent": round(
            float(np.mean(luma <= 16) * 100.0),
            4,
        ),
        "limited_range_highlight_percent": round(
            float(np.mean(luma >= 235) * 100.0),
            4,
        ),
    }


def capture_frame(
    exposure=2645,
    analogue_gain=0,
    vertical_blanking=2446,
    device="/dev/video10",
    raw_width=960,
    raw_height=540,
    blue_gain=0.95,
    green_gain=1.06,
    red_gain=1.00,
    saturation=1.25,
    skip_frames=8,
    output_dir="/home/sunrise/imx219-captures",
    frame_name=None,
):
    frame_bytes = raw_width * raw_height * 3 // 2
    set_camera_controls(
        exposure=exposure,
        analogue_gain=analogue_gain,
        vertical_blanking=vertical_blanking,
    )
    actual_controls, controls_text = _query_camera_controls()

    producer = subprocess.Popen(
        [PRODUCER, device, str(raw_width), str(raw_height)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )
    frame_data = None
    producer_log = ""

    try:
        for _ in range(skip_frames + 1):
            frame_data = read_exact(producer.stdout, frame_bytes)
    finally:
        if producer.stdout is not None:
            producer.stdout.close()
        if producer.poll() is None:
            producer.terminate()
            try:
                producer.wait(timeout=3)
            except subprocess.TimeoutExpired:
                producer.kill()
                producer.wait()
        if producer.stderr is not None:
            producer_log = producer.stderr.read().decode(
                "utf-8",
                errors="replace",
            ).strip()
            producer.stderr.close()

    if frame_data is None:
        raise RuntimeError("No frame was captured")

    nv12 = np.frombuffer(frame_data, dtype=np.uint8).reshape(
        (raw_height * 3 // 2, raw_width)
    )
    direct = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    corrected = apply_color_correction(
        direct,
        blue_gain=blue_gain,
        green_gain=green_gain,
        red_gain=red_gain,
        saturation=saturation,
    )

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    if frame_name is None:
        frame_name = "imx219-" + datetime.now().strftime("%Y%m%d-%H%M%S")

    nv12_path = destination / f"{frame_name}.nv12"
    direct_path = destination / f"{frame_name}-direct.png"
    corrected_path = destination / f"{frame_name}-corrected.png"
    metadata_path = destination / f"{frame_name}.json"

    nv12_path.write_bytes(frame_data)
    if not cv2.imwrite(str(direct_path), direct):
        raise RuntimeError(f"Failed to write {direct_path}")
    if not cv2.imwrite(str(corrected_path), corrected):
        raise RuntimeError(f"Failed to write {corrected_path}")

    metadata = {
        "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "device": device,
        "sensor_subdevice": SENSOR_SUBDEVICE,
        "format": {
            "pixel_format": "NV12",
            "width": raw_width,
            "height": raw_height,
            "frame_bytes": frame_bytes,
            "v4l2_get_fmt_video": _query_video_format(device),
        },
        "requested_controls": {
            "vertical_blanking": vertical_blanking,
            "exposure": exposure,
            "analogue_gain": analogue_gain,
        },
        "actual_controls": actual_controls,
        "actual_controls_text": controls_text,
        "software_correction": {
            "blue_gain": blue_gain,
            "green_gain": green_gain,
            "red_gain": red_gain,
            "saturation": saturation,
        },
        "statistics": {
            "source_luma": _summarize_luma(nv12[:raw_height, :]),
            "direct_bgr": _summarize_bgr(direct),
            "corrected_bgr": _summarize_bgr(corrected),
        },
        "producer_log": producer_log,
        "files": {
            "nv12": str(nv12_path),
            "direct_png": str(direct_path),
            "corrected_png": str(corrected_path),
            "metadata_json": str(metadata_path),
        },
    }
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print("Saved V4L2 frame analysis set:")
    print(f"  NV12:      {nv12_path}")
    print(f"  Direct PNG:{direct_path}")
    print(f"  Corrected: {corrected_path}")
    print(f"  Metadata:  {metadata_path}")
    print(
        "  Luma:      "
        f"mean={metadata['statistics']['source_luma']['mean']}, "
        f"p01={metadata['statistics']['source_luma']['p01']}, "
        f"p99={metadata['statistics']['source_luma']['p99']}"
    )

    return metadata


class IMX219VisionSystem:
    def __init__(
        self,
        src=0,
        device=None,
        raw_width=960,
        raw_height=540,
        display_width=960,
        display_height=540,
        exposure=2645,
        analogue_gain=0,
        vertical_blanking=2446,
        blue_gain=0.95,
        green_gain=1.06,
        red_gain=1.00,
        saturation=1.08,
        luma_denoise=0,
        chroma_denoise=0,
        temporal_denoise=0.0,
        blur_kernel_size=7,
        threshold_value=60,
        min_contour_area=800,
        max_contour_area=200000,
        dilate_iterations=1,
        erode_iterations=1,
        brightness_threshold=200,
        min_bright_area=100,
        max_bright_area=5000,
        detection_zone=None,
        enable_bright_detection=True,
        capture_thread_daemon=False,
        auto_open=True,
    ):
        self.src = src
        if device is None:
            if isinstance(src, str):
                device = src
            elif src in (None, 0):
                device = "/dev/video10"
            else:
                device = f"/dev/video{src}"
        self.device = device
        self.raw_width = raw_width
        self.raw_height = raw_height
        self.display_width = display_width
        self.display_height = display_height
        self.exposure = exposure
        self.analogue_gain = analogue_gain
        self.vertical_blanking = vertical_blanking
        self.blue_gain = blue_gain
        self.green_gain = green_gain
        self.red_gain = red_gain
        self.saturation = saturation
        self.luma_denoise = luma_denoise
        self.chroma_denoise = chroma_denoise
        self.temporal_denoise = temporal_denoise

        self.blur_kernel_size = max(3, int(blur_kernel_size))
        if self.blur_kernel_size % 2 == 0:
            self.blur_kernel_size += 1
        self.threshold_value = int(np.clip(threshold_value, 0, 255))
        self.min_contour_area = max(1, int(min_contour_area))
        self.max_contour_area = max(
            self.min_contour_area,
            int(max_contour_area),
        )
        self.dilate_iterations = max(0, int(dilate_iterations))
        self.erode_iterations = max(0, int(erode_iterations))
        self.brightness_threshold = int(np.clip(brightness_threshold, 0, 255))
        self.min_bright_area = max(1, int(min_bright_area))
        self.max_bright_area = max(
            self.min_bright_area,
            int(max_bright_area),
        )
        self.enable_bright_detection = bool(enable_bright_detection)
        if detection_zone is None:
            detection_zone = (
                self.display_width // 3,
                self.display_height // 3,
                self.display_width * 2 // 3,
                self.display_height * 2 // 3,
            )
        self.detection_zone = tuple(detection_zone)
        self.detection_enabled = True
        self.capture_thread_daemon = bool(capture_thread_daemon)

        self.frame_lock = threading.Lock()
        self.detection_lock = threading.Lock()
        self.frame = None
        self.centers = []
        self.shape = None
        self.last_detections = []
        self.last_error = None
        self.capture_fps = 0.0
        self.camera_open = False
        self.running = False
        self.producer = None
        self.capture_thread = None

        if auto_open:
            self.open()

    def open(self, timeout=5.0):
        if self.running:
            return self

        set_camera_controls(
            exposure=self.exposure,
            analogue_gain=self.analogue_gain,
            vertical_blanking=self.vertical_blanking,
        )
        self.last_error = None
        self.frame = None
        self.producer = subprocess.Popen(
            [
                PRODUCER,
                self.device,
                str(self.raw_width),
                str(self.raw_height),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        self.running = True
        self.camera_open = True
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            name="imx219-capture",
            daemon=self.capture_thread_daemon,
        )
        self.capture_thread.start()

        deadline = time.monotonic() + timeout
        while self.frame is None and self.running and time.monotonic() < deadline:
            time.sleep(0.02)

        if self.frame is None:
            error = self.last_error or "Timed out waiting for the first camera frame"
            self.close()
            raise RuntimeError(error)

        return self

    def _capture_loop(self):
        frame_bytes = self.raw_width * self.raw_height * 3 // 2
        frame_count = 0
        statistics_start = time.monotonic()
        previous_frame = None

        try:
            while self.running:
                frame_data = read_exact(self.producer.stdout, frame_bytes)
                nv12 = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                    (self.raw_height * 3 // 2, self.raw_width)
                )
                frame = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

                if (
                    self.display_width != self.raw_width
                    or self.display_height != self.raw_height
                ):
                    frame = cv2.resize(
                        frame,
                        (self.display_width, self.display_height),
                        interpolation=cv2.INTER_AREA,
                    )

                if self.luma_denoise >= 3 or self.chroma_denoise >= 3:
                    frame = apply_fast_denoise(
                        frame,
                        luma_kernel=self.luma_denoise,
                        chroma_kernel=self.chroma_denoise,
                    )

                if (
                    self.blue_gain != 1.0
                    or self.green_gain != 1.0
                    or self.red_gain != 1.0
                    or self.saturation != 1.0
                ):
                    frame = apply_color_correction(
                        frame,
                        blue_gain=self.blue_gain,
                        green_gain=self.green_gain,
                        red_gain=self.red_gain,
                        saturation=self.saturation,
                    )

                if previous_frame is not None and self.temporal_denoise > 0:
                    frame = cv2.addWeighted(
                        frame,
                        1.0 - self.temporal_denoise,
                        previous_frame,
                        self.temporal_denoise,
                        0,
                    )
                previous_frame = frame.copy()

                if self.enable_bright_detection:
                    self._update_bright_spots(frame)

                with self.frame_lock:
                    self.frame = frame

                frame_count += 1
                now = time.monotonic()
                elapsed = now - statistics_start
                if elapsed >= 1.0:
                    self.capture_fps = frame_count / elapsed
                    frame_count = 0
                    statistics_start = now

        except (EOFError, OSError, ValueError) as error:
            if self.running:
                self.last_error = str(error)
        finally:
            self.camera_open = False
            self.running = False

    def is_camera_open(self):
        return self.camera_open and self.running

    def get_frame(self):
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None

    def get_image_center(self):
        frame = self.get_frame()
        if frame is None:
            return None
        height, width = frame.shape[:2]
        return width // 2, height // 2

    def set_processing_params(
        self,
        blur_kernel_size=None,
        threshold_value=None,
        min_contour_area=None,
        max_contour_area=None,
        dilate_iterations=None,
        erode_iterations=None,
    ):
        if blur_kernel_size is not None:
            self.blur_kernel_size = max(3, int(blur_kernel_size))
            if self.blur_kernel_size % 2 == 0:
                self.blur_kernel_size += 1
        if threshold_value is not None:
            self.threshold_value = int(np.clip(threshold_value, 0, 255))
        if min_contour_area is not None:
            self.min_contour_area = max(1, int(min_contour_area))
        if max_contour_area is not None:
            self.max_contour_area = max(
                self.min_contour_area,
                int(max_contour_area),
            )
        if dilate_iterations is not None:
            self.dilate_iterations = max(0, int(dilate_iterations))
        if erode_iterations is not None:
            self.erode_iterations = max(0, int(erode_iterations))

    def preprocess_frame(self, frame=None):
        source = self.get_frame() if frame is None else frame.copy()
        if source is None:
            return None

        gray = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(
            gray,
            (self.blur_kernel_size, self.blur_kernel_size),
            0,
        )
        _, binary = cv2.threshold(
            blurred,
            self.threshold_value,
            255,
            cv2.THRESH_BINARY_INV,
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        if self.dilate_iterations > 0:
            binary = cv2.dilate(
                binary,
                kernel,
                iterations=self.dilate_iterations,
            )
        if self.erode_iterations > 0:
            binary = cv2.erode(
                binary,
                kernel,
                iterations=self.erode_iterations,
            )
        return binary

    def detect_main_contour(self, processed_frame=None):
        if processed_frame is None:
            processed_frame = self.preprocess_frame()
        if processed_frame is None:
            return (0, 0), None

        contours, _ = cv2.findContours(
            processed_frame,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        filtered_contours = [
            contour
            for contour in contours
            if self.min_contour_area
            < cv2.contourArea(contour)
            < self.max_contour_area
        ]
        if not filtered_contours:
            return (0, 0), None

        contour = max(filtered_contours, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return (0, 0), None
        center = (
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]),
        )
        return center, contour

    def detect_color(self, contour, frame=None, color_space="hsv"):
        if contour is None or len(contour) < 3:
            return None
        source = self.get_frame() if frame is None else frame.copy()
        if source is None:
            return None

        contour = np.asarray(contour, dtype=np.int32).reshape((-1, 1, 2))
        mask = np.zeros(source.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        blue, green, red = cv2.mean(source, mask=mask)[:3]

        if color_space.lower() == "bgr":
            maximum = max(red, green, blue)
            if maximum < 30:
                return "black"
            if max(abs(red - green), abs(red - blue), abs(green - blue)) < 30:
                return "white" if maximum > 200 else "gray"
            if red > green * 1.4 and red > blue * 1.4:
                return "red"
            if green > red * 1.4 and green > blue * 1.4:
                return "green"
            if blue > red * 1.4 and blue > green * 1.4:
                return "blue"
            if red > blue * 1.3 and green > blue * 1.2:
                return "yellow"
            return "unknown"

        mean_bgr = np.uint8([[[blue, green, red]]])
        hue, saturation, value = cv2.cvtColor(
            mean_bgr,
            cv2.COLOR_BGR2HSV,
        )[0, 0]
        if value < 30:
            return "black"
        if saturation < 40:
            return "white" if value > 200 else "gray"
        if hue < 12 or hue >= 165:
            return "red"
        if hue < 38:
            return "yellow"
        if hue < 90:
            return "green"
        if hue < 105:
            return "cyan"
        if hue < 135:
            return "blue"
        if hue < 155:
            return "purple"
        return "pink"

    def detect_color_objects(self, frame=None, color="red"):
        source = self.get_frame() if frame is None else frame.copy()
        if source is None:
            return None
        hsv = cv2.cvtColor(source, cv2.COLOR_BGR2HSV)
        color_ranges = {
            "red": [((0, 65, 45), (12, 255, 255)), ((165, 65, 45), (179, 255, 255))],
            "yellow": [((15, 65, 55), (38, 255, 255))],
            "green": [((40, 55, 40), (90, 255, 255))],
            "blue": [((90, 60, 40), (135, 255, 255))],
            "black": [((0, 0, 0), (179, 70, 60))],
        }
        ranges = color_ranges.get(color.lower())
        if ranges is None:
            raise ValueError(f"Unsupported color: {color}")

        mask = np.zeros(source.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(
                    hsv,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                ),
            )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def detect_color_shape(
        self,
        shape="square",
        color="red",
        frame=None,
        minimum_area=None,
        maximum_area=None,
    ):
        source = self.get_frame() if frame is None else frame.copy()
        if source is None:
            return (0, 0), None
        mask = self.detect_color_objects(source, color=color)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        minimum_area = self.min_contour_area if minimum_area is None else minimum_area
        maximum_area = self.max_contour_area if maximum_area is None else maximum_area
        requested_shape = shape.lower()

        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not minimum_area <= area <= maximum_area:
                continue
            detected_shape = self.detect_shape(contour)
            shape_matches = detected_shape == requested_shape
            if requested_shape == "quadrilateral":
                shape_matches = detected_shape in ("square", "rectangle")
            if shape_matches:
                candidates.append(contour)

        if not candidates:
            return (0, 0), None
        contour = max(candidates, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return (0, 0), None
        center = (
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]),
        )
        return center, contour

    def detect_color_triangle(self, shape="triangle", color="red"):
        return self.detect_color_shape(shape=shape, color=color)

    def detect_circle_type(self, contour):
        if contour is None or len(contour) < 5:
            return None
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            return None
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        if circularity < 0.7:
            return None
        _, radius = cv2.minEnclosingCircle(contour)
        enclosing_area = np.pi * radius * radius
        if enclosing_area <= 0:
            return None
        return "solid_circle" if area / enclosing_area > 0.5 else "hollow_circle"

    def detect_circles(self, frame=None, minimum_area=None, maximum_area=None):
        source = self.get_frame() if frame is None else frame.copy()
        if source is None:
            return None, []
        gray = cv2.cvtColor(source, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        _, binary = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(
            binary,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        minimum_area = self.min_contour_area if minimum_area is None else minimum_area
        maximum_area = self.max_contour_area if maximum_area is None else maximum_area
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not minimum_area <= area <= maximum_area:
                continue
            circle_type = self.detect_circle_type(contour)
            if circle_type is None:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            center = (
                int(moments["m10"] / moments["m00"]),
                int(moments["m01"] / moments["m00"]),
            )
            detections.append(
                {
                    "label": circle_type,
                    "center": center,
                    "area": int(area),
                    "contour": contour,
                }
            )
            cv2.drawContours(source, [contour], -1, (0, 255, 0), 2)
            cv2.circle(source, center, 5, (0, 0, 255), -1)
        return source, detections

    def _update_bright_spots(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        value = hsv[:, :, 2]
        _, mask = cv2.threshold(
            value,
            self.brightness_threshold,
            255,
            cv2.THRESH_BINARY,
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        centers = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not self.min_bright_area < area < self.max_bright_area:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            centers.append(
                (
                    int(moments["m10"] / moments["m00"]),
                    int(moments["m01"] / moments["m00"]),
                )
            )
        with self.detection_lock:
            self.centers = centers

    def get_bright_spots(self):
        with self.detection_lock:
            return self.centers.copy()

    def get_annotated_bright_frame(self):
        frame = self.get_frame()
        if frame is None:
            return None
        for center_x, center_y in self.get_bright_spots():
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"({center_x},{center_y})",
                (center_x + 8, center_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        left, top, right, bottom = self.detection_zone
        cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)
        return frame

    def check_bright_spot_in_zone(self):
        if not self.detection_enabled:
            return False
        left, top, right, bottom = self.detection_zone
        for center_x, center_y in self.get_bright_spots():
            if left <= center_x <= right and top <= center_y <= bottom:
                self.detection_enabled = False
                return True
        return False

    def reset_bright_detection(self):
        self.detection_enabled = True

    def show_frame(self, annotated_bright=False, window_name="IMX219 View"):
        while self.running:
            frame = (
                self.get_annotated_bright_frame()
                if annotated_bright
                else self.get_frame()
            )
            if frame is None:
                time.sleep(0.01)
                continue
            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        try:
            cv2.destroyWindow(window_name)
        except cv2.error:
            pass

    def show_bright_frame(self):
        self.show_frame(
            annotated_bright=True,
            window_name="IMX219 Bright Detection",
        )

    def set_controls(
        self,
        exposure=None,
        analogue_gain=None,
        vertical_blanking=None,
    ):
        if exposure is not None:
            self.exposure = exposure
        if analogue_gain is not None:
            self.analogue_gain = analogue_gain
        if vertical_blanking is not None:
            self.vertical_blanking = vertical_blanking

        set_camera_controls(
            exposure=self.exposure,
            analogue_gain=self.analogue_gain,
            vertical_blanking=self.vertical_blanking,
        )

    def detect_colors(self, frame=None, minimum_area=600):
        source = self.get_frame() if frame is None else frame.copy()
        if source is None:
            return None, []
        detections = draw_color_detections(
            source,
            minimum_area=minimum_area,
        )
        self.last_detections = detections
        return source, detections

    def detect_red_squares(self, frame=None, minimum_area=600):
        source = self.get_frame() if frame is None else frame.copy()
        if source is None:
            return None, []
        detections = draw_red_square_detections(
            source,
            minimum_area=minimum_area,
        )
        self.last_detections = detections
        return source, detections

    @staticmethod
    def detect_shape(contour, epsilon_factor=0.035):
        if contour is None or len(contour) < 3:
            return None

        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(
            contour,
            epsilon_factor * perimeter,
            True,
        )
        vertices = len(polygon)
        if vertices == 3:
            return "triangle"
        if vertices == 4:
            _, (width, height), _ = cv2.minAreaRect(contour)
            shorter_side = min(width, height)
            longer_side = max(width, height)
            if shorter_side <= 0:
                return None
            return "square" if longer_side / shorter_side <= 1.20 else "rectangle"
        if vertices == 5:
            return "pentagon"

        area = cv2.contourArea(contour)
        if perimeter <= 0:
            return None
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        return "circle" if circularity >= 0.75 else "polygon"

    def show_preview(
        self,
        detect_colors=False,
        detect_red_squares=False,
        minimum_color_area=600,
        minimum_square_area=600,
        display_interval=0.08,
        jpeg_quality=65,
        close_on_stop=True,
    ):
        _require_jupyter_display()
        if not self.running:
            self.open()

        blank = np.zeros(
            (self.display_height, self.display_width, 3),
            dtype=np.uint8,
        )
        encoded, blank_jpeg = cv2.imencode(".jpg", blank)
        if not encoded:
            raise RuntimeError("Failed to create the initial JPEG image")

        image_widget = widgets.Image(
            value=blank_jpeg.tobytes(),
            format="jpeg",
            layout=widgets.Layout(
                width=f"{self.display_width}px",
                height=f"{self.display_height}px",
            ),
        )
        status = widgets.Label(value="Starting IMX219 vision system...")
        display(status, image_widget)

        display_count = 0
        display_start = time.monotonic()
        last_display_time = 0.0

        try:
            while self.running:
                now = time.monotonic()
                if now - last_display_time < display_interval:
                    time.sleep(0.005)
                    continue
                last_display_time = now

                frame = self.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                detections = []
                if detect_colors:
                    detections.extend(
                        draw_color_detections(
                            frame,
                            minimum_area=minimum_color_area,
                        )
                    )
                if detect_red_squares:
                    detections.extend(
                        draw_red_square_detections(
                            frame,
                            minimum_area=minimum_square_area,
                        )
                    )
                self.last_detections = detections

                encoded, jpeg = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
                )
                if not encoded:
                    continue

                image_widget.value = jpeg.tobytes()
                display_count += 1
                elapsed = max(now - display_start, 1e-6)
                counts = {}
                for detection in detections:
                    label = detection["label"]
                    counts[label] = counts.get(label, 0) + 1
                status.value = (
                    f"capture={self.capture_fps:.1f} fps, "
                    f"display={display_count / elapsed:.1f} fps, "
                    f"detections={counts}"
                )

        except KeyboardInterrupt:
            status.value = "Stopped by user"
        finally:
            if close_on_stop:
                self.close()
                status.value += " | camera closed"

    def close(self):
        self.running = False
        producer = self.producer

        if producer is not None and producer.poll() is None:
            producer.terminate()
            try:
                producer.wait(timeout=3)
            except subprocess.TimeoutExpired:
                producer.kill()
                producer.wait()

        if producer is not None:
            if producer.stdout is not None:
                producer.stdout.close()
            if producer.stderr is not None:
                producer.stderr.close()

        if (
            self.capture_thread is not None
            and self.capture_thread.is_alive()
            and self.capture_thread is not threading.current_thread()
        ):
            self.capture_thread.join(timeout=3)

        self.producer = None
        self.capture_thread = None
        self.camera_open = False

    def reopen(self):
        self.close()
        return self.open()

    def release(self):
        self.close()

    def __enter__(self):
        if not self.running:
            self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


VisionSystem = IMX219VisionSystem
