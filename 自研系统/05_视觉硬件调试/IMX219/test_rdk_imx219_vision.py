import argparse
import signal
import threading
import time

import cv2

import rdk_imx219_jupyter_preview as vision_system


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Test the RDK X5 IMX219 vision system",
    )
    parser.add_argument(
        "--mode",
        choices=("red-square", "main-contour", "bright"),
        default="red-square",
        help="visual detection mode",
    )
    parser.add_argument("--device", default="/dev/video10")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--minimum-area", type=int, default=800)
    parser.add_argument(
        "--display",
        action="store_true",
        help="open an OpenCV window; leave disabled for autostart/headless use",
    )
    return parser.parse_args()


def process_frame(vision, frame, mode, minimum_area):
    if mode == "red-square":
        annotated, detections = vision.detect_red_squares(
            frame,
            minimum_area=minimum_area,
        )
        if not detections:
            return annotated, None
        target = max(detections, key=lambda item: item["area"])
        return annotated, target

    if mode == "main-contour":
        processed = vision.preprocess_frame(frame)
        center, contour = vision.detect_main_contour(processed)
        annotated = frame.copy()
        if contour is None:
            return annotated, None
        cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 2)
        cv2.circle(annotated, center, 5, (0, 0, 255), -1)
        return annotated, {
            "label": vision.detect_shape(contour),
            "color": vision.detect_color(contour, frame),
            "center": center,
            "area": int(cv2.contourArea(contour)),
        }

    annotated = vision.get_annotated_bright_frame()
    centers = vision.get_bright_spots()
    return annotated, {
        "label": "BRIGHT_SPOTS",
        "centers": centers,
        "in_zone": vision.check_bright_spot_in_zone(),
    }


def main():
    arguments = parse_arguments()
    stop_event = threading.Event()

    def request_stop(_signal_number, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    vision = vision_system.VisionSystem(
        0,
        device=arguments.device,
        raw_width=arguments.width,
        raw_height=arguments.height,
        display_width=arguments.width,
        display_height=arguments.height,
        exposure=2645,
        analogue_gain=0,
        vertical_blanking=2446,
        blue_gain=0.95,
        green_gain=1.06,
        red_gain=1.00,
        saturation=1.08,
        enable_bright_detection=arguments.mode == "bright",
        capture_thread_daemon=False,
    )

    last_report_time = 0.0
    last_target = None

    try:
        print(
            "Vision system started: "
            f"mode={arguments.mode}, device={arguments.device}, "
            f"size={arguments.width}x{arguments.height}"
        )

        while not stop_event.is_set() and vision.is_camera_open():
            frame = vision.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            annotated, target = process_frame(
                vision,
                frame,
                arguments.mode,
                arguments.minimum_area,
            )

            now = time.monotonic()
            if target != last_target or now - last_report_time >= 1.0:
                print(
                    f"capture_fps={vision.capture_fps:.1f}, "
                    f"target={target}"
                )
                last_target = target
                last_report_time = now

            if arguments.display and annotated is not None:
                image_center = vision.get_image_center()
                if image_center is not None:
                    cv2.circle(annotated, image_center, 5, (255, 0, 0), -1)
                cv2.imshow("RDK IMX219 Vision Test", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    stop_event.set()

            time.sleep(0.02)

        if vision.last_error:
            raise RuntimeError(vision.last_error)

    finally:
        vision.release()
        if arguments.display:
            cv2.destroyAllWindows()
        print("Vision system stopped")


if __name__ == "__main__":
    main()
