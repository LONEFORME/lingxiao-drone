import sys
import threading
import time
from pathlib import Path

import cv2
from IPython.display import Image as IPythonImage
from IPython.display import display


class QRVisionSystem:
    """Capture QR codes and assign unique contents sequential numbers."""

    def __init__(
        self,
        src="/dev/video42",
        width=1280,
        height=720,
        fps=15,
        scan_interval=0.10,
        detection_width=800,
        max_qr_number=24,
        display_size=(640, 360),
        display_interval=0.15,
        jpeg_quality=60,
    ):
        self.src = src
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.scan_interval = float(scan_interval)
        self.detection_width = int(detection_width)
        self.max_qr_number = int(max_qr_number)
        self.display_width, self.display_height = map(int, display_size)
        self.display_interval = float(display_interval)
        self.jpeg_quality = int(jpeg_quality)

        self.cap = None
        self.running = False
        self.capture_thread = None
        self.frame_lock = threading.Lock()
        self.frame = None
        self.frame_id = 0

        self.qr_detector = cv2.QRCodeDetector()
        self.qr_mapping = {}
        self.qr_reverse_mapping = {}
        self.next_qr_number = 1
        self.last_detected_number = None
        self.last_printed_status = None
        self.capture_frame_count = 0
        self.capture_fps = 0.0
        self.display_fps = 0.0
        self.display_handle = None

    def open(self):
        if self.cap is not None and self.cap.isOpened():
            return self

        self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = None
            raise RuntimeError(f"Cannot open camera: {self.src}")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.running = True
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name="camera-capture",
        )
        self.capture_thread.start()
        return self

    def _capture_loop(self):
        frame_interval = 1.0 / max(self.fps, 1)

        while self.running and self.cap is not None:
            started_at = time.monotonic()
            ok, frame = self.cap.read()

            if ok:
                with self.frame_lock:
                    self.frame = frame
                    self.frame_id += 1
                self.capture_frame_count += 1
            else:
                time.sleep(0.01)

            remaining = frame_interval - (time.monotonic() - started_at)
            if remaining > 0:
                time.sleep(remaining)

    def read(self):
        with self.frame_lock:
            if self.frame is None:
                return None, self.frame_id
            return self.frame.copy(), self.frame_id

    def register_qr_content(self, content):
        """Return (number, is_new) without numbering duplicate content twice."""
        content = str(content).strip()
        if not content:
            return None, False

        existing_number = self.qr_reverse_mapping.get(content)
        if existing_number is not None:
            return existing_number, False

        if self.next_qr_number > self.max_qr_number:
            return None, False

        number = self.next_qr_number
        self.qr_mapping[number] = content
        self.qr_reverse_mapping[content] = number
        self.next_qr_number += 1
        return number, True

    def detect_qr(self, frame):
        """Decode one QR code and return its assigned number and raw content."""
        detection_frame = self.prepare_detection_frame(frame)
        content, points, _ = self.qr_detector.detectAndDecode(detection_frame)
        if not content or points is None:
            return None

        number, is_new = self.register_qr_content(content)
        if number is None:
            return None

        self.last_detected_number = number
        return {
            "number": number,
            "content": content,
            "is_new": is_new,
        }

    def prepare_detection_frame(self, frame):
        """Downscale large frames to reduce QR detection latency and CPU work."""
        height, width = frame.shape[:2]
        if self.detection_width <= 0 or width <= self.detection_width:
            return frame

        scale = self.detection_width / width
        detection_height = max(1, round(height * scale))
        return cv2.resize(
            frame,
            (self.detection_width, detection_height),
            interpolation=cv2.INTER_AREA,
        )

    def get_qr_content(self, number):
        """Get the original QR content by its assigned number."""
        return self.qr_mapping.get(int(number))

    def get_qr_number(self, content):
        """Get the assigned number by the original QR content."""
        return self.qr_reverse_mapping.get(str(content).strip())

    def _show_frame(self, frame, status_text):
        """Update the in-notebook preview using one persistent display handle."""
        preview = cv2.resize(
            frame,
            (self.display_width, self.display_height),
            interpolation=cv2.INTER_LINEAR,
        )
        cv2.putText(
            preview, status_text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 0), 2,
        )
        cv2.putText(
            preview,
            "Capture: {:.1f} FPS | Display: {:.1f} FPS".format(
                self.capture_fps, self.display_fps
            ),
            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
        )
        ok, jpg = cv2.imencode(
            ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            return True

        image = IPythonImage(data=jpg.tobytes(), format="jpeg")
        if self.display_handle is None:
            self.display_handle = display(image, display_id=True)
        else:
            self.display_handle.update(image)
        return True

    def save_mapping(self, file_path="qr_mapping.txt"):
        """Save one mapping per line as: number, tab, QR content."""
        path = Path(file_path)
        lines = [
            f"{number}\t{content}"
            for number, content in sorted(self.qr_mapping.items())
        ]
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def load_mapping(self, file_path="qr_mapping.txt"):
        """Load a previously saved mapping and continue from its next number."""
        path = Path(file_path)
        if not path.exists():
            return False

        mapping = {}
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line.strip():
                continue
            try:
                number_text, content = line.split("\t", 1)
                mapping[int(number_text)] = content
            except (ValueError, TypeError) as exc:
                raise ValueError(
                    f"Invalid mapping at line {line_number}: {line!r}"
                ) from exc

        if any(number < 1 or number > self.max_qr_number for number in mapping):
            raise ValueError("Mapping contains a number outside the allowed range")
        if len(set(mapping.values())) != len(mapping):
            raise ValueError("Mapping contains duplicate QR contents")

        self.qr_mapping = dict(sorted(mapping.items()))
        self.qr_reverse_mapping = {
            content: number for number, content in self.qr_mapping.items()
        }
        self.next_qr_number = max(self.qr_mapping, default=0) + 1
        return True

    def _print_status(self):
        number = self.last_detected_number
        number_text = str(number) if number is not None else "WAITING"
        status = (number_text, len(self.qr_mapping))
        if status == self.last_printed_status:
            return

        sys.stdout.write(
            f"\rQR_NUMBER: {number_text} | SAVED: "
            f"{len(self.qr_mapping)}/{self.max_qr_number}\033[K"
        )
        sys.stdout.flush()
        self.last_printed_status = status

    def run(self):
        """Run until stopped or all expected QR codes have been collected."""
        self.open()

        last_scanned_frame_id = -1
        next_scan_at = 0.0
        completed = False
        last_display_at = 0.0
        display_count = 0
        stats_started_at = time.monotonic()
        stats_capture_count = self.capture_frame_count

        try:
            while self.running:
                now = time.monotonic()

                if now < next_scan_at:
                    time.sleep(min(0.01, next_scan_at - now))
                    continue

                frame, frame_id = self.read()

                if (
                    frame is not None
                    and frame_id != last_scanned_frame_id
                ):
                    self.detect_qr(frame)
                    last_scanned_frame_id = frame_id
                    next_scan_at = now + self.scan_interval
                    self._print_status()

                    now = time.monotonic()
                    if now - last_display_at >= self.display_interval:
                        display_count += 1
                        elapsed = now - stats_started_at
                        if elapsed >= 1.0:
                            self.capture_fps = (
                                self.capture_frame_count - stats_capture_count
                            ) / elapsed
                            self.display_fps = display_count / elapsed
                            stats_capture_count = self.capture_frame_count
                            display_count = 0
                            stats_started_at = now
                        if not self._show_frame(
                            frame,
                            "QR: {} | SAVED: {}/{}".format(
                                self.last_detected_number or "WAITING",
                                len(self.qr_mapping), self.max_qr_number,
                            ),
                        ):
                            break
                        last_display_at = now

                    if len(self.qr_mapping) >= self.max_qr_number:
                        completed = True
                        break
                else:
                    time.sleep(0.005)
        except KeyboardInterrupt:
            pass
        finally:
            self.release()
            print()

        if completed:
            self.save_mapping("qr_mapping.txt")
            print("Completed: 24 unique QR codes saved to qr_mapping.txt")

    def stop(self):
        self.running = False

    def release(self):
        self.running = False

        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2)
        self.capture_thread = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()


if __name__ == "__main__":
    camera = QRVisionSystem(
        src="/dev/video42",
        width=1280,
        height=720,
        fps=15,
        scan_interval=0.10,
        detection_width=800,
        max_qr_number=24,
    )
    camera.run()

    print(camera.qr_mapping)
    # camera.save_mapping("qr_mapping.txt")
