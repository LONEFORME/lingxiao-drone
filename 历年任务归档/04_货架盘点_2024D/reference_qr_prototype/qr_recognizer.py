import sys
import time
from pathlib import Path

from qr_scanner import QRVisionSystem


class SysfsPWM:
    """Small sysfs PWM adapter used by the servo notebook."""

    def __init__(self, chip=0, channel=0, frequency=50):
        self.base = f"/sys/class/pwm/pwmchip{chip}"
        self.path = f"{self.base}/pwm{channel}"
        self.period = int(1_000_000_000 / frequency)

        if not Path(self.path).exists():
            with open(f"{self.base}/export", "w", encoding="ascii") as pwm_file:
                pwm_file.write(str(channel))
            time.sleep(0.2)

        self._write("enable", 0)
        self._write("period", self.period)

    def _write(self, name, value):
        with open(f"{self.path}/{name}", "w", encoding="ascii") as pwm_file:
            pwm_file.write(str(value))

    def set_duty_ns(self, duty_ns):
        self._write("duty_cycle", max(0, min(self.period, int(duty_ns))))

    def start(self):
        self._write("enable", 1)

    def stop(self):
        self._write("enable", 0)


class Servo:
    def __init__(self, channel=0):
        self.pwm = SysfsPWM(channel=channel, frequency=50)

    def start(self, angle=0):
        self.pwm.start()
        self.angle(angle)

    def angle(self, angle):
        angle = max(0, min(180, float(angle)))
        duty_ns = 500_000 + int(angle / 180 * 2_000_000)
        self.pwm.set_duty_ns(duty_ns)

    def stop(self):
        self.pwm.stop()


class Laser:
    def __init__(self, pin=19):
        try:
            import Hobot.GPIO as GPIO
        except ImportError as exc:
            raise RuntimeError("需要在安装 Hobot.GPIO 的开发板环境中运行") from exc

        self.GPIO = GPIO
        self.pin = pin
        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setup(self.pin, self.GPIO.OUT)
        self.GPIO.output(self.pin, self.GPIO.LOW)

    def pulse(self, duration=1.0, value=20, pwm_range=200, period=0.02):
        """Keep the laser PWM output active for the requested duration."""
        on_time = period * value / pwm_range
        off_time = period - on_time
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.GPIO.output(self.pin, self.GPIO.HIGH)
            time.sleep(on_time)
            self.GPIO.output(self.pin, self.GPIO.LOW)
            time.sleep(off_time)
        self.GPIO.output(self.pin, self.GPIO.LOW)

    def close(self):
        self.GPIO.output(self.pin, self.GPIO.LOW)
        self.GPIO.cleanup()


class QRDictionaryRecognizer(QRVisionSystem):
    """Recognize QR codes using a fixed number-to-content mapping."""

    def __init__(
        self,
        mapping_file="qr_mapping.txt",
        laser_pin=19,
        servo_channel=0,
        servo_batch_size=6,
        laser_duration=1.0,
        enable_hardware=True,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.mapping_file = Path(mapping_file)

        if not self.load_mapping(self.mapping_file):
            raise FileNotFoundError(f"Mapping file not found: {self.mapping_file}")

        self.current_number = None
        self.current_status = "WAITING"
        self.laser_duration = float(laser_duration)
        self.servo_batch_size = int(servo_batch_size)
        self.recognized_numbers = set()
        self.laser = None
        self.servo = None
        self.servo_angle = 0

        if enable_hardware:
            self.laser = Laser(pin=laser_pin)
            self.servo = Servo(channel=servo_channel)
            self.servo.start(self.servo_angle)

    def _handle_new_qr(self, number):
        """Trigger hardware once for each new mapped QR number."""
        if number is None or number in self.recognized_numbers:
            return

        self.recognized_numbers.add(number)
        print(
            f"\nRecognized QR {number} "
            f"({len(self.recognized_numbers)} total)"
        )

        if self.laser is not None:
            self.laser.pulse(self.laser_duration)

        if (
            self.servo is not None
            and len(self.recognized_numbers) % self.servo_batch_size == 0
        ):
            self.servo_angle = 0 if self.servo_angle == 180 else 180
            self.servo.angle(self.servo_angle)
            print(
                f"Servo moved to {self.servo_angle} degrees after "
                f"{len(self.recognized_numbers)} QRs"
            )

    def recognize(self, frame):
        """Return the mapped number, UNKNOWN, or None when no QR is visible."""
        detection_frame = self.prepare_detection_frame(frame)
        content, points, _ = self.qr_detector.detectAndDecode(detection_frame)
        if not content or points is None:
            return None

        number = self.get_qr_number(content)
        if number is None:
            return "UNKNOWN"
        return number

    def _print_status(self, status):
        sys.stdout.write(f"\rQR_NUMBER: {status}\033[K")
        sys.stdout.flush()

    def run(self):
        """Continuously recognize mapped QR codes until Ctrl+C is pressed."""
        self.open()

        last_scanned_frame_id = -1
        next_scan_at = 0.0
        last_status = None
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
                    result = self.recognize(frame)
                    status = "WAITING" if result is None else str(result)

                    if isinstance(result, int):
                        self._handle_new_qr(result)

                    if status != last_status:
                        self._print_status(status)
                        last_status = status

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
                            frame, "QR_NUMBER: {}".format(status)
                        ):
                            break
                        last_display_at = now

                    last_scanned_frame_id = frame_id
                    next_scan_at = now + self.scan_interval
                else:
                    time.sleep(0.005)
        except KeyboardInterrupt:
            pass
        finally:
            self.release()
            if self.servo is not None:
                self.servo.stop()
            if self.laser is not None:
                self.laser.close()
            print()


if __name__ == "__main__":
    recognizer = QRDictionaryRecognizer(
        mapping_file="qr_mapping.txt",
        src="/dev/video42",
        width=1280,
        height=720,
        fps=15,
        scan_interval=0.10,
        detection_width=800,
        max_qr_number=24,
        laser_pin=19,
        servo_channel=0,
        servo_batch_size=6,
        laser_duration=1.0,
    )
    recognizer.run()
