import glob
import json
import os
import socket
import socketserver
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlsplit


PORT = 8080
VIDEO_SIZE = "1280x720"
FRAME_RATE = "30"

HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
html,body{margin:0;width:100%;height:100%;background:#111;overflow:hidden}
body{display:flex;align-items:center;justify-content:center}
img{max-width:100%;max-height:100%;object-fit:contain}
</style></head><body><img src="/stream" alt="Camera stream"></body></html>"""

process_lock = threading.Lock()
frame_lock = threading.Lock()
process = None
latest_frame = None
latest_frame_id = 0


def find_camera():
    paths = glob.glob("/dev/v4l/by-id/usb-*-video-index0")
    return paths[0] if paths else None


def start_ffmpeg(camera_path):
    global process

    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", VIDEO_SIZE, "-framerate", FRAME_RATE,
        "-i", camera_path,
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-c:v", "copy", "-f", "image2pipe", "-",
    ]
    new_process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL,
        bufsize=0,
    )
    with process_lock:
        old_process = process
        process = new_process
    if old_process and old_process.poll() is None:
        old_process.terminate()
    return new_process


def read_frames():
    global latest_frame, latest_frame_id

    while True:
        with process_lock:
            current_process = process
        if not current_process or current_process.poll() is not None:
            time.sleep(0.1)
            continue

        buffer = b""
        while current_process.poll() is None:
            chunk = current_process.stdout.read(65536)
            if not chunk:
                break
            buffer += chunk
            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    buffer = buffer[-1:]
                    break
                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    buffer = buffer[start:]
                    break
                with frame_lock:
                    latest_frame = buffer[start:end + 2]
                    latest_frame_id += 1
                buffer = buffer[end + 2:]


def monitor_camera():
    while True:
        camera_path = find_camera()
        if not camera_path:
            time.sleep(2)
            continue
        current_process = start_ffmpeg(camera_path)
        current_process.wait()
        time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(HTML.encode("utf-8"))
            return

        if path == "/health":
            status = {
                "status": "ok",
                "camera": find_camera() is not None,
                "streaming": latest_frame is not None,
                "frame_count": latest_frame_id,
            }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(json.dumps(status).encode("utf-8"))
            return

        if path != "/stream":
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()

        sent_frame_id = -1
        while True:
            with frame_lock:
                frame = latest_frame
                frame_id = latest_frame_id
            if frame is None or frame_id == sent_frame_id:
                time.sleep(0.005)
                continue
            try:
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(frame)).encode()
                    + b"\r\n\r\n" + frame + b"\r\n"
                )
                self.wfile.flush()
                sent_frame_id = frame_id
            except (BrokenPipeError, ConnectionResetError):
                return

    def log_message(self, format, *args):
        pass


class DualStackHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    address_family = socket.AF_INET6
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


threading.Thread(target=read_frames, daemon=True).start()
threading.Thread(target=monitor_camera, daemon=True).start()
DualStackHTTPServer(("::", PORT), Handler).serve_forever()