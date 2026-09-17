#!/usr/bin/env python3
from http import server
import io
import logging
import socket
import socketserver
from threading import Condition
from urllib.parse import urlsplit

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput


PORT = 8090
WIDTH = 640
HEIGHT = 360
FPS = 20


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    def send_low_latency_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_low_latency_headers()
        self.end_headers()

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_low_latency_headers()
            self.end_headers()
            self.wfile.write(
                b"<html><body><img src='/stream.mjpg' style='max-width:100%;height:auto'></body></html>"
            )
            return

        if path != "/stream.mjpg":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.send_low_latency_headers()
        self.end_headers()
        try:
            while True:
                with output.condition:
                    output.condition.wait(timeout=1)
                    frame = output.frame
                if frame is None:
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except Exception as exc:
            logging.warning("stream client removed: %s", exc)


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


picam2 = Picamera2()
try:
    video_config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT)},
        controls={"FrameRate": FPS},
        buffer_count=2,
    )
except TypeError:
    video_config = picam2.create_video_configuration(main={"size": (WIDTH, HEIGHT)})
picam2.configure(video_config)
output = StreamingOutput()
try:
    picam2.start_recording(MJPEGEncoder(), FileOutput(output), quality=8)
except Exception:
    picam2.start_recording(MJPEGEncoder(), FileOutput(output))

try:
    address = ("0.0.0.0", PORT)
    print(f"Picamera2 MJPEG stream: http://<cm5-ip>:{PORT}/stream.mjpg", flush=True)
    StreamingServer(address, StreamingHandler).serve_forever()
finally:
    picam2.stop_recording()
