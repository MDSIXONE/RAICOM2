#!/usr/bin/env python3
"""YOLO 检查工具：画全部 NMS 框，8090 MJPEG 推流，不动狗。

不用 xgoscreen LCD：LCD 走 SPI，与原厂 main.py 并发会 spidev_ioctl 死锁
（2026-08-30 两次实机复现，kill 无效只能重启），画面在浏览器看即可。
"""

from __future__ import annotations

import argparse
import atexit
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from http import server
from pathlib import Path
from socketserver import ThreadingMixIn
from threading import Condition
from urllib.parse import urlsplit

import cv2
import numpy as np
import onnxruntime as ort
from picamera2 import Picamera2

PORT = 8090
DEFAULT_MODEL = "/home/pi/ros_ws/models/letters.onnx"
DEFAULT_NAMES = "A,B,C,D"
CLASS_COLORS = [
    (0, 0, 255),
    (0, 255, 0),
    (255, 0, 0),
    (0, 255, 255),
    (255, 0, 255),
    (255, 255, 0),
    (128, 128, 255),
    (255, 128, 0),
]


class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def set_frame(self, jpeg_bytes: bytes):
        with self.condition:
            self.frame = jpeg_bytes
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    output: StreamingOutput = None  # type: ignore[assignment]

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            body = (
                b"<html><body><img src='/stream.mjpg' "
                b"style='max-width:100%;height:auto'></body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.end_headers()
        try:
            while True:
                with self.output.condition:
                    self.output.condition.wait(timeout=1.0)
                    frame = self.output.frame
                if frame is None:
                    continue
                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return


class StreamingServer(ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


SERVICES = ("oumax-camera.service",)
_service_was_active = {}


def _is_active(name):
    completed = subprocess.run(
        ["systemctl", "is-active", name],
        capture_output=True, text=True,
        check=False,
    )
    return completed.returncode == 0 and completed.stdout.strip() == "active"


def _wait_inactive(name, timeout=15.0):
    """轮询等待服务真正 inactive；防 Restart=always 竞态（停完又被拉起）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not _is_active(name):
            return True
        time.sleep(0.2)
    return False


def stop_services():
    """停相机推流服务（oumax-camera 占 Picamera2 与 8090）。

    本工具不用 LCD（SPI 死锁，2026-08-30 实机复现）也不碰串口，只需释放相机。
    """
    for name in SERVICES:
        _service_was_active[name] = _is_active(name)
        print(f"Stopping {name} ...", flush=True)
        subprocess.run(["sudo", "systemctl", "stop", name], check=True)
        if not _wait_inactive(name):
            raise RuntimeError(
                f"{name} still active after stop (Restart=always race?); "
                f"check: systemctl status {name}"
            )


def restore_services():
    """按启动前状态恢复服务（仅恢复原本 active 的）。"""
    for name in SERVICES:
        if _service_was_active.get(name, True):
            print(f"Restoring {name} ...", flush=True)
            completed = subprocess.run(
                ["sudo", "systemctl", "start", name],
                check=False,
            )
            if completed.returncode != 0:
                print(
                    f"ERROR: failed to start {name} "
                    f"(exit={completed.returncode})",
                    flush=True,
                )


def letterbox(image, size):
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x = (size - resized.shape[1]) // 2
    pad_y = (size - resized.shape[0]) // 2
    canvas[pad_y:pad_y + resized.shape[0], pad_x:pad_x + resized.shape[1]] = resized
    return canvas, scale, pad_x, pad_y


def detect_all(session, input_name, image, confidence):
    """复用 ball_yolo_grab 的 letterbox/blob/NMS，返回全部 NMS 框。"""
    input_size = int(session.get_inputs()[0].shape[2])
    boxed, scale, pad_x, pad_y = letterbox(image, input_size)
    # Picamera2 的 RGB888 已是模型训练所需的 RGB 通道顺序，不能再交换 R/B。
    blob = cv2.dnn.blobFromImage(boxed, 1 / 255.0, (input_size, input_size), swapRB=False)
    predictions = session.run(None, {input_name: blob})[0][0].T
    class_scores = predictions[:, 4:]
    classes = np.argmax(class_scores, axis=1)
    scores = class_scores[np.arange(len(predictions)), classes]
    keep = np.where(scores >= confidence)[0]
    if not len(keep):
        return []
    boxes = predictions[keep, :4]
    xyxy = np.column_stack((
        boxes[:, 0] - boxes[:, 2] / 2,
        boxes[:, 1] - boxes[:, 3] / 2,
        boxes[:, 0] + boxes[:, 2] / 2,
        boxes[:, 1] + boxes[:, 3] / 2,
    ))
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores[keep].tolist(), confidence, 0.45)
    if indices is None or not len(indices):
        return []
    results = []
    for position in np.asarray(indices).ravel():
        idx = int(position)
        x1, y1, x2, y2 = xyxy[idx]
        x1, x2 = (x1 - pad_x) / scale, (x2 - pad_x) / scale
        y1, y2 = (y1 - pad_y) / scale, (y2 - pad_y) / scale
        results.append({
            "x1": float(x1),
            "y1": float(y1),
            "x2": float(x2),
            "y2": float(y2),
            "confidence": float(scores[keep[idx]]),
            "cls": int(classes[keep[idx]]),
        })
    return results


def draw_detections(image, detections, names):
    canvas = image.copy()
    for det in detections:
        cls_id = det["cls"]
        color = CLASS_COLORS[cls_id % len(CLASS_COLORS)]
        x1, y1, x2, y2 = (
            int(det["x1"]), int(det["y1"]), int(det["x2"]), int(det["y2"])
        )
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        if 0 <= cls_id < len(names):
            label_name = names[cls_id]
        else:
            label_name = str(cls_id)
        label = f"{label_name} {det['confidence']:.2f}"
        cv2.putText(
            canvas, label, (x1, max(0, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
        )
    return canvas


def read_key(timeout=0.0):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ""
    return sys.stdin.read(1)


def main():
    parser = argparse.ArgumentParser(description="YOLO view tool (no motion)")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--names",
        default=DEFAULT_NAMES,
        help="Comma-separated class names, e.g. A,B,C,D or red,blue,green",
    )
    parser.add_argument("--confidence", type=float, default=0.60)
    args = parser.parse_args()
    names = [n.strip() for n in args.names.split(",") if n.strip()]
    if not names:
        raise ValueError("--names must provide at least one class name")

    stop_services()
    atexit.register(restore_services)

    print(f"model={args.model} names={names} conf={args.confidence}", flush=True)
    print(f"MJPEG 2x: http://192.168.137.157:{PORT}/stream.mjpg", flush=True)
    print("Keys: s save JPG, q/Ctrl-C quit", flush=True)

    session = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    output = StreamingOutput()
    StreamingHandler.output = output
    httpd = StreamingServer(("0.0.0.0", PORT), StreamingHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    camera = Picamera2()
    camera.configure(
        camera.create_preview_configuration(
            main={"size": (320, 240), "format": "RGB888"}
        )
    )
    camera.start()

    if not sys.stdin.isatty():
        raise RuntimeError("yolo_view requires an interactive SSH/TTY stdin")
    settings = termios.tcgetattr(sys.stdin)
    save_dir = Path(__file__).resolve().parent
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            frame_rgb = camera.capture_array()
            detections = detect_all(session, input_name, frame_rgb, args.confidence)
            # 推理用 RGB；画框/JPEG 用 BGR，颜色常量按 OpenCV 约定
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            annotated = draw_detections(frame_bgr, detections, names)
            stream_frame = cv2.resize(annotated, None, fx=2.0, fy=2.0)

            ok, jpeg = cv2.imencode(
                ".jpg", stream_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            )
            if not ok:
                raise RuntimeError("cv2.imencode failed")
            output.set_frame(jpeg.tobytes())

            key = read_key(0.01)
            if key in ("q", "\x03"):
                break
            if key == "s":
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                path = save_dir / f"yolo_view_{stamp}.jpg"
                if not cv2.imwrite(str(path), annotated):
                    raise RuntimeError(f"failed to write {path}")
                sys.stdout.write(f"\r\nsaved {path}\r\n")
                sys.stdout.flush()
    finally:
        # 清理期间忽略 SIGINT：避免二次 Ctrl-C 打断 httpd.shutdown 导致清理不完整
        old_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            camera.stop()
            httpd.shutdown()
            print("yolo_view exited", flush=True)
        finally:
            signal.signal(signal.SIGINT, old_handler)


if __name__ == "__main__":
    main()
