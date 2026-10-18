#!/usr/bin/env python3
"""Save 100 camera frames at 0.4-second intervals, continuing existing numbers."""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
import urllib.request


class CameraFrames:
    def __init__(self, url):
        self.url = url
        self.condition = threading.Condition()
        self.latest = None
        self.error = None
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._read, daemon=True)

    def _read(self):
        try:
            with urllib.request.urlopen(self.url, timeout=5) as response:
                if 'multipart/x-mixed-replace' not in response.headers.get('Content-Type', ''):
                    raise RuntimeError('Camera URL did not return an MJPEG stream')
                while not self.stopped.is_set():
                    line = response.readline()
                    if not line:
                        raise RuntimeError('Camera stream closed')
                    if not line.startswith(b'--'):
                        continue
                    length = None
                    while True:
                        header = response.readline()
                        if not header:
                            raise RuntimeError('Camera frame headers ended unexpectedly')
                        if header in (b'\r\n', b'\n'):
                            break
                        key, value = header.decode('ascii').split(':', 1)
                        if key.lower() == 'content-length':
                            length = int(value.strip())
                    if length is None or length <= 0:
                        raise RuntimeError('Camera frame has no valid Content-Length')
                    jpeg = response.read(length)
                    if len(jpeg) != length or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                        raise RuntimeError('Camera returned an incomplete JPEG frame')
                    with self.condition:
                        self.latest = (time.monotonic(), time.time(), jpeg)
                        self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error = error
                self.condition.notify_all()

    def frame_after(self, earliest):
        timeout_at = time.monotonic() + 10
        with self.condition:
            while True:
                if self.error is not None:
                    raise RuntimeError('Camera stream failed') from self.error
                if self.latest is not None and self.latest[0] >= earliest:
                    return self.latest
                remaining = timeout_at - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('No fresh camera frame within 10 seconds')
                self.condition.wait(remaining)


def next_number(directory):
    numbers = [int(path.stem) for path in directory.glob('*.jpg') if path.stem.isdecimal()]
    return max(numbers, default=-1) + 1


def capture_batch(directory, frames, count=100, interval=0.4, clock=time.monotonic, sleep=time.sleep):
    start = next_number(directory)
    metadata = []
    next_capture = clock()
    earliest = next_capture
    for number in range(start, start + count):
        sleep(max(0, next_capture - clock()))
        stamp, wall_time, jpeg = frames.frame_after(earliest)
        captured_at = clock()
        filename = directory / f'{number}.jpg'
        # Exclusive creation also protects against unexpected concurrent writers.
        with filename.open('xb') as output:
            output.write(jpeg)
            output.flush()
            os.fsync(output.fileno())
        metadata.append({'number': number, 'file': filename.name, 'received_at': wall_time,
                         'elapsed_seconds': captured_at - metadata[0]['monotonic'] if metadata else 0,
                         'monotonic': captured_at})
        print(f'[{len(metadata)}/{count}] {filename}', flush=True)
        # Sample the latest fresh frame on a 0.4s timer, rather than rounding
        # each interval up to the next 10fps camera arrival.
        next_capture = captured_at + interval
        earliest = stamp + 1e-9
    for entry in metadata:
        del entry['monotonic']
    with (directory / f'batch-{start}-{start+count-1}.json').open('x', encoding='utf-8') as output:
        json.dump({'count': count, 'interval_seconds': interval, 'frames': metadata}, output, indent=2)
    return start, start + count - 1


def main():
    parser = argparse.ArgumentParser(description='每次连拍100张，间隔0.4秒，编号自动续接。')
    parser.add_argument('--output', type=Path, default=Path.home() / 'robot-photos', help='照片保存目录')
    parser.add_argument('--url', default='http://127.0.0.1:8088/camera.mjpg', help='现有摄像头 MJPEG 地址')
    parser.add_argument('--count', type=int, default=100, help='照片数量，默认100')
    parser.add_argument('--interval', type=float, default=0.4, help='间隔秒数，默认0.4')
    args = parser.parse_args()
    if args.count <= 0 or not math.isfinite(args.interval) or args.interval <= 0:
        parser.error('count 和 interval 必须为正数')
    args.output.mkdir(parents=True, exist_ok=True)
    # Linux flock releases automatically on exit, including interruption/crash.
    import fcntl
    with (args.output / '.capture.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        camera = CameraFrames(args.url)
        camera.thread.start()
        try:
            start, end = capture_batch(args.output, camera, args.count, args.interval)
            print(f'完成：{start}–{end}，保存至 {args.output.resolve()}', flush=True)
        finally:
            camera.stopped.set()


if __name__ == '__main__':
    main()
