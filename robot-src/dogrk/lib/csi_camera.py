"""Capture BGR frames from the CSI camera without opening robot hardware."""
import time
import cv2
from picamera2 import Picamera2


def capture_frame(width=320, height=240):
    camera = Picamera2()
    try:
        camera.configure(camera.create_video_configuration(
            main={'size': (width, height), 'format': 'YUV420'}))
        camera.start()
        time.sleep(0.5)
        image = camera.capture_array('main')
        if image is None or image.size == 0:
            raise RuntimeError('CSI 摄像头未取得有效抓放图像')
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)[:height, :width].copy()
    finally:
        camera.close()
