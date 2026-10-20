"""Use the primary recognition-area model and CSI camera pipeline."""
import sys
import time
from pathlib import Path

import cv2
import onnxruntime as ort
from picamera2 import Picamera2

sys.path.insert(0, '/home/pi/oumax-lidar-console')
from recognize_at_point import class_names, run_frame, choose_best

MODEL = Path('/home/pi/ros_ws/models/model.onnx')
ITEM_NAMES = {'shirt': 'clothe', 'toilet_paper': 'paper'}


def detect():
    session = ort.InferenceSession(str(MODEL), providers=['CPUExecutionProvider'])
    info = session.get_inputs()[0]
    size = int(info.shape[-1])
    names = class_names(session)
    results = []
    camera = Picamera2()
    try:
        camera.configure(camera.create_video_configuration(main={'size': (640, 480), 'format': 'YUV420'}))
        camera.start()
        time.sleep(1)
        for _ in range(3):
            image = camera.capture_array('main')
            if image is None or image.size == 0:
                raise RuntimeError('Picamera2 未取得有效识别图像')
            image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
            result, detections, _ = run_frame(session, image, names, info.name, size, 0.25, 0.45,
                                               item_threshold=0.30, letter_threshold=0.20)
            results.append(result)
            time.sleep(0.2)
    finally:
        camera.close()
    best = choose_best(results)
    print(f'[识别区] {best}', flush=True)
    if not best['letter'] or not best['item']:
        return {}
    return {best['letter']: ITEM_NAMES.get(best['item'], best['item'])}
