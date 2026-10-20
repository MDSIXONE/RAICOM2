"""Read-only target detection over the console's shared camera stream."""
import argparse
import ast
import json
import time
from pathlib import Path

from capture_photos import CameraFrames
from recognize_at_point import run_frame


class TargetModel:
    def __init__(self, path, expected):
        import onnxruntime as ort
        self.session = ort.InferenceSession(str(path), providers=['CPUExecutionProvider'])
        self.names = {int(k): v for k, v in ast.literal_eval(self.session.get_modelmeta().custom_metadata_map['names']).items()}
        if set(self.names.values()) != set(expected):
            raise ValueError(f'模型类别错误: {path}: {self.names}')
        info = self.session.get_inputs()[0]
        self.input_name, self.input_size = info.name, int(info.shape[-1])

    def detect(self, image, target, confidence):
        if target not in self.names.values():
            raise ValueError(f'目标 {target} 不在模型类别中')
        _, detections, _ = run_frame(self.session, image, self.names, self.input_name,
                                      self.input_size, confidence, 0.45)
        matches = [d for d in detections if d['class'] == target and d['confidence'] >= confidence]
        if not matches:
            return None
        result = dict(max(matches, key=lambda d: d['confidence']))
        height, width = image.shape[:2]
        box = result['box']
        result.update(target=target, cx_norm=(box[0] + box[2]) / 2 / width,
                      radius_ratio=max(box[2] - box[0], box[3] - box[1]) / 2 / width,
                      frame_width=width, frame_height=height)
        return result


class SharedVision:
    def __init__(self, camera_url, ball_model, letter_model, confidence=0.5):
        self.models = {'ball': TargetModel(ball_model, ('red', 'blue', 'green')),
                       'letter': TargetModel(letter_model, 'ABCD')}
        self.confidence = confidence
        self.camera = CameraFrames(camera_url)
        self.earliest = time.monotonic()
        self.camera.thread.start()

    def detect(self, kind, target):
        import cv2
        import numpy as np
        # Require a fresh frame after stopping, never reuse one taken while
        # strafing. Inference runs while the base is stationary.
        stamp, _, jpeg = self.camera.frame_after(max(self.earliest, time.monotonic()))
        self.earliest = stamp + 1e-9
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('摄像头帧无法解码')
        return self.models[kind].detect(image, target, self.confidence)

    def close(self):
        self.camera.stopped.set()


def main(argv=None):
    import cv2
    parser = argparse.ArgumentParser(description='检测指定球颜色或 A/B/C/D，不驱动机器人')
    parser.add_argument('--model', required=True)
    parser.add_argument('--target', choices=['red', 'blue', *'ABCD'], required=True)
    parser.add_argument('--image', required=True)
    parser.add_argument('--confidence', type=float, default=0.5)
    args = parser.parse_args(argv)
    model = TargetModel(Path(args.model), 'ABCD' if args.target in 'ABCD' else ('red', 'blue', 'green'))
    image = cv2.imread(args.image)
    if image is None:
        raise ValueError(f'无法读取 {args.image}')
    result = model.detect(image, args.target, args.confidence)
    print(json.dumps({'ok': result is not None, 'detection': result}, ensure_ascii=False))
    return 0 if result else 2


if __name__ == '__main__':
    raise SystemExit(main())
