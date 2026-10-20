from lib.csi_camera import capture_frame
import time
import cv2
import numpy as np
import xgolib
import onnxruntime as ort
from pathlib import Path
from lib.Motion import Motion

# 放置参数
PLACE_PARAMS = {
    "target_x": 0,
    "target_y": -0.9,  # 稍微靠前
    "err_x_tolerance": 0.18,
    "err_y_tolerance": 0.16,
}

def ready_for_place(dog):
    """准备放置姿势"""
    dog.reset()
    dog.attitude("p", 15)
    time.sleep(0.5)


class ABDCLetterDetector:
    """字母检测器（仅识别 A/B/C/D）"""

    def __init__(self, model_path: str, conf_threshold: float = 0.5, input_size: int = 640):
        self.conf_threshold = conf_threshold
        self.input_size = input_size
        self.letters = {0: 'A', 1: 'B', 2: 'C', 3: 'D'}

        if not Path(model_path).exists():
            raise FileNotFoundError(f"ABCD 模型文件不存在: {model_path}")

        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 2
        sess_options.inter_op_num_threads = 1

        self.session = ort.InferenceSession(
            model_path,
            sess_options=sess_options,
            providers=['CPUExecutionProvider']
        )

        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [out.name for out in self.session.get_outputs()]

    def preprocess(self, image: np.ndarray):
        h, w = image.shape[:2]
        ratio = min(self.input_size / w, self.input_size / h)
        new_w, new_h = int(w * ratio), int(h * ratio)
        resized = cv2.resize(image, (new_w, new_h))

        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        pad_x = (self.input_size - new_w) // 2
        pad_y = (self.input_size - new_h) // 2
        canvas[pad_y:pad_y+new_h, pad_x:pad_x+new_w] = resized

        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        input_tensor = np.expand_dims(np.transpose(rgb, (2, 0, 1)), axis=0)

        return input_tensor, (ratio, pad_x, pad_y, w, h)

    def postprocess(self, outputs, scale_info):
        predictions = outputs[0]
        if len(predictions.shape) == 3:
            predictions = predictions[0].T

        ratio, pad_x, pad_y, orig_w, orig_h = scale_info
        detections = []

        for pred in predictions:
            scores = pred[4:]
            class_id = int(np.argmax(scores))
            confidence = float(scores[class_id])
            if confidence < self.conf_threshold or class_id not in self.letters:
                continue

            cx, cy, w, h = pred[0], pred[1], pred[2], pred[3]
            x1 = int(max(0, (cx - w / 2 - pad_x) / ratio))
            y1 = int(max(0, (cy - h / 2 - pad_y) / ratio))
            x2 = int(min(orig_w, (cx + w / 2 - pad_x) / ratio))
            y2 = int(min(orig_h, (cy + h / 2 - pad_y) / ratio))

            detections.append({
                'letter': self.letters[class_id],
                'confidence': confidence,
                'bbox': [x1, y1, x2, y2]
            })

        # 对每个字母只保留最优框
        best = {}
        for det in detections:
            letter = det['letter']
            if letter not in best or det['confidence'] > best[letter]['confidence']:
                best[letter] = det

        return list(best.values())

    def detect(self, image: np.ndarray):
        input_tensor, scale_info = self.preprocess(image)
        outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
        return self.postprocess(outputs, scale_info)


def get_letter_center(detector, frame, target_letter):
    """在图像中找到目标字母的中心位置"""
    detection = detector.detect(frame, target_letter, 0.50)
    if detection is None:
        return None, None
    bbox = detection['box']
    center_x = (bbox[0] + bbox[2]) / 2
    center_y = (bbox[1] + bbox[3]) / 2
    height, width = frame.shape[:2]
    print(f"[地面字母模型] letter={target_letter} confidence={detection['confidence']:.3f} box={bbox}", flush=True)
    return (2 * center_x / width - 1, 1 - 2 * center_y / height), bbox


def keep_center_to_letter(midpoint, dog, control_params):
    """Use the same low-speed pulse controller as pickup alignment."""
    from lib.catch_ball import keep_center
    return keep_center(midpoint, dog, control_params)


def align_letter_target(dog, bbox, frame_width, target_letter):
    x1, y1, x2, y2 = bbox
    scale = 320 / frame_width
    dx = ((x1 + x2) / 2 - frame_width / 2) * scale
    radius = max(x2 - x1, y2 - y1) / 2 * scale
    release_radius = 22 if target_letter == 'A' else 24
    print(f'[放置对准] target={target_letter} dx={dx:.1f} radius={radius:.1f} release_radius={release_radius}', flush=True)
    if abs(dx) > 25:
        command, speed, duration = dog.turn, 15 if dx < 0 else -15, 0.74
    elif radius < release_radius:
        command, speed = dog.move_x, 3
        duration = 1.0 if radius < 18 else 0.15
    else:
        dog.stop()
        return True
    print(f'[放置微步] speed={speed} duration={duration}s', flush=True)
    try:
        command(speed)
        time.sleep(duration)
    finally:
        dog.stop()
    return False


def place(dog):
    """放置小球"""
    dog.translation("x", 20)
    dog.motor(52, -55)
    time.sleep(0.5)
    dog.translation("z", 60)
    time.sleep(0.5)
    dog.motor(53, 80)
    dog.attitude("p", 20)
    time.sleep(2)
    dog.claw(0)  # 打开爪子放置
    time.sleep(2)
    dog.reset()
    time.sleep(1)

def visible_letter_direction(detector, frame, target_letter):
    seen = []
    for letter in 'ABCD':
        if letter == target_letter:
            continue
        result = detector.detect(frame, letter, 0.50)
        if result is not None:
            seen.append((letter, result['confidence']))
    if not seen:
        return None
    letter, confidence = min(seen, key=lambda item: (abs(ord(item[0]) - ord(target_letter)), -item[1]))
    direction = 8 if target_letter < letter else -8
    print(f'[放置定向] 看到{letter}，目标{target_letter}，向{"左" if direction > 0 else "右"}移动', flush=True)
    return direction


def search_letter_step(dog, step_index, direction=None):
    """Four right steps, then eight left/eight right steps repeatedly."""
    if direction is not None:
        speed = direction
        phase = '根据已见字母左移' if direction > 0 else '根据已见字母右移'
    elif step_index < 4:
        speed = -8
        phase = '小范围右移'
    elif ((step_index - 4) // 8) % 2 == 0:
        speed = 8
        phase = '大范围左移'
    else:
        speed = -8
        phase = '大范围右移'
    print(f'[放置搜索] {phase} step={step_index + 1} speed={speed} duration=1.5s', flush=True)
    try:
        dog.move_y(speed)
        time.sleep(1.5)
    finally:
        dog.move_y(0)
    time.sleep(0.2)


def place_ball_to_abcd(dog, motion, target_letter='A', model_path='/home/pi/ros_ws/models/letters.onnx', debug=False):
    """
    将小球放置到指定字母区域

    Args:
        dog: 机器人对象
        motion: 运动控制对象
        target_letter: 目标字母 ('A', 'B', 'C', 'D')
        model_path: ABCD ONNX 模型路径
        debug: 是否开启调试模式
    """
    consecutive_failures = 0
    import sys
    sys.path.insert(0, '/home/pi/oumax-lidar-console')
    from delivery_vision import TargetModel
    detector = TargetModel(model_path, 'ABCD')
    ready_for_place(dog)
    adjustment_complete = False
    aligned_frames = 0
    search_step = 0
    for i in range(120):
        try:
            frame = capture_frame()
        except Exception:
            dog.stop()
            raise
        # if debug:
        #     cv2.imwrite(f"frame_{i}.jpg", frame)

        norm_center, bbox = get_letter_center(detector, frame, target_letter)
        if norm_center is None:
            consecutive_failures += 1
            aligned_frames = 0
            dog.stop()
            print(f"[放置] 未检测到目标字母 {target_letter}，执行横向搜索", flush=True)
            direction = visible_letter_direction(detector, frame, target_letter)
            search_letter_step(dog, search_step, direction)
            if direction is None:
                search_step += 1
            continue

        print(f"第{i+1}次调整，当前字母 {target_letter} 位置: {norm_center}")
        consecutive_failures = 0
        if debug:
            display_frame = frame.copy()
            if bbox:
                cv2.rectangle(display_frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
                cv2.putText(display_frame, target_letter, (int(bbox[0]), int(bbox[1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
            cv2.imshow("Detected Letter", display_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        adjustment_complete = align_letter_target(dog, bbox, frame.shape[1], target_letter)

        if adjustment_complete:
            aligned_frames += 1
            print(f"[放置确认] {aligned_frames}/3", flush=True)
            if aligned_frames >= 3:
                dog.stop()
                time.sleep(0.3)
                place(dog)
                break
        else:
            aligned_frames = 0

    if aligned_frames < 3:
        dog.stop()
        raise RuntimeError("未能视觉对准，已停止放置")

    if debug:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    dog = xgolib.XGO("/dev/ttyAMA0")
    motion = Motion()
    # 默认放置到A，可以改变参数进行调试
    place_ball_to_abcd(dog, motion, target_letter='A', debug=True)
