#!/usr/bin/env python3
"""机器端独立 YOLO 抓球程序：厂商硬件接口 + 项目训练的 ONNX 球模型。"""

import argparse
import time

import cv2
import numpy as np
import onnxruntime as ort
from picamera2 import Picamera2
from uiutils import dog


def letterbox(image, size):
    height, width = image.shape[:2]
    scale = min(size / width, size / height)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x = (size - resized.shape[1]) // 2
    pad_y = (size - resized.shape[0]) // 2
    canvas[pad_y:pad_y + resized.shape[0], pad_x:pad_x + resized.shape[1]] = resized
    return canvas, scale, pad_x, pad_y


def detect_ball(session, input_name, image, confidence):
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
        return None
    boxes = predictions[keep, :4]
    xyxy = np.column_stack((boxes[:, 0] - boxes[:, 2] / 2, boxes[:, 1] - boxes[:, 3] / 2,
                            boxes[:, 0] + boxes[:, 2] / 2, boxes[:, 1] + boxes[:, 3] / 2))
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores[keep].tolist(), confidence, 0.45)
    if indices is None or not len(indices):
        return None
    index = int(max(np.asarray(indices).ravel(), key=lambda position: scores[keep[int(position)]]))
    x1, y1, x2, y2 = xyxy[index]
    x1, x2 = (x1 - pad_x) / scale, (x2 - pad_x) / scale
    y1, y2 = (y1 - pad_y) / scale, (y2 - pad_y) / scale
    return {"cx": (x1 + x2) / 2, "cy": (y1 + y2) / 2,
            "radius": max(x2 - x1, y2 - y1) / 2,
            "confidence": float(scores[keep[index]]), "cls": int(classes[keep[index]])}


def pulse(command, speed, duration, enabled):
    if enabled:
        command(speed)
        time.sleep(duration)
        dog.stop()


def grab(enabled):
    if not enabled:
        return
    print("action=grab-start", flush=True)
    dog.stop()
    dog.translation("z", 10)
    dog.attitude("p", 15)
    dog.attitude("y", -3)
    time.sleep(1)
    dog.motor([31, 41], [26, 25])
    time.sleep(1)
    dog.arm_mode(1)
    for servo, angle, pause in ((51, -65, .5), (52, -50, 1.2), (53, 90, 1.2),
                                (52, -45, 1.2), (51, 40, 1), (53, 0, .8), (52, 0, .8)):
        dog.motor(servo, angle)
        time.sleep(pause)
    dog.attitude("y", 0)
    time.sleep(1)
    print("action=grab-complete", flush=True)


def prepare_approach_pose(enabled):
    """步态设定后低趴接近（z=10/p=15，不加 yaw 补偿）；后肢仅在最终抓取前抬高。

    车体 y=-6 机械臂左偏补偿只在抓球瞬间（grab）使用，接近阶段不得带，
    否则接近时车体会向右偏斜。
    """
    if enabled:
        dog.gait_type("slow_trot")
        dog.translation("z", 10)
        dog.attitude("p", 15)
        time.sleep(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="/home/pi/ros_ws/models/best.onnx")
    parser.add_argument(
        "--target-radius", type=float, default=28.0,
        help="YOLO 框半径达到该像素值后执行抓取；需按实机继续标定",
    )
    parser.add_argument("--confidence", type=float, default=.60)
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--frames", type=int, default=0, help="仅检测 N 帧后退出；0 表示持续运行")
    args = parser.parse_args()
    session = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": (320, 240), "format": "RGB888"}))
    camera.start()
    prepare_approach_pose(args.enable_motion)
    confirmed = 0
    frame_count = 0
    try:
        while not args.frames or frame_count < args.frames:
            frame_count += 1
            ball = detect_ball(session, input_name, camera.capture_array(), args.confidence)
            if ball is None:
                confirmed = 0
                print("YOLO no ball; holding position", flush=True)
                time.sleep(.05)
                continue
            confirmed += 1
            dx = ball["cx"] - 160
            name = ("red", "blue", "green")[ball["cls"]]
            print(
                f"YOLO {name} conf={ball['confidence']:.2f} radius={ball['radius']:.1f} "
                f"dx={dx:.1f} confirmed={confirmed}",
                flush=True,
            )
            if confirmed < 3:
                continue
            if abs(dx) > 25:
                print("action=turn", flush=True)
                pulse(dog.turn, 15 if dx < 0 else -15, .74, args.enable_motion)
            elif ball["radius"] < args.target_radius:
                print("action=forward-step", flush=True)
                pulse(dog.move_x, 3, .15, args.enable_motion)
            else:
                grab(args.enable_motion)
                return
            confirmed = 0
    finally:
        dog.stop()
        camera.stop()


if __name__ == "__main__":
    main()
