#!/usr/bin/env python3
"""机器端放球程序：夹球到达目标区域后，低趴 → 视觉跟踪对齐 → 张开爪子释放小球。

与 robot_dog_ball_grab 的 ball_yolo_grab.py 同源：低趴后复用与抓球一致的
YOLO 检测跟踪对齐（转向/前进脉冲，目标框半径达标才继续），对齐完成后再执行
放球序列（抓球序列末端状态的逆操作）。

视觉模型先用球模型（best.onnx）代替：将来目标区域对齐应换成字母模型
（/home/pi/ros_ws/models/letters.onnx）。
"""

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


def prepare_low_pose(enabled):
    """步态设定后低趴车体（z=10/p=15，不加 yaw 补偿），等待视觉跟踪对齐。

    与抓球接近阶段一致：对齐/接近阶段不加车体 y 补偿，只在放球瞬间（drop_ball）使用。
    """
    if enabled:
        dog.gait_type("slow_trot")
        dog.translation("z", 10)
        dog.attitude("p", 15)
        time.sleep(1)


def align_to_target(enabled, session, input_name, camera, confidence, target_radius,
                    align_timeout):
    """与抓球接近阶段一致的视觉跟踪对齐：转向/前进脉冲，框半径达标才返回。

    超过 align_timeout 秒仍未达标时输出 action=align-timeout 并返回
    （放球时球已在爪内，球模型可能检测不到；超时后仍需完成放球动作）。
    """
    confirmed = 0
    start_time = time.monotonic()
    while not align_timeout or time.monotonic() - start_time < align_timeout:
        ball = detect_ball(session, input_name, camera.capture_array(), confidence)
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
            pulse(dog.turn, 15 if dx < 0 else -15, .74, enabled)
        elif ball["radius"] < target_radius:
            print("action=forward-step", flush=True)
            pulse(dog.move_x, 3, .15, enabled)
        else:
            print("action=align-complete", flush=True)
            return
        confirmed = 0
    print("action=align-timeout", flush=True)


def drop_ball(enabled, claw_open, hold_after_open, drop_yaw):
    """对齐达标后放球：重建低趴 → 抬后肢 → 安全伸臂（持球）→ 张爪放球 → 收臂 → yaw 复原。

    对齐阶段的转向/前进脉冲可能让车体恢复站姿，故放球前必须重新低趴。
    """
    if not enabled:
        return
    print("action=release-start", flush=True)
    dog.stop()
    # 1. 重新低趴车体：低趴指令会复位机械臂姿态，必须先于机械臂序列发送。
    #    放球瞬间使用与抓球同款的车体 y 补偿（drop_yaw=-3），补偿机械臂左偏。
    dog.translation("z", 10)
    dog.attitude("p", 15)
    dog.attitude("y", drop_yaw)
    time.sleep(1)
    # 2. 抬高后肢（左右后小腿同步下发）
    dog.motor([31, 41], [26, 25])
    time.sleep(1)
    # 3. 机械臂安全伸出（爪保持闭合持球）：先小臂避让摄像头路径 → 大臂前抬 → 小臂到位
    dog.arm_mode(1)
    for servo, angle, pause in ((52, -50, 1.2), (53, 90, 1.2), (52, -45, 1.2)):
        dog.motor(servo, angle)
        time.sleep(pause)
    # 4. 张开爪子放球
    dog.motor(51, claw_open)
    time.sleep(hold_after_open)
    # 5. 收臂：先收大臂，再收小臂（顺序反了爪子会扫摄像头）
    for servo, angle, pause in ((53, 0, .8), (52, 0, .8)):
        dog.motor(servo, angle)
        time.sleep(pause)
    # 6. 车体 yaw 补偿复原
    dog.attitude("y", 0)
    time.sleep(1)
    print("action=release-complete", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="/home/pi/ros_ws/models/best.onnx",
                        help="跟踪对齐用模型；当前先用球模型，后续换字母模型 letters.onnx")
    parser.add_argument(
        "--target-radius", type=float, default=28.0,
        help="YOLO 框半径达到该像素值后结束对齐并放球；需按实机继续标定",
    )
    parser.add_argument("--confidence", type=float, default=.60)
    parser.add_argument(
        "--align-timeout", type=float, default=60.0,
        help="视觉跟踪对齐超时秒数；0 表示不超时。放球时球已在爪内、球模型可能检测不到，"
             "超时后仍继续放球",
    )
    parser.add_argument(
        "--drop-yaw", type=float, default=-3.0,
        help="放球瞬间低趴时的车体 y 补偿角度；默认 -3（与抓球同款机械臂左偏补偿）。"
             "对齐接近阶段不加补偿",
    )
    parser.add_argument(
        "--claw-open", type=float, default=-65.0,
        help="放球时 51 号爪子张开角度；-65 为实际下限（最大张开，与抓球准备同款）；"
             "若需让球落在更近处可调小（如 -50）",
    )
    parser.add_argument(
        "--hold-after-open", type=float, default=1.0,
        help="张爪后等待秒数，让球完全落下；球卡爪时可增大",
    )
    parser.add_argument("--enable-motion", action="store_true",
                        help="显式开启才允许真实运动")
    args = parser.parse_args()
    session = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": (320, 240), "format": "RGB888"}))
    camera.start()
    try:
        prepare_low_pose(args.enable_motion)
        align_to_target(args.enable_motion, session, input_name, camera,
                        args.confidence, args.target_radius, args.align_timeout)
        drop_ball(args.enable_motion, args.claw_open, args.hold_after_open, args.drop_yaw)
    finally:
        dog.stop()
        camera.stop()


if __name__ == "__main__":
    main()
