from lib.csi_camera import capture_frame
import time
import cv2
import numpy as np
import xgolib
from lib.Motion import Motion

EPSILON = 0.02

# OPENCV_RED_LAB_THRESHOLD = (
#     0,
#     255,
#     150,
#     255,
#     33,
#     159,
# )
OPENCV_RED_LAB_THRESHOLD = (
    20,
    255,
    150,
    255,
    20,
    150,
)
# OPENCV_RED_LAB_THRESHOLD = (
#     44,
#     255,
#     138,
#     255,
#     0,
#     255,
# )
OPENCV_GREEN_LAB_THRESHOLD = (
    0,
    143,
    69,
    108,
    79,
    255,
)
OPENCV_BLUE_LAB_THRESHOLD = (
    0,
    255,
    0,
    255,
    0,
    121,
)

COLOR_LAB_THRESHOLD_MAP = {
    "red": OPENCV_RED_LAB_THRESHOLD,
    "green": OPENCV_GREEN_LAB_THRESHOLD,
    "blue": OPENCV_BLUE_LAB_THRESHOLD,
}


def ready_for_grasp(dog):
    """准备抓取姿势"""
    dog.reset()
    dog.attitude("p", 15)
    time.sleep(0.5)


def get_keypoints(approx):
    if len(approx) < 2:
        return None, None

    points = approx.reshape(-1, 2)

    # 按y坐标排序（y值小的在前，即更靠上），然后按x坐标区分左右
    sorted_by_y = points[np.argsort(points[:, 1])]
    top_points = sorted_by_y[:2]  # 取最靠上的两个点
    top_points = top_points[np.argsort(top_points[:, 0])]  # 按x坐标排序

    top_left = top_points[0]  # x坐标较小的为左上
    top_right = top_points[1]  # x坐标较大的为右上

    return top_left, top_right


def calculate_midpoint(left_point, right_point):
    """计算两个点的中点坐标"""
    mid_x = (left_point[0] + right_point[0]) / 2
    mid_y = (left_point[1] + right_point[1]) / 2
    return (mid_x, mid_y)


def get_largest_contour(img, threshold=OPENCV_RED_LAB_THRESHOLD):

    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    binary_mask = cv2.inRange(
        img_lab,
        (threshold[0], threshold[2], threshold[4]),
        (threshold[1], threshold[3], threshold[5]),
    )

    kernel = np.ones((5, 5), np.uint8)
    binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    approx = cv2.approxPolyDP(
        largest_contour, EPSILON * cv2.arcLength(largest_contour, True), True
    )

    return approx


GRASP_BALL_PARAMS = {
    "target_x": 0,
    "target_y": -0.87,
    "err_x_tolerance": 0.12,
    "err_y_tolerance": 0.13,
    "right_speed": -8,
    "lateral_pulse_max": 0.15,
}


def get_largest_ball_center(img, threshold=OPENCV_RED_LAB_THRESHOLD, previous_center=None):
    """在图像中找到颜色阈值内最合适的小球圆心和半径，优先保持上一个目标球。"""
    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    binary_mask = cv2.inRange(
        img_lab,
        (threshold[0], threshold[2], threshold[4]),
        (threshold[1], threshold[3], threshold[5]),
    )

    kernel = np.ones((5, 5), np.uint8)
    binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
    binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 50:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius < 5:
            continue
        candidates.append((int(x), int(y), int(radius), contour, area))

    if not candidates:
        return None

    if previous_center is not None:
        candidates.sort(
            key=lambda item: (item[0] - previous_center[0]) ** 2 + (item[1] - previous_center[1]) ** 2
        )
        return candidates[0][:4]

    img_height, img_width = img.shape[:2]
    img_center = (img_width / 2.0, img_height / 2.0)
    candidates.sort(
        key=lambda item: (
            item[0] - img_center[0]
        ) ** 2 + (item[1] - img_center[1]) ** 2
    )
    return candidates[0][:4]


def grasp(dog):
    dog.claw(0)
    dog.translation("x", 20)
    dog.motor(52, -55)
    time.sleep(0.5)
    dog.translation("z", 60)
    time.sleep(0.5)
    dog.motor(53, 80)
    dog.attitude("p", 20)
    time.sleep(2)
    dog.claw(255)
    time.sleep(2)
    dog.arm(100, 0)
    dog.reset()
    dog.claw(255)
    dog.arm(-90, 90)
    dog.arm_mode(1)
    time.sleep(3)


def place(dog):
    dog.translation("x", 20)
    dog.arm_mode(0)
    dog.motor(52, -55)
    time.sleep(0.5)
    dog.translation("z", 60)
    time.sleep(0.5)
    dog.motor(53, 80)
    dog.attitude("p", 20)
    time.sleep(2)
    dog.claw(0)
    time.sleep(2)
    dog.reset()
    time.sleep(1)


# GRASP_CUBOID_PARAMS = {
#     "target_x": 0,
#     "target_y": -0.1,
#     "err_x_tolerance": 0.05,
#     "err_y_tolerance": 0.06qt.qpa.xcb: could not connect to display
# qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "/home/pi/.local/lib/python3.9/site-packages/cv2/qt/plugins" even though it was found.
# This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

# Available platform plugins are: xcb.,
# }


def keep_center(midpoint, dog, control_params):
    """Align laterally first, then approach with short stop-and-look pulses."""
    x, y = midpoint
    err_x = x - control_params["target_x"]
    err_y = y - control_params["target_y"]
    if abs(err_x) > control_params["err_x_tolerance"]:
        # Leftward command 8 respects the measured lateral starting threshold.
        speed = 8 if err_x < 0 else control_params.get('right_speed', -4)
        duration = min(control_params.get('lateral_pulse_max', 0.22), max(0.08, abs(err_x) * 0.30))
        axis = dog.move_y
    elif abs(err_y) > control_params["err_y_tolerance"]:
        speed = -8 if err_y < 0 else 5
        duration = min(0.25, max(0.08, abs(err_y) * 0.28))
        axis = dog.move_x
    else:
        dog.stop()
        return True
    print(f"[抓球微步] err=({err_x:.3f},{err_y:.3f}) speed={speed} duration={duration:.2f}s", flush=True)
    try:
        axis(speed)
        time.sleep(duration)
    finally:
        axis(0)
    time.sleep(0.20)
    return False


def search_ball_step(dog, index):
    speed = -8 if index < 4 or ((index - 4) // 8) % 2 else 8
    print(f'[找球横移] step={index + 1} speed={speed} duration=0.15s', flush=True)
    try:
        dog.move_y(speed)
        time.sleep(0.15)
    finally:
        dog.stop()
    time.sleep(0.2)


def grasp_ball(
    dog,
    motion,
    threshold=OPENCV_GREEN_LAB_THRESHOLD,
    debug=False,
    target_color=None,
):
    """Follow RAICOM ball_yolo_grab: confirmed target, turn, short approach."""
    import sys
    sys.path.insert(0, '/home/pi/oumax-lidar-console')
    from delivery_vision import TargetModel
    if target_color is None:
        target_color = {
            OPENCV_RED_LAB_THRESHOLD: 'red',
            OPENCV_BLUE_LAB_THRESHOLD: 'blue',
            OPENCV_GREEN_LAB_THRESHOLD: 'green',
        }[threshold]
    detector = TargetModel('/home/pi/ros_ws/models/best.onnx', ('red', 'blue', 'green'))
    pickup_heading = (dog.read_yaw() - motion.start_yaw + 180) % 360 - 180
    print(f'[抓球朝向] 保存找球前朝向 {pickup_heading:.2f}°', flush=True)
    dog.gait_type('slow_trot')
    print('[抓球准备] 先前进 speed=3 duration=1.3s，再趴下识别', flush=True)
    try:
        dog.move_x(3)
        time.sleep(1.3)
    finally:
        dog.stop()
    time.sleep(0.2)
    dog.translation('z', 10)
    dog.attitude('p', 15)
    time.sleep(1)
    confirmed = 0
    search_phase = 'approach'
    search_index = 0
    while True:
        frame = capture_frame()
        detection = detector.detect(frame, target_color, 0.60)
        if detection is None:
            confirmed = 0
            if search_phase == 'approach':
                other_color = next((color for color in ('red', 'blue', 'green')
                                    if color != target_color and detector.detect(frame, color, 0.60) is not None), None)
                if other_color is not None:
                    search_phase = 'scan'
                    print(f'[找球] 看到 {other_color} 球，目标为 {target_color}，开始横移搜索', flush=True)
                else:
                    print('[找球] 尚未看到任何颜色的球，前进3/0.15秒后重看', flush=True)
                    try:
                        dog.move_x(3)
                        time.sleep(0.15)
                    finally:
                        dog.stop()
                    time.sleep(0.2)
                    continue
            if search_phase == 'scan':
                search_ball_step(dog, search_index)
                search_index += 1
                continue
            print('[抓球] 目标球暂时丢失，原地重新识别', flush=True)
            time.sleep(0.05)
            continue
        search_phase = 'align'
        x1, y1, x2, y2 = detection['box']
        center_x = (x1 + x2) / 2
        # Reference parameters are measured on a 320-pixel-wide image.
        scale = 320 / frame.shape[1]
        dx = (center_x - frame.shape[1] / 2) * scale
        radius = max(x2 - x1, y2 - y1) / 2 * scale
        confirmed += 1
        print(f"[抓球] color={target_color} confidence={detection['confidence']:.3f} dx={dx:.1f} radius={radius:.1f} confirmed={confirmed}/3", flush=True)
        if confirmed < 3:
            continue
        if abs(dx) > 25:
            command, speed, duration = dog.turn, 15 if dx < 0 else -15, 0.74
            action = '转向对准'
        elif radius < 24:
            duration = 1.0 if radius < 18 else 0.15
            command, speed = dog.move_x, 3
            action = '远距离前进' if radius < 18 else '近距离微步前进'
        else:
            dog.stop()
            grasp(dog)
            print(f'[抓球朝向] 夹取动作完成，恢复朝向 {pickup_heading:.2f}°', flush=True)
            motion.change_yaw(pickup_heading)
            return
        print(f'[抓球动作] {action} speed={speed} duration={duration}s', flush=True)
        try:
            command(speed)
            time.sleep(duration)
        finally:
            dog.stop()
        confirmed = 0


def get_dominant_color(color_threshold_map, colors):
    dominant_color = None
    max_pixel_number = -1
    assert len(colors) != 0
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    ret, frame = cap.read()
    assert ret
    cap.release()
    lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    for color in colors:
        threshold = color_threshold_map[color]
        binary_mask = cv2.inRange(
            lab_frame,
            (threshold[0], threshold[2], threshold[4]),
            (threshold[1], threshold[3], threshold[5]),
        )
        pixel_number = np.sum(binary_mask == 255)
        if pixel_number > max_pixel_number:
            max_pixel_number = pixel_number
            dominant_color = color
    return dominant_color


if __name__ == "__main__":
    dog = xgolib.XGO("/dev/ttyAMA0")
    motion = Motion()
    # 小球抓取：默认识别绿色小球，可设置为 OPENCV_RED_LAB_THRESHOLD 或 OPENCV_BLUE_LAB_THRESHOLD
    grasp_ball(dog, motion, OPENCV_RED_LAB_THRESHOLD, debug=True)
