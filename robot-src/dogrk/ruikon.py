import cv2
import time
import xgolib




from lib.Motion import Motion
from lib.ruikon_detect import detect
from lib.ruikon_sound import RuiKonSound
from lib.dog1_socket import send_meter_statuses
from lib.sound import Sound
from lib.catch_ball import grasp_ball, OPENCV_RED_LAB_THRESHOLD, OPENCV_GREEN_LAB_THRESHOLD, OPENCV_BLUE_LAB_THRESHOLD
from lib.place_abcd import place_ball_to_abcd




# ---------------- 初始化 ----------------
dog = xgolib.XGO("/dev/ttyAMA0")
dog.reset()
time.sleep(1)
move = Motion()

# 收集识别到的仪表状态，最后统一发送给狗2
meter_states = {}

# 收集识别到的区域和对应球颜色，例如 [('A', 'blue'), ('B', 'red')]
recognized_results = []

ITEM_NAME_MAP = {
    'clothe': '衣服',
    'paper': '卫生纸',
    'toothbrush': '牙刷',
    'apple': '苹果',
    'orange': '橘子',
    'banana': '香蕉',
}
COLOR_GROUPS = {
    'red': {'clothe', 'paper', 'toothbrush'},
    'blue': {'apple', 'orange', 'banana'},
}


def classify_recognized_item(item_name: str):
    normalized_name = (item_name or '').strip().lower()
    if normalized_name in COLOR_GROUPS['red']:
        return 'red', ITEM_NAME_MAP.get(normalized_name, item_name)
    if normalized_name in COLOR_GROUPS['blue']:
        return 'blue', ITEM_NAME_MAP.get(normalized_name, item_name)
    return None, item_name


def collect_recognized_results(result: dict):
    global recognized_results
    if not isinstance(result, dict):
        return recognized_results

    for area, item_name in sorted(result.items()):
        color, _ = classify_recognized_item(item_name)
        if color is None:
            continue

        record = (area, color)
        recognized_results = [r for r in recognized_results if r[0] != area]
        recognized_results.append(record)

    if recognized_results:
        print(f"[记录] 当前已收集识别结果 -> {recognized_results}")

    return recognized_results


def get_grouped_recognized_results():
    red_areas = [area for area, color in recognized_results if color == 'red']
    blue_areas = [area for area, color in recognized_results if color == 'blue']
    return {'red': red_areas, 'blue': blue_areas}


def detect_and_announce():
    """使用主任务识别区模型识别真实字母、物品及球颜色。"""
    result = detect()
    if not result:
        print("[识别] 未识别到字母，取消播报。")
        return {}

    configured_result = result
    collect_recognized_results(configured_result)

    for area, item in sorted(configured_result.items()):
        try:
            print(f"[识别] {area} 区域，识别物品: {item}")
            RuiKonSound.play(area, item)
        except Exception as exc:
            print(f"[播报] 播报失败: {area} {item} -> {exc}")

    return configured_result


def send_all_states():
    all_states = {k: meter_states.get(k, 'Unknown') for k in ['A', 'B', 'C', 'D']}
    missing = sorted([k for k, v in all_states.items() if v == 'Unknown'])
    ok = send_meter_statuses(all_states, host="10.44.232.14", port=9999, robot_id="robot_1")
    if ok:
        if missing:
            print(f"[通知] 已发送当前识别结果，缺失区域: {missing}")
        else:
            print("[通知] 已将四个仪表状态一次性发送给狗2")
    else:
        print("[通知] 发送失败")



print("--- 2026国赛 狗一任务开始 ---")


dog.move_x(20)
time.sleep(4.8)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(-45)
# dog.move_y(-20)
# time.sleep(2)
# dog.move_y(0)
# time.sleep(0.5)
detect_and_announce()

time.sleep(1)
move.change_yaw(0)
time.sleep(0.5)
dog.move_x(20)
time.sleep(4.0)
dog.move_x(0)
time.sleep(0.5)
# # dog_go(move, 20, 4)
# dog.move_x(20)
# time.sleep(4)
# dog.move_x(0)
# time.sleep(0.5)
move.change_yaw(0)
time.sleep(0.5)
move.change_yaw(-90)
time.sleep(0.5)
move.change_yaw(-90)
time.sleep(0.5)
dog.move_x(20)
time.sleep(1.2)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(180)
time.sleep(0.5)
detect_and_announce()
time.sleep(1)
move.change_yaw(-90)
time.sleep(0.5)
move.change_yaw(-90)
time.sleep(0.5)
dog.move_x(20)
time.sleep(2.8)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(180)
detect_and_announce()
time.sleep(1)
move.change_yaw(-110)
time.sleep(0.5)
move.change_yaw(-110)
time.sleep(0.5)
dog.move_x(20)
time.sleep(1.5)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(180)
time.sleep(0.5)
move.change_yaw(180)
time.sleep(0.5)
dog.move_x(20)
time.sleep(3.2)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(90)
time.sleep(0.5)
move.change_yaw(90)
time.sleep(0.5)
dog.move_x(20)
time.sleep(1.0)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(0)
time.sleep(0.5)
move.change_yaw(0)
time.sleep(0.5)

detect_and_announce()
time.sleep(1)

move.change_yaw(90)
time.sleep(0.5)
move.change_yaw(90)
time.sleep(0.5)
dog.move_x(20)
time.sleep(1.5)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(180)
time.sleep(0.5)
move.change_yaw(180)
time.sleep(0.5)
dog.move_x(20)
time.sleep(2.2)
dog.move_x(0)
time.sleep(0.5)
move.change_yaw(-90)
time.sleep(0.5)
move.change_yaw(-90)
time.sleep(0.5)


print(f"[记录] 最终收集到的识别结果 -> {recognized_results}")
if len(recognized_results) < 2:
    dog.stop()
    raise RuntimeError(f'识别目标不足2个，停止配送：{recognized_results}')
a1,b1 = recognized_results[0]
if b1 == 'red':
    grasp_ball(dog, move, target_color='red', debug=False)
    move.change_yaw(-90)
    time.sleep(0.5)
    move.change_yaw(90)
    time.sleep(0.5)
    dog.move_x(20)
    time.sleep(1.5)
    dog.move_x(0)
    time.sleep(0.5)
    place_ball_to_abcd(dog, move, a1,debug=False)
    move.change_yaw(-90)
    dog.move_x(20)
    time.sleep(0.8)
    dog.move_x(0)
    time.sleep(0.5)

    if a1=='A':
        dog.move_y(20)
        time.sleep(1.5)
        dog.move_y(0)
        time.sleep(0.5)
    if a1=='D':
        dog.move_y(-20)
        time.sleep(1.5)
        dog.move_y(0)
        time.sleep(0.5)
    move.change_yaw(-90)
    time.sleep(0.5)

else:
    grasp_ball(dog, move, target_color='blue', debug=False)
    move.change_yaw(-90)
    time.sleep(0.5)
    move.change_yaw(90)
    time.sleep(0.5)
    dog.move_x(20)
    time.sleep(1.5)
    dog.move_x(0)
    time.sleep(0.5)
    place_ball_to_abcd(dog, move, a1,debug=False)
    move.change_yaw(-90)
    dog.move_x(20)
    time.sleep(0.8)
    dog.move_x(0)
    time.sleep(0.5)

    if a1=='A':
        dog.move_y(20)
        time.sleep(1.5)
        dog.move_y(0)
        time.sleep(0.5)
    if a1=='D':
        dog.move_y(-20)
        time.sleep(1.5)
        dog.move_y(0)
        time.sleep(0.5)
    move.change_yaw(-90)
    time.sleep(0.5)

a2,b2 = recognized_results[1]
if b2 == 'red':
    grasp_ball(dog, move, target_color='red', debug=False)
    move.change_yaw(-90)
    time.sleep(0.5)
    move.change_yaw(90)
    time.sleep(0.5)
    dog.move_x(20)
    time.sleep(1.5)
    dog.move_x(0)
    time.sleep(0.5)
    place_ball_to_abcd(dog, move, a2,debug=False)
else:
    grasp_ball(dog, move, target_color='blue', debug=False)
    move.change_yaw(-90)
    time.sleep(0.5)
    move.change_yaw(90)
    time.sleep(0.5)
    dog.move_x(20)
    time.sleep(1.5)
    dog.move_x(0)
    time.sleep(0.5)
    place_ball_to_abcd(dog, move, a2,debug=False)



# while True:
#     send_all_states()

# detect()

'''
while True:
    collect_state('A')
'''
