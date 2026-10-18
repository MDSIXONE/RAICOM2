"""Web-console friendly black-line detector and PID controller.

The original follow-line program owns Picamera2, the XGO serial port and the
LCD.  The web console already owns the camera stream and sends motion through
the manual-control service, so this module contains only the reusable vision
and PID part.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2 as cv
import numpy as np
from motion_limits import MIN_VX, MIN_YAW_SETTING, MIN_YAW_LEFT, MIN_YAW_RIGHT, motor_floor, migrate_speed_floor


PID_SCALE = 1000.0


DEFAULT_LINE_CONFIG: dict[str, Any] = {
    "lower": [0, 0, 0],
    "upper": [180, 255, 30],
    "crop_top": 150,
    "crop": [0, 319],
    "line_width": [5, 100],
    "pid_p": 396.0,
    "pid_i": 0.0,
    "pid_d": 30.0,
    "straight_speed": 8.0,
    "turn_move_speed": 6.0,
    "turn_speed_min": 16.0,
    "turn_speed_max": 16.0,
    "direction": 1.0,
    "control_hz": 12.0,
    "lost_confirm_frames": 5,
    "probe_steps": 6,
    "probe_step_duration": 1.167,
    "probe_pause": 0.5,
    "probe_turn_speed": -16.0,
    "probe_cooldown": 5.0,
    "surge_line_width": 75.0,
    "surge_frames": 3,
    "surge_turn_duration": 4.2,
    "surge_turn_speed": -16.0,
}


def _copy_default_config() -> dict[str, Any]:
    return json.loads(json.dumps(DEFAULT_LINE_CONFIG))


def _as_int_triplet(value: Any, name: str) -> list[int]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} 必须是三个整数")
    result = [int(item) for item in value]
    if not 0 <= result[0] <= 180 or any(item < 0 or item > 255 for item in result[1:]):
        raise ValueError(f"{name} 必须满足 H=0~180、S/V=0~255")
    return result


def validate_line_config(raw: dict[str, Any]) -> dict[str, Any]:
    config = _copy_default_config()
    config.update(raw)
    config["lower"] = _as_int_triplet(config["lower"], "HSV 下限")
    config["upper"] = _as_int_triplet(config["upper"], "HSV 上限")
    if any(a > b for a, b in zip(config["lower"], config["upper"])):
        raise ValueError("HSV 下限不能大于上限")

    crop_top = int(config["crop_top"])
    if not 0 <= crop_top < 240:
        raise ValueError("上方裁剪边界必须在 0~239 之间")
    config["crop_top"] = crop_top

    crop = [int(item) for item in config["crop"]]
    if len(crop) != 2 or not (0 <= crop[0] < crop[1] <= 319):
        raise ValueError("左右裁剪必须满足 0 <= 左边界 < 右边界 <= 319")
    config["crop"] = crop

    line_width = [int(item) for item in config["line_width"]]
    if len(line_width) != 2 or not (0 < line_width[0] < line_width[1] <= 300):
        raise ValueError("线宽过滤必须满足 0 < 最小值 < 最大值 <= 300")
    config["line_width"] = line_width

    for key in (
        "pid_p",
        "pid_i",
        "pid_d",
        "straight_speed",
        "turn_move_speed",
        "turn_speed_min",
        "turn_speed_max",
        "direction",
        "control_hz",
        "probe_step_duration",
        "probe_pause",
        "probe_turn_speed",
        "probe_cooldown",
        "surge_line_width",
        "surge_turn_duration",
        "surge_turn_speed",
    ):
        config[key] = float(config[key])
    for key in (
        "lost_confirm_frames",
        "probe_steps",
        "surge_frames",
    ):
        config[key] = int(config[key])

    if config["pid_p"] < 0 or config["pid_i"] < 0 or config["pid_d"] < 0:
        raise ValueError("PID 参数不能为负数")
    if not (MIN_VX <= config["straight_speed"] <= 20):
        raise ValueError(f"直行速度必须在 {MIN_VX}~20 之间")
    if not (MIN_VX <= config["turn_move_speed"] <= 20):
        raise ValueError(f"转向后前进速度必须在 {MIN_VX}~20 之间")
    if not (MIN_YAW_SETTING <= config["turn_speed_min"] <= config["turn_speed_max"] <= 60):
        raise ValueError("转向速度范围无效")
    for key in ("probe_turn_speed", "surge_turn_speed"):
        value = config[key]
        if abs(value) > 60 or motor_floor(value, MIN_YAW_LEFT, MIN_YAW_RIGHT) != value:
            raise ValueError(f"{key} 不能落入实测转向死区（右转至少 14，左转至少 16，0 为不转向）")
    if config["direction"] not in (-1.0, 1.0):
        raise ValueError("转向方向只能是 1 或 -1")
    if not (2 <= config["control_hz"] <= 20):
        raise ValueError("巡线频率必须在 2~20 Hz 之间")
    if config["lost_confirm_frames"] < 1 or config["probe_steps"] < 1:
        raise ValueError("丢线确认帧数和探测格数必须为正数")
    if config["surge_frames"] < 1 or config["surge_line_width"] <= 0:
        raise ValueError("线宽突变参数无效")
    return config


def load_line_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return _copy_default_config()
    with path.open("r", encoding="utf-8") as stream:
        raw = json.load(stream)
    if not isinstance(raw, dict):
        raise ValueError(f"{path.name} 必须是 JSON 对象")
    raw = {**_copy_default_config(), **raw}
    for key, minimum in (("straight_speed", MIN_VX), ("turn_move_speed", MIN_VX), ("turn_speed_min", MIN_YAW_SETTING), ("turn_speed_max", MIN_YAW_SETTING)):
        migrate_speed_floor(raw, key, minimum)
    for key in ("probe_turn_speed", "surge_turn_speed"):
        old = float(raw[key])
        corrected = motor_floor(old, MIN_YAW_LEFT, MIN_YAW_RIGHT)
        if corrected != old:
            print(f"校准升级 {key}: {old} -> {corrected}", flush=True)
            raw[key] = corrected
    return validate_line_config(raw)


def save_line_config(path: Path, raw: dict[str, Any]) -> dict[str, Any]:
    config = validate_line_config(raw)
    path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return config


class SimplePID:
    def __init__(self, p: float, i: float, d: float) -> None:
        self.p = float(p)
        self.i = float(i)
        self.d = float(d)
        self.last_error = 0.0
        self.integrator = 0.0
        self.last_time: float | None = None

    def reset(self) -> None:
        self.last_error = 0.0
        self.integrator = 0.0
        self.last_time = None

    def update(self, error: float) -> float:
        now = time.perf_counter()
        if self.last_time is None:
            self.last_time = now
            return 0.0
        delta = now - self.last_time
        if delta > 0.5:
            self.reset()
            self.last_time = now
            return 0.0
        if delta <= 0.0:
            raise RuntimeError("巡线 PID 时间间隔无效")
        self.integrator += error * delta
        derivative = (error - self.last_error) / delta
        self.last_error = error
        self.last_time = now
        return self.p * error + self.i * self.integrator + self.d * derivative


@dataclass(frozen=True)
class LineResult:
    frame: np.ndarray
    binary: np.ndarray
    found: bool
    center_x: float | None
    center_y: float | None
    radius: float | None
    line_width: float
    raw_line_width: float
    area: float
    pid: float


class LineProcessor:
    """Detect the same black line used by the standalone follow_line script."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = validate_line_config(config)
        self.pid = SimplePID(
            self.config["pid_p"] / PID_SCALE,
            self.config["pid_i"] / PID_SCALE,
            self.config["pid_d"] / PID_SCALE,
        )

    def update_config(self, config: dict[str, Any]) -> None:
        self.config = validate_line_config(config)
        self.pid.p = self.config["pid_p"] / PID_SCALE
        self.pid.i = self.config["pid_i"] / PID_SCALE
        self.pid.d = self.config["pid_d"] / PID_SCALE
        self.pid.reset()

    def reset(self) -> None:
        self.pid.reset()

    def process(self, frame: np.ndarray) -> LineResult:
        image = cv.resize(frame, (320, 240))
        height, width = image.shape[:2]
        masked_image = image.copy()
        crop_top = self.config["crop_top"]
        masked_image[:crop_top, :] = 0

        crop_left, crop_right = self.config["crop"]
        if crop_left > 0:
            masked_image[:, :crop_left] = 0
        if crop_right < width:
            masked_image[:, crop_right:] = 0

        hsv = cv.cvtColor(masked_image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(
            hsv,
            np.array(self.config["lower"], dtype=np.uint8),
            np.array(self.config["upper"], dtype=np.uint8),
        )
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # The old standalone script converted the masked HSV image to gray
        # before thresholding.  That discards a pure black pixel (V=0), so a
        # valid black line can never become a contour.  The HSV mask is already
        # a binary image; close it directly and preserve the intended detector.
        binary = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        contours_info = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours = contours_info[1] if len(contours_info) == 3 else contours_info[0]
        line_min, line_max = self.config["line_width"]
        best = None
        best_area = 0.0
        best_width = 0.0
        raw_width = 0.0
        raw_area = 0.0
        for candidate in contours:
            rect = cv.minAreaRect(candidate)
            box_width, box_height = rect[1]
            width_px = min(box_width, box_height)
            area = float(cv.contourArea(candidate))
            if area > raw_area:
                raw_area = area
                raw_width = width_px
            if line_min <= width_px <= line_max and area > best_area:
                best = candidate
                best_area = area
                best_width = width_px

        annotated = image.copy()
        cv.line(annotated, (0, crop_top), (width - 1, crop_top), (255, 0, 255), 1)
        cv.line(annotated, (crop_left, 0), (crop_left, height - 1), (255, 0, 255), 1)
        cv.line(annotated, (crop_right, 0), (crop_right, height - 1), (255, 0, 255), 1)
        center_x: float | None = None
        center_y: float | None = None
        radius: float | None = None
        pid_value = 0.0
        if best is None:
            self.pid.reset()
            cv.putText(
                annotated,
                "LINE LOST",
                (4, 20),
                cv.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
        else:
            rect = cv.minAreaRect(best)
            box = np.intp(cv.boxPoints(rect))
            (center_x, center_y), radius = cv.minEnclosingCircle(box)
            crop_center = (crop_left + crop_right) / 2.0
            pid_value = self.config["direction"] * self.pid.update(center_x - crop_center)
            cv.polylines(annotated, [box], True, (255, 0, 0), 2)
            cv.circle(annotated, (int(center_x), int(center_y)), 5, (255, 0, 255), -1)
            cv.putText(
                annotated,
                f"cx={center_x:.0f} PID={pid_value:.1f} LW={best_width:.0f}",
                (4, 20),
                cv.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv.LINE_AA,
            )
        cv.putText(
            annotated,
            f"FOLLOW V={self.config['straight_speed']:.1f}/{self.config['turn_move_speed']:.1f}",
            (4, 236),
            cv.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 0),
            1,
            cv.LINE_AA,
        )
        return LineResult(
            frame=annotated,
            binary=binary,
            found=best is not None,
            center_x=center_x,
            center_y=center_y,
            radius=radius,
            line_width=best_width,
            raw_line_width=raw_width,
            area=best_area,
            pid=float(pid_value),
        )


def make_mask_preview(binary: np.ndarray, crop_top: int | None = None) -> np.ndarray:
    preview = np.zeros((binary.shape[0], binary.shape[1], 3), dtype=np.uint8)
    preview[binary > 0] = (0, 0, 255)
    if crop_top is not None:
        cv.line(preview, (0, int(crop_top)), (binary.shape[1] - 1, int(crop_top)), (255, 0, 255), 1)
    cv.putText(
        preview,
        "BLACK LINE MASK",
        (4, 20),
        cv.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 0),
        1,
        cv.LINE_AA,
    )
    return preview
