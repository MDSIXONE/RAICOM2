#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import io
import json
import math
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any

from aiohttp import web
from motion_limits import (
    MIN_VX, MIN_VY_SETTING, MIN_YAW_SETTING, MIN_YAW_LEFT, MIN_YAW_RIGHT,
    MOTION_LIMITS, enforce_axes, motor_floor, migrate_speed_floor,
)

try:
    import numpy as np  # type: ignore
    from scipy.spatial import cKDTree  # type: ignore
except Exception as exc:  # pragma: no cover - optional numeric stack
    np = None
    cKDTree = None
    LOCALIZATION_IMPORT_ERROR = str(exc)
else:
    LOCALIZATION_IMPORT_ERROR = ""

YDLIDAR_PYTHON = os.environ.get(
    "YDLIDAR_PYTHON",
    "/home/pi/RaspberryPi-CM5/robots/Mini3W_W/demos/YDLidar-SDK/build/python",
)
if YDLIDAR_PYTHON not in sys.path:
    sys.path.insert(0, YDLIDAR_PYTHON)

try:
    import ydlidar  # type: ignore
except Exception as exc:  # pragma: no cover - hardware dependent
    ydlidar = None
    YDLIDAR_IMPORT_ERROR = str(exc)
else:
    YDLIDAR_IMPORT_ERROR = ""

try:
    from picamera2 import Picamera2  # type: ignore
    from picamera2.encoders import MJPEGEncoder  # type: ignore
    from picamera2.outputs import FileOutput  # type: ignore
except Exception as exc:  # pragma: no cover - hardware dependent
    Picamera2 = None
    MJPEGEncoder = None
    FileOutput = None
    CAMERA_IMPORT_ERROR = str(exc)
else:
    CAMERA_IMPORT_ERROR = ""

try:
    import cv2 as cv  # type: ignore
    import numpy as line_np  # type: ignore
    from line_follow import (  # type: ignore
        DEFAULT_LINE_CONFIG,
        LineProcessor,
        load_line_config,
        make_mask_preview,
        save_line_config,
        validate_line_config,
    )
except Exception as exc:  # pragma: no cover - optional vision feature
    cv = None
    line_np = None
    DEFAULT_LINE_CONFIG = {}
    LineProcessor = None
    load_line_config = None
    make_mask_preview = None
    save_line_config = None
    validate_line_config = None
    LINE_IMPORT_ERROR = str(exc)
else:
    LINE_IMPORT_ERROR = ""

HOST = os.environ.get("LIDAR_CONSOLE_HOST", "0.0.0.0")
PORT = int(os.environ.get("LIDAR_CONSOLE_PORT", "8088"))
LIDAR_PORT = os.environ.get("LIDAR_PORT", "/dev/ttyUSB0")
MANUAL_URL = os.environ.get("OUMAX_MANUAL_URL", "http://127.0.0.1:8765/command")
MANUAL_UDP_HOST = os.environ.get("OUMAX_MANUAL_UDP_HOST", "127.0.0.1")
MANUAL_UDP_PORT = int(os.environ.get("OUMAX_MANUAL_UDP_PORT", "8766"))
STATIC_DIR = Path(__file__).with_name("static")
CONFIG_FILE = Path(__file__).with_name("config.json")
LINE_CONFIG_FILE = Path(__file__).with_name("line_follow_config.json")
CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "480"))
CAMERA_FPS = float(os.environ.get("CAMERA_FPS", "30"))
CAMERA_JPEG_QUALITY = int(os.environ.get("CAMERA_JPEG_QUALITY", "70"))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_angle(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def robust_near(values: list[float]) -> float | None:
    """Return a near distance that ignores one or two isolated speckles."""
    if not values:
        return None
    values = sorted(values)
    if len(values) >= 5:
        return values[2]
    if len(values) >= 3:
        return values[1]
    return values[0]


DEFAULT_CONFIG: dict[str, Any] = {
    "cruise_vx": 12.0,          # normal forward speed
    "avoid_vx": 6.0,            # forward speed while avoiding
    "speed_change_ratio": 0.30,  # smoothing ratio per control cycle (0..1)
    "max_vy": 9.0,              # XGO move_y unit ~= cm/s
    "caution_distance": 0.80,   # begin avoidance
    "danger_distance": 0.34,    # stop forward motion / escape laterally
    "emergency_distance": 0.22,
    "self_min_range": 0.12,
    "display_max_range": 5.0,
    "forward_offset_deg": 0.0,
    "front_half_deg": 20.0,
    "lateral_sign": 1.0,        # +1 assumes positive VY means robot-left; flip in UI if needed
    "mask_ranges": [],           # fully ignored angles, use sparingly
    "self_filter_zones": [[-115.0, -30.0, 0.34]],  # [angle_min, angle_max, max_self_range_m]
    "control_hz": 8.0,
    "lateral_action_hold_sec": 0.70,  # keep one strafe decision long enough to complete a gait cycle
    "lateral_action_min_vy": 4.5,     # minimum effective strafe command once a strafe is requested
    "wall_forward_vx": 5.0,
    "wall_target_left": 0.30,
    "wall_stop_front": 0.30,
    "wall_approach_front": 0.55,
    "wall_distance_tolerance": 0.03,
    "wall_heading_tolerance": 3.0,
    "wall_lateral_kp": 30.0,
    "wall_min_vy": 8.0,
    "wall_max_vy": 8.0,
    "wall_yaw_kp": 1.5,
    "wall_max_yaw_rate": 16.0,
    "wall_control_hz": 10.0,
    "wall_complete_frames": 3.0,
    "wall_second_forward_vx": 5.0,
    "wall_second_target_left": 0.30,
    "wall_second_stop_front": 0.30,
    "wall_second_approach_front": 0.55,
    "wall_second_distance_tolerance": 0.03,
    "wall_second_heading_tolerance": 3.0,
    "wall_second_lateral_kp": 30.0,
    "wall_second_min_vy": 8.0,
    "wall_second_max_vy": 8.0,
    "wall_second_yaw_kp": 1.5,
    "wall_second_max_yaw_rate": 16.0,
    "wall_second_control_hz": 10.0,
    "wall_second_complete_frames": 3.0,
    "wall_final_forward_vx": 5.0,
    "wall_final_target_left": 0.30,
    "wall_final_stop_front": 0.30,
    "wall_final_approach_front": 0.55,
    "wall_final_distance_tolerance": 0.03,
    "wall_final_heading_tolerance": 3.0,
    "wall_final_lateral_kp": 30.0,
    "wall_final_min_vy": 8.0,
    "wall_final_max_vy": 8.0,
    "wall_final_yaw_kp": 1.5,
    "wall_final_max_yaw_rate": 16.0,
    "wall_final_control_hz": 10.0,
    "wall_final_complete_frames": 3.0,
}


WALL_PARAMETER_NAMES = (
    "forward_vx", "target_left", "stop_front", "approach_front",
    "distance_tolerance", "heading_tolerance", "lateral_kp", "min_vy", "max_vy",
    "yaw_kp", "max_yaw_rate", "control_hz", "complete_frames",
)


def wall_segment_config(cfg: dict[str, Any], segment: int) -> dict[str, Any]:
    selected = dict(cfg)
    if segment in (2, 3):
        prefix = "wall_second_" if segment == 2 else "wall_final_"
        for name in WALL_PARAMETER_NAMES:
            selected["wall_" + name] = cfg[prefix + name]
    return selected


class SharedState:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.config = dict(DEFAULT_CONFIG)
        self.scan_points: list[list[float]] = []
        self.sectors: dict[str, float | None] = {
            "front": None,
            "front_left": None,
            "front_right": None,
            "left": None,
            "right": None,
        }
        self.lidar = {
            "connected": False,
            "model": "YDLIDAR Tmini Plus",
            "port": LIDAR_PORT,
            "hz": 0.0,
            "points": 0,
            "error": YDLIDAR_IMPORT_ERROR,
            "stamp": 0.0,
        }
        self.robot = {
            "manual_service": False,
            "error": "",
            "vx": 0.0,
            "vy": 0.0,
            "yaw_rate": 0.0,
            "mode": "idle",
            "auto_enabled": False,
            "imu_stable": False,
        }
        self.line = {
            "enabled": False,
            "found": False,
            "center_x": None,
            "center_y": None,
            "line_width": 0.0,
            "raw_line_width": 0.0,
            "area": 0.0,
            "pid": 0.0,
            "lost_frames": 0,
            "probe_active": False,
            "probe_step": 0,
            "action": "",
            "error": LINE_IMPORT_ERROR,
        }
        self.line_config = dict(DEFAULT_LINE_CONFIG)
        self.wall = {
            "enabled": False,
            "completed": False,
            "phase": "idle",
            "segment": 0,
            "total_segments": 3,
            "initial_yaw": None,
            "phase_started": 0.0,
            "confirmation_stamp": 0.0,
            "target_yaw": None,
            "yaw_error": 0.0,
            "left_distance": None,
            "front_distance": None,
            "distance_error": None,
            "complete_frames": 0,
            "action": "",
            "error": "",
        }
        self.imu = {
            "connected": False,
            "yaw": 0.0,
            "error": "",
            "stamp": 0.0,
        }
        self.localization = {
            "method": "lidar_icp_imu_yaw",
            "initialized": False,
            "accepted": False,
            "inliers": 0,
            "overlap": 0.0,
            "residual": None,
            "error": LOCALIZATION_IMPORT_ERROR,
            "stamp": 0.0,
        }
        self.camera = {
            "connected": False,
            "model": "OV5647",
            "width": CAMERA_WIDTH,
            "height": CAMERA_HEIGHT,
            "fps": 0.0,
            "error": CAMERA_IMPORT_ERROR,
            "stamp": 0.0,
        }
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.path: deque[list[float]] = deque([[0.0, 0.0]], maxlen=1200)
        self._last_integrate = time.monotonic()
        self._last_path_add = self._last_integrate
        self.localization_reset_id = 0
        self.running = True

    def set_command(self, vx: float, vy: float, yaw_rate: float = 0.0, mode: str | None = None) -> None:
        with self.lock:
            # Commanded velocity is displayed and used for motion control only.
            # Localization is exclusively LiDAR scan matching + IMU yaw.
            self.robot["vx"] = float(vx)
            self.robot["vy"] = float(vy)
            self.robot["yaw_rate"] = float(yaw_rate)
            if mode is not None:
                self.robot["mode"] = mode

    def _integrate_locked(self) -> None:
        now = time.monotonic()
        dt = clamp(now - self._last_integrate, 0.0, 0.5)
        self._last_integrate = now
        if dt <= 0:
            return
        vx = float(self.robot["vx"]) / 100.0
        vy = float(self.robot["vy"]) / 100.0
        yaw_rate = math.radians(float(self.robot["yaw_rate"]))
        yaw = float(self.pose["yaw"])
        self.pose["x"] += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        self.pose["y"] += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.pose["yaw"] = normalize_angle(math.degrees(yaw + yaw_rate * dt))
        moved = math.hypot(vx * dt, vy * dt)
        if moved >= 0.002 or now - self._last_path_add >= 0.5:
            last = self.path[-1]
            if math.hypot(self.pose["x"] - last[0], self.pose["y"] - last[1]) >= 0.005:
                self.path.append([round(self.pose["x"], 4), round(self.pose["y"], 4)])
            self._last_path_add = now

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "ts": time.time(),
                "lidar": dict(self.lidar),
                "imu": dict(self.imu),
                "localization": dict(self.localization),
                "robot": dict(self.robot),
                "line": dict(self.line),
                "line_config": dict(self.line_config),
                "wall": dict(self.wall),
                "camera": dict(self.camera),
                "sectors": dict(self.sectors),
                "pose": dict(self.pose),
                "path": list(self.path),
                "scan": list(self.scan_points),
                "config": dict(self.config),
                "motion_limits": dict(MOTION_LIMITS),
                "path_note": "2D LiDAR scan matching for translation + IMU yaw for heading; no ROS, no wheel odometry.",
            }

    def reset_path(self) -> None:
        with self.lock:
            self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            self.path.clear()
            self.path.append([0.0, 0.0])
            self.localization_reset_id += 1
            self.localization.update(
                initialized=False,
                accepted=False,
                inliers=0,
                overlap=0.0,
                residual=None,
                error="",
                stamp=time.time(),
            )
            self._last_integrate = time.monotonic()
            self._last_path_add = self._last_integrate


def validate_wall_parameters(candidate: dict[str, Any]) -> str | None:
    try:
        wall_forward = float(candidate["wall_forward_vx"])
        wall_target = float(candidate["wall_target_left"])
        wall_stop = float(candidate["wall_stop_front"])
        wall_approach = float(candidate["wall_approach_front"])
        wall_distance_tolerance = float(candidate["wall_distance_tolerance"])
        wall_heading_tolerance = float(candidate["wall_heading_tolerance"])
        wall_lateral_kp = float(candidate["wall_lateral_kp"])
        wall_min_vy = float(candidate["wall_min_vy"])
        wall_max_vy = float(candidate["wall_max_vy"])
        wall_yaw_kp = float(candidate["wall_yaw_kp"])
        wall_max_yaw = float(candidate["wall_max_yaw_rate"])
        wall_hz = float(candidate["wall_control_hz"])
        wall_complete_frames = float(candidate["wall_complete_frames"])
    except (KeyError, TypeError, ValueError):
        return "左墙参数必须是有效数字"
    if not MIN_VX <= wall_forward <= 12.0:
        return f"左墙前进速度必须在 {MIN_VX}~12 之间，不能低于实测起动下限"
    if not 0.15 <= wall_target <= 1.0:
        return "左墙目标距离必须在 0.15~1.0m 之间"
    if not 0.15 <= wall_stop <= 1.0 or not wall_stop + 0.05 <= wall_approach <= 2.0:
        return "左墙模式减速距离必须至少比停车距离大 0.05m"
    if not 0.01 <= wall_distance_tolerance <= 0.15:
        return "左墙距离容差必须在 0.01~0.15m 之间"
    if not 0.5 <= wall_heading_tolerance <= 15.0:
        return "左墙航向容差必须在 0.5~15° 之间"
    if not 1.0 <= wall_lateral_kp <= 100.0:
        return "左墙横移增益必须在 1~100 之间"
    if not MIN_VY_SETTING <= wall_min_vy <= wall_max_vy <= 15.0:
        return f"左墙横移速度必须满足 {MIN_VY_SETTING} ≤ 最小 VY ≤ 最大 VY ≤ 15"
    if not 0.1 <= wall_yaw_kp <= 10.0 or not MIN_YAW_SETTING <= wall_max_yaw <= 60.0:
        return "左墙航向控制参数超出范围"
    if not 4.0 <= wall_hz <= 20.0 or not 1.0 <= wall_complete_frames <= 20.0:
        return "左墙控制频率或完成确认帧数超出范围"
    return None


def load_saved_config() -> dict[str, Any]:
    cfg = dict(DEFAULT_CONFIG)
    raw: dict[str, Any] = {}
    try:
        loaded = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        if isinstance(loaded, dict):
            raw = loaded
            for key, value in raw.items():
                if key not in cfg:
                    continue
                if key in {"mask_ranges", "self_filter_zones"} and isinstance(value, list):
                    cfg[key] = value[:16]
                elif key not in {"mask_ranges", "self_filter_zones"}:
                    cfg[key] = float(value)
    except FileNotFoundError:
        pass
    except Exception as exc:
        print(f"Config load warning: {exc}", flush=True)

    # Migrate older saved configs safely.
    cfg["cruise_vx"] = clamp(float(cfg["cruise_vx"]), 0.5, 20.0)
    if "avoid_vx" not in raw:
        cfg["avoid_vx"] = round(cfg["cruise_vx"] * 0.5, 2)
    cfg["avoid_vx"] = clamp(float(cfg["avoid_vx"]), 0.0, cfg["cruise_vx"])
    cfg["speed_change_ratio"] = clamp(float(cfg["speed_change_ratio"]), 0.05, 1.0)
    cfg["max_vy"] = clamp(float(cfg["max_vy"]), 0.0, 15.0)
    cfg["wall_forward_vx"] = clamp(float(cfg["wall_forward_vx"]), 0.5, 12.0)
    cfg["wall_target_left"] = clamp(float(cfg["wall_target_left"]), 0.15, 1.0)
    cfg["wall_stop_front"] = clamp(float(cfg["wall_stop_front"]), 0.15, 1.0)
    cfg["wall_approach_front"] = clamp(
        float(cfg["wall_approach_front"]),
        cfg["wall_stop_front"] + 0.05,
        2.0,
    )
    cfg["wall_distance_tolerance"] = clamp(float(cfg["wall_distance_tolerance"]), 0.01, 0.15)
    cfg["wall_heading_tolerance"] = clamp(float(cfg["wall_heading_tolerance"]), 0.5, 15.0)
    cfg["wall_lateral_kp"] = clamp(float(cfg["wall_lateral_kp"]), 1.0, 100.0)
    cfg["wall_min_vy"] = clamp(float(cfg["wall_min_vy"]), 0.0, 15.0)
    cfg["wall_max_vy"] = clamp(float(cfg["wall_max_vy"]), cfg["wall_min_vy"], 15.0)
    cfg["wall_yaw_kp"] = clamp(float(cfg["wall_yaw_kp"]), 0.1, 10.0)
    cfg["wall_max_yaw_rate"] = clamp(float(cfg["wall_max_yaw_rate"]), 1.0, 60.0)
    cfg["wall_control_hz"] = clamp(float(cfg["wall_control_hz"]), 4.0, 20.0)
    cfg["wall_complete_frames"] = clamp(float(cfg["wall_complete_frames"]), 1.0, 20.0)
    # One-time schema migration preserves the operator's current wall tuning.
    for prefix in ("wall_second_", "wall_final_"):
        for name in WALL_PARAMETER_NAMES:
            key = prefix + name
            if key not in raw:
                cfg[key] = cfg["wall_" + name]
    for key, minimum in (("cruise_vx", MIN_VX), ("avoid_vx", MIN_VX), ("max_vy", MIN_VY_SETTING)):
        migrate_speed_floor(cfg, key, minimum)
    for prefix in ("wall_", "wall_second_", "wall_final_"):
        for name, minimum in (("forward_vx", MIN_VX), ("min_vy", MIN_VY_SETTING), ("max_vy", MIN_VY_SETTING), ("max_yaw_rate", MIN_YAW_SETTING)):
            migrate_speed_floor(cfg, prefix + name, minimum)
    second_error = validate_wall_parameters(wall_segment_config(cfg, 2))
    if second_error:
        raise ValueError("第二段配置无效：" + second_error)
    final_error = validate_wall_parameters(wall_segment_config(cfg, 3))
    if final_error:
        raise ValueError("第三段配置无效：" + final_error)
    emergency = float(cfg["emergency_distance"])
    danger = float(cfg["danger_distance"])
    caution = float(cfg["caution_distance"])
    if not (0.05 <= emergency < danger < caution <= 5.0):
        # Keep a valid danger/emergency pair when possible, but restore a safe
        # avoidance-entry distance if an old config inverted the ordering.
        emergency = clamp(emergency, 0.05, 4.8)
        danger = clamp(max(danger, emergency + 0.05), emergency + 0.05, 4.9)
        caution = clamp(max(DEFAULT_CONFIG["caution_distance"], danger + 0.10), danger + 0.05, 5.0)
        cfg["emergency_distance"] = round(emergency, 3)
        cfg["danger_distance"] = round(danger, 3)
        cfg["caution_distance"] = round(caution, 3)
    return cfg


STATE = SharedState()
STATE.config = load_saved_config()
if load_line_config is not None:
    STATE.line_config = load_line_config(LINE_CONFIG_FILE)

CAMERA_FRAME_LOCK = threading.Lock()
CAMERA_FRAME = None
CAMERA_FRAME_SEQ = 0
LINE_PREVIEW_LOCK = threading.Lock()
LINE_PREVIEW = None
LINE_MASK_PREVIEW = None


def is_masked(angle: float, masks: list[Any]) -> bool:
    for pair in masks:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            continue
        try:
            a, b = normalize_angle(float(pair[0])), normalize_angle(float(pair[1]))
        except Exception:
            continue
        if a <= b:
            if a <= angle <= b:
                return True
        else:
            if angle >= a or angle <= b:
                return True
    return False


def sector_distance(points: list[tuple[float, float]], amin: float, amax: float) -> float | None:
    vals = [r for a, r in points if amin <= a <= amax]
    return robust_near(vals)


def is_self_filtered(angle: float, distance: float, zones: list[Any]) -> bool:
    """Filter only near-field robot geometry while preserving farther obstacles at the same angle."""
    for zone in zones:
        if not isinstance(zone, (list, tuple)) or len(zone) != 3:
            continue
        try:
            a, b = normalize_angle(float(zone[0])), normalize_angle(float(zone[1]))
            max_range = max(0.0, float(zone[2]))
        except Exception:
            continue
        inside = (a <= angle <= b) if a <= b else (angle >= a or angle <= b)
        if inside and distance <= max_range:
            return True
    return False


class LidarImuOdometry:
    """Planar LiDAR odometry: ICP translation with IMU-constrained yaw."""

    def __init__(self) -> None:
        self.ref_points = None
        self.ref_imu_yaw: float | None = None
        self.ref_pose = (0.0, 0.0, 0.0)
        self.reset_id = -1

    def reset(self, reset_id: int) -> None:
        self.ref_points = None
        self.ref_imu_yaw = None
        self.ref_pose = (0.0, 0.0, 0.0)
        self.reset_id = reset_id

    @staticmethod
    def _cloud(points: list[tuple[float, float]]):
        if np is None:
            return None
        xy = [
            (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
            for a, r in points
            if 0.15 <= r <= 6.0
        ]
        if len(xy) < 30:
            return np.asarray(xy, dtype=np.float64)
        arr = np.asarray(xy, dtype=np.float64)
        # Small voxel filter keeps ICP light and suppresses dense duplicate hits.
        keys = np.floor(arr / 0.035).astype(np.int32)
        _, unique_idx = np.unique(keys, axis=0, return_index=True)
        arr = arr[np.sort(unique_idx)]
        if len(arr) > 320:
            step = max(1, len(arr) // 320)
            arr = arr[::step][:320]
        return arr

    def update(self, points: list[tuple[float, float]]) -> None:
        if np is None or cKDTree is None:
            with STATE.lock:
                STATE.localization.update(
                    initialized=False,
                    accepted=False,
                    error=LOCALIZATION_IMPORT_ERROR or "numpy/scipy unavailable",
                    stamp=time.time(),
                )
            return

        with STATE.lock:
            reset_id = int(STATE.localization_reset_id)
            imu = dict(STATE.imu)
        if reset_id != self.reset_id:
            self.reset(reset_id)

        if not imu.get("connected") or time.time() - float(imu.get("stamp", 0.0)) > 0.5:
            with STATE.lock:
                STATE.localization.update(
                    accepted=False,
                    error="IMU yaw unavailable",
                    stamp=time.time(),
                )
            return

        cloud = self._cloud(points)
        imu_yaw = float(imu.get("yaw", 0.0))
        if cloud is None or len(cloud) < 35:
            with STATE.lock:
                STATE.localization.update(
                    accepted=False,
                    inliers=0,
                    overlap=0.0,
                    residual=None,
                    error="too few LiDAR points for scan matching",
                    stamp=time.time(),
                )
            return

        if self.ref_points is None or self.ref_imu_yaw is None:
            self.ref_points = cloud
            self.ref_imu_yaw = imu_yaw
            with STATE.lock:
                self.ref_pose = (
                    float(STATE.pose["x"]),
                    float(STATE.pose["y"]),
                    float(STATE.pose["yaw"]),
                )
            with STATE.lock:
                STATE.localization.update(
                    initialized=True,
                    accepted=False,
                    inliers=len(cloud),
                    overlap=1.0,
                    residual=0.0,
                    error="",
                    stamp=time.time(),
                )
            return

        dyaw_deg = normalize_angle(imu_yaw - self.ref_imu_yaw)
        if abs(dyaw_deg) > 35.0:
            # At 10 Hz this is an implausible jump and usually means an IMU wrap/glitch.
            with STATE.lock:
                STATE.localization.update(
                    accepted=False,
                    error=f"IMU yaw jump rejected: {dyaw_deg:.1f} deg",
                    stamp=time.time(),
                )
            return

        theta = math.radians(dyaw_deg)
        c, s = math.cos(theta), math.sin(theta)
        rot = np.asarray([[c, -s], [s, c]], dtype=np.float64)
        source = cloud @ rot.T
        target = self.ref_points
        tree = cKDTree(target)
        translation = np.zeros(2, dtype=np.float64)
        k = min(7, len(target))
        _, neighbor_idx = tree.query(target, k=k)
        if k == 1:
            neighbor_idx = neighbor_idx[:, None]
        normals = np.zeros_like(target)
        for i, ids in enumerate(neighbor_idx):
            neighborhood = target[np.asarray(ids, dtype=np.int64)]
            centered = neighborhood - np.mean(neighborhood, axis=0)
            cov = centered.T @ centered
            try:
                _, eigenvectors = np.linalg.eigh(cov)
                normals[i] = eigenvectors[:, 0]
            except Exception:
                normals[i] = (1.0, 0.0)

        # Translation-only point-to-line ICP. Rotation comes from IMU yaw, so
        # wheel slip does not contaminate heading and ICP has only x/y to solve.
        geometry_rank = 0
        for _ in range(8):
            transformed = source + translation
            dist, idx = tree.query(transformed, k=1)
            valid = dist < 0.32
            if int(np.count_nonzero(valid)) < 25:
                break
            valid_dist = dist[valid]
            trim = min(0.32, float(np.quantile(valid_dist, 0.82)) + 0.01)
            keep = valid & (dist <= trim)
            if int(np.count_nonzero(keep)) < 25:
                break
            a = normals[idx[keep]]
            b = np.sum(a * (target[idx[keep]] - transformed[keep]), axis=1)
            try:
                delta, _, geometry_rank, _ = np.linalg.lstsq(a, b, rcond=None)
            except Exception:
                break
            if int(geometry_rank) < 2:
                break
            translation += delta
            if float(np.linalg.norm(delta)) < 0.0008:
                break

        transformed = source + translation
        dist, idx = tree.query(transformed, k=1)
        inlier_mask = dist < 0.18
        inliers = int(np.count_nonzero(inlier_mask))
        overlap = min(1.0, inliers / max(1, len(source)))
        if inliers:
            line_error = np.abs(
                np.sum(
                    normals[idx[inlier_mask]]
                    * (transformed[inlier_mask] - target[idx[inlier_mask]]),
                    axis=1,
                )
            )
            residual = float(np.median(line_error))
        else:
            residual = float("inf")
        travel = float(np.linalg.norm(translation))

        ref_x, ref_y, ref_yaw = self.ref_pose
        ref_yaw_rad = math.radians(ref_yaw)
        candidate_x = ref_x + math.cos(ref_yaw_rad) * float(translation[0]) - math.sin(ref_yaw_rad) * float(translation[1])
        candidate_y = ref_y + math.sin(ref_yaw_rad) * float(translation[0]) + math.cos(ref_yaw_rad) * float(translation[1])
        candidate_yaw = normalize_angle(ref_yaw + dyaw_deg)
        with STATE.lock:
            last_x = float(STATE.pose["x"])
            last_y = float(STATE.pose["y"])
        frame_jump = math.hypot(candidate_x - last_x, candidate_y - last_y)

        accepted = (
            inliers >= 28
            and overlap >= 0.28
            and int(geometry_rank) >= 2
            and residual <= 0.050
            and travel <= 0.25
            and frame_jump <= 0.08
        )

        if accepted:
            with STATE.lock:
                STATE.pose["x"] = candidate_x
                STATE.pose["y"] = candidate_y
                STATE.pose["yaw"] = candidate_yaw

                last = STATE.path[-1]
                if math.hypot(STATE.pose["x"] - last[0], STATE.pose["y"] - last[1]) >= 0.012:
                    STATE.path.append([round(STATE.pose["x"], 4), round(STATE.pose["y"], 4)])
                STATE.localization.update(
                    initialized=True,
                    accepted=True,
                    inliers=inliers,
                    overlap=round(overlap, 3),
                    residual=round(residual, 4),
                    error="",
                    stamp=time.time(),
                )

            # Keep matching against a stable keyframe so static scan noise does
            # not random-walk into the path. Advance the keyframe only after a
            # meaningful translation or heading change.
            if travel >= 0.08 or abs(dyaw_deg) >= 8.0:
                self.ref_points = cloud
                self.ref_imu_yaw = imu_yaw
                self.ref_pose = (candidate_x, candidate_y, candidate_yaw)
        else:
            with STATE.lock:
                STATE.localization.update(
                    initialized=True,
                    accepted=False,
                    inliers=inliers,
                    overlap=round(overlap, 3),
                    residual=None if not math.isfinite(residual) else round(residual, 4),
                    error=f"scan match rejected: overlap={overlap:.2f}, residual={residual:.3f}m, step={travel:.3f}m, jump={frame_jump:.3f}m",
                    stamp=time.time(),
                )


class LidarWorker(threading.Thread):
    daemon = True

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.odometry = LidarImuOdometry()

    def run(self) -> None:  # pragma: no cover - hardware loop
        if ydlidar is None:
            return
        try:
            ydlidar.os_init()
        except Exception:
            pass
        while STATE.running:
            laser = None
            try:
                laser = self._open_lidar()
                with STATE.lock:
                    STATE.lidar["connected"] = True
                    STATE.lidar["error"] = ""
                while STATE.running:
                    scan = ydlidar.LaserScan()
                    if not laser.doProcessSimple(scan):
                        raise RuntimeError(laser.DescribeError() or "failed to get lidar scan")
                    self._consume(scan)
                    # The SWIG lidar call can hold the GIL for most of a scan.
                    # Yield briefly so aiohttp can serve the dashboard smoothly.
                    time.sleep(0.01)
            except Exception as exc:
                with STATE.lock:
                    STATE.lidar["connected"] = False
                    STATE.lidar["error"] = str(exc)
                time.sleep(1.0)
            finally:
                if laser is not None:
                    try:
                        laser.turnOff()
                    except Exception:
                        pass
                    try:
                        laser.disconnecting()
                    except Exception:
                        pass

    def _open_lidar(self):
        laser = ydlidar.CYdLidar()
        options = [
            (ydlidar.LidarPropSerialPort, LIDAR_PORT),
            (ydlidar.LidarPropSerialBaudrate, 230400),
            (ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE),
            (ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL),
            (ydlidar.LidarPropSampleRate, 4),
            (ydlidar.LidarPropFixedResolution, True),
            (ydlidar.LidarPropReversion, False),
            (ydlidar.LidarPropInverted, False),
            (ydlidar.LidarPropAutoReconnect, True),
            (ydlidar.LidarPropSingleChannel, False),
            (ydlidar.LidarPropIntenstiy, True),
            (ydlidar.LidarPropSupportMotorDtrCtrl, False),
            (ydlidar.LidarPropSupportHeartBeat, False),
            (ydlidar.LidarPropMaxAngle, 180.0),
            (ydlidar.LidarPropMinAngle, -180.0),
            (ydlidar.LidarPropMaxRange, 64.0),
            (ydlidar.LidarPropMinRange, 0.05),
            (ydlidar.LidarPropScanFrequency, 10.0),
        ]
        for key, value in options:
            laser.setlidaropt(key, value)
        try:
            laser.setlidaropt(ydlidar.LidarPropIntenstiyBit, 8)
        except Exception:
            pass
        if not laser.initialize():
            raise RuntimeError("lidar initialize failed: " + str(laser.DescribeError()))
        if not laser.turnOn():
            raise RuntimeError("lidar turnOn failed: " + str(laser.DescribeError()))
        return laser

    def _consume(self, scan) -> None:
        with STATE.lock:
            cfg = dict(STATE.config)
        offset = float(cfg["forward_offset_deg"])
        self_min = float(cfg["self_min_range"])
        display_max = float(cfg["display_max_range"])
        masks = cfg.get("mask_ranges", [])
        self_zones = cfg.get("self_filter_zones", [])
        display: list[list[float]] = []
        control: list[tuple[float, float]] = []
        for p in scan.points:
            r = float(p.range)
            if r <= 0.05 or not math.isfinite(r):
                continue
            a = normalize_angle(-math.degrees(float(p.angle)) - offset)
            filtered = r < self_min or is_masked(a, masks) or is_self_filtered(a, r, self_zones)
            if r <= display_max:
                # Third field is 1 when the point is shown for diagnostics but excluded from avoidance.
                display.append([round(a, 2), round(r, 3), 1 if filtered else 0])
            if not filtered:
                control.append((a, r))

        front_half = float(cfg["front_half_deg"])
        sectors = {
            "front": sector_distance(control, -front_half, front_half),
            "front_left": sector_distance(control, 10.0, 65.0),
            "front_right": sector_distance(control, -65.0, -10.0),
            "left": sector_distance(control, 65.0, 115.0),
            "right": sector_distance(control, -115.0, -65.0),
        }
        hz = 0.0
        try:
            if scan.config.scan_time:
                hz = 1.0 / float(scan.config.scan_time)
        except Exception:
            pass
        with STATE.lock:
            STATE.scan_points = display
            STATE.sectors = sectors
            STATE.lidar.update(
                connected=True,
                hz=round(hz, 2),
                points=len(display),
                error="",
                stamp=time.time(),
            )

        # Localization uses the same self-filtered LiDAR points as avoidance,
        # but never uses commanded VX/VY as odometry.
        self.odometry.update(control)


class CameraWorker(threading.Thread):
    daemon = True

    def run(self) -> None:  # pragma: no cover - hardware loop
        global CAMERA_FRAME, CAMERA_FRAME_SEQ
        if Picamera2 is None or MJPEGEncoder is None or FileOutput is None:
            with STATE.lock:
                STATE.camera["connected"] = False
                STATE.camera["error"] = CAMERA_IMPORT_ERROR or "camera libraries unavailable"
            return

        class LatestJpegOutput(io.BufferedIOBase):
            def __init__(self) -> None:
                self.fps_t0 = time.monotonic()
                self.fps_count = 0

            def write(self, buf: bytes) -> int:
                global CAMERA_FRAME, CAMERA_FRAME_SEQ
                data = bytes(buf)
                with CAMERA_FRAME_LOCK:
                    CAMERA_FRAME = data
                    CAMERA_FRAME_SEQ += 1

                self.fps_count += 1
                now = time.monotonic()
                elapsed = now - self.fps_t0
                if elapsed >= 1.0:
                    with STATE.lock:
                        STATE.camera.update(
                            connected=True,
                            fps=round(self.fps_count / elapsed, 1),
                            error="",
                            stamp=time.time(),
                        )
                    self.fps_t0 = now
                    self.fps_count = 0
                return len(data)

        while STATE.running:
            camera = None
            recording = False
            try:
                camera = Picamera2()
                frame_us = int(round(1_000_000.0 / max(1.0, CAMERA_FPS)))
                config = camera.create_video_configuration(
                    main={"format": "YUV420", "size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
                    controls={"FrameDurationLimits": (frame_us, frame_us)},
                )
                camera.configure(config)
                output = LatestJpegOutput()
                encoder = MJPEGEncoder(framerate=max(1, int(round(CAMERA_FPS))))
                camera.start_recording(encoder, FileOutput(output))
                recording = True
                camera.set_controls({"FrameDurationLimits": (frame_us, frame_us)})
                with STATE.lock:
                    STATE.camera.update(
                        connected=True,
                        width=CAMERA_WIDTH,
                        height=CAMERA_HEIGHT,
                        error="",
                        stamp=time.time(),
                    )

                while STATE.running:
                    time.sleep(0.2)
            except Exception as exc:
                with STATE.lock:
                    STATE.camera["connected"] = False
                    STATE.camera["fps"] = 0.0
                    STATE.camera["error"] = str(exc)
                time.sleep(1.0)
            finally:
                if camera is not None:
                    try:
                        if recording:
                            camera.stop_recording()
                        else:
                            camera.stop()
                    except Exception:
                        pass
                    try:
                        camera.close()
                    except Exception:
                        pass


class RobotClient:
    def __init__(self) -> None:
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_lock = threading.Lock()

    def post(self, payload: dict[str, Any], timeout: float = 0.8) -> dict[str, Any]:
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(MANUAL_URL, data=data, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                result = json.loads(resp.read().decode("utf-8"))
            with STATE.lock:
                STATE.robot["manual_service"] = bool(result.get("ok"))
                STATE.robot["error"] = "" if result.get("ok") else str(result.get("message", "robot command failed"))
            return result
        except Exception as exc:
            with STATE.lock:
                STATE.robot["manual_service"] = False
                STATE.robot["error"] = str(exc)
            raise

    def _send_gamepad_udp(self, vx: float, vy: float, yaw_rate: float = 0.0) -> None:
        # manual_control_server's UDP path has a 0.35 s watchdog.  If this
        # process/browser dies, the robot therefore stops instead of keeping
        # the last persistent VX/VY command forever.
        with STATE.lock:
            y_sign = 1.0 if float(STATE.config.get("lateral_sign", 1.0)) >= 0 else -1.0
        payload = {
            "kind": "gamepad",
            "x": round(clamp(vx / 25.0, -1.0, 1.0), 4),
            "y": round(clamp(vy * y_sign / 18.0, -1.0, 1.0), 4),
            "yaw": round(clamp(yaw_rate / 80.0, -1.0, 1.0), 4),
            "drive_mode": "dog",
            "name": "lidar-console",
        }
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        with self._udp_lock:
            self._udp.sendto(data, (MANUAL_UDP_HOST, MANUAL_UDP_PORT))
        with STATE.lock:
            STATE.robot["manual_service"] = True
            STATE.robot["error"] = ""

    def axes(self, vx: float, vy: float, yaw_rate: float = 0.0, mode: str = "manual") -> None:
        with STATE.lock:
            vx, vy, yaw_rate = enforce_axes(vx, vy, yaw_rate, float(STATE.config["lateral_sign"]))
            self._send_gamepad_udp(vx, vy, yaw_rate)
            STATE.set_command(vx, vy, yaw_rate, mode=mode)

    def stop(self) -> None:
        # Zero UDP frame first (fast), then HTTP stop as a confirmed backup.
        try:
            self._send_gamepad_udp(0.0, 0.0, 0.0)
        finally:
            try:
                self.post({"kind": "motion", "axis": "stop", "value": 0, "drive_mode": "dog"}, timeout=1.0)
            finally:
                STATE.set_command(0.0, 0.0, 0.0, mode="idle")


ROBOT = RobotClient()


class ImuWorker(threading.Thread):
    daemon = True

    def run(self) -> None:  # pragma: no cover - hardware loop
        while STATE.running:
            try:
                result = ROBOT.post({"kind": "imu_yaw"}, timeout=0.45)
                yaw = float(result.get("yaw", 0.0))
                with STATE.lock:
                    STATE.imu.update(
                        connected=True,
                        yaw=yaw,
                        error="",
                        stamp=time.time(),
                    )
            except Exception as exc:
                with STATE.lock:
                    STATE.imu.update(
                        connected=False,
                        error=str(exc),
                        stamp=time.time(),
                    )
            time.sleep(0.08)


def distance_or_far(value: float | None, far: float = 99.0) -> float:
    return far if value is None else float(value)


def repulsion(distance: float, caution: float) -> float:
    if distance >= caution:
        return 0.0
    return clamp((caution - distance) / max(caution, 0.05), 0.0, 1.0)


def compute_auto_command(sectors: dict[str, float | None], cfg: dict[str, Any]) -> tuple[float, float]:
    cruise = float(cfg["cruise_vx"])
    max_vy = abs(float(cfg["max_vy"]))
    caution = float(cfg["caution_distance"])
    danger = float(cfg["danger_distance"])
    emergency = float(cfg["emergency_distance"])
    front = distance_or_far(sectors.get("front"))
    fl = distance_or_far(sectors.get("front_left"))
    fr = distance_or_far(sectors.get("front_right"))
    left = distance_or_far(sectors.get("left"))
    right = distance_or_far(sectors.get("right"))

    # Positive physical lateral command means robot-left. Left obstacles push right (negative), and vice versa.
    lateral_physical = (repulsion(fr, caution) - repulsion(fl, caution)) * max_vy

    # A centered frontal obstacle chooses the side with more free space.
    if front < caution:
        clear_left = min(fl, left)
        clear_right = min(fr, right)
        if abs(clear_left - clear_right) > 0.05:
            escape = 1.0 if clear_left > clear_right else -1.0
            lateral_physical += escape * max_vy * repulsion(front, caution)

    vy = clamp(lateral_physical, -max_vy, max_vy)

    if front <= emergency:
        vx = 0.0
        # Do not blindly drive laterally if both sides are also critically blocked.
        if min(left, right, fl, fr) <= emergency:
            vy = 0.0
    elif front <= danger:
        vx = 0.0
    elif front < caution:
        vx = clamp(float(cfg.get("avoid_vx", cruise)), 0.0, cruise)
    else:
        vx = cruise

    vx, vy, _ = enforce_axes(vx, vy, 0.0, float(cfg["lateral_sign"]))
    return round(vx, 2), round(vy, 2)


def compute_wall_command(
    front_distance: float,
    left_distance: float,
    current_yaw: float,
    target_yaw: float,
    cfg: dict[str, Any],
) -> tuple[float, float, float, float, float, bool]:
    """Compute physical VX/VY/yaw for left-wall distance and heading hold."""
    target_left = float(cfg["wall_target_left"])
    stop_front = float(cfg["wall_stop_front"])
    approach_front = float(cfg["wall_approach_front"])
    distance_tolerance = float(cfg["wall_distance_tolerance"])
    heading_tolerance = float(cfg["wall_heading_tolerance"])
    distance_error = float(left_distance) - target_left
    yaw_error = normalize_angle(float(target_yaw) - float(current_yaw))

    left_on_target = abs(distance_error) <= distance_tolerance
    heading_on_target = abs(yaw_error) <= heading_tolerance
    completed = float(front_distance) <= stop_front and left_on_target and heading_on_target
    if completed:
        return 0.0, 0.0, 0.0, distance_error, yaw_error, True

    if float(front_distance) <= stop_front:
        vx = 0.0
    elif float(front_distance) < approach_front:
        span = max(approach_front - stop_front, 0.01)
        ratio = clamp((float(front_distance) - stop_front) / span, 0.0, 1.0)
        vx = float(cfg["wall_forward_vx"]) * ratio
    else:
        vx = float(cfg["wall_forward_vx"])

    if left_on_target:
        vy = 0.0
    else:
        max_vy = float(cfg["wall_max_vy"])
        min_vy = min(float(cfg["wall_min_vy"]), max_vy)
        vy = math.copysign(
            clamp(abs(distance_error) * float(cfg["wall_lateral_kp"]), min_vy, max_vy),
            distance_error,
        )

    if heading_on_target:
        yaw_rate = 0.0
    else:
        yaw_rate = clamp(
            yaw_error * float(cfg["wall_yaw_kp"]),
            -float(cfg["wall_max_yaw_rate"]),
            float(cfg["wall_max_yaw_rate"]),
        )
    vx, vy, yaw_rate = enforce_axes(vx, vy, yaw_rate, float(cfg["lateral_sign"]))
    return (
        round(vx, 2),
        round(vy, 2),
        round(yaw_rate, 2),
        round(distance_error, 4),
        round(yaw_error, 3),
        False,
    )


class AutoController(threading.Thread):
    daemon = True

    def run(self) -> None:  # pragma: no cover - hardware loop
        last_sent = (999.0, 999.0)
        smoothed_vx: float | None = None
        last_tx = 0.0
        last_health = 0.0
        latched_vy = 0.0
        lateral_hold_until = 0.0
        while STATE.running:
            with STATE.lock:
                enabled = bool(STATE.robot["auto_enabled"])
                cfg = dict(STATE.config)
                sectors = dict(STATE.sectors)
                lidar_ok = bool(STATE.lidar["connected"])
            hz = clamp(float(cfg.get("control_hz", 8.0)), 2.0, 20.0)
            if not enabled:
                last_sent = (999.0, 999.0)
                smoothed_vx = None
                latched_vy = 0.0
                lateral_hold_until = 0.0
                now = time.monotonic()
                if now - last_health >= 2.0:
                    try:
                        ROBOT.post({"kind": "health"}, timeout=0.5)
                    except Exception:
                        pass
                    last_health = now
                time.sleep(0.1)
                continue
            if not lidar_ok or sectors.get("front") is None:
                latched_vy = 0.0
                lateral_hold_until = 0.0
                try:
                    ROBOT.stop()
                except Exception:
                    pass
                time.sleep(0.2)
                continue

            target_vx, target_vy = compute_auto_command(sectors, cfg)
            now = time.monotonic()

            # XGO lateral motion is a gait action, not an instantaneous velocity.
            # Once a strafe starts, keep the same direction long enough for one
            # gait cycle, and raise tiny VY requests to a physically effective
            # minimum. Emergency side blockage may still cancel it immediately.
            hold_sec = clamp(float(cfg.get("lateral_action_hold_sec", 0.70)), 0.25, 1.50)
            min_vy = clamp(float(cfg.get("lateral_action_min_vy", 4.5)), 0.0, abs(float(cfg.get("max_vy", 9.0))))
            front = distance_or_far(sectors.get("front"))
            fl = distance_or_far(sectors.get("front_left"))
            fr = distance_or_far(sectors.get("front_right"))
            left = distance_or_far(sectors.get("left"))
            right = distance_or_far(sectors.get("right"))
            emergency = float(cfg.get("emergency_distance", 0.22))

            blocked_latched_side = (
                (latched_vy > 0.0 and min(left, fl) <= emergency)
                or (latched_vy < 0.0 and min(right, fr) <= emergency)
                or (front <= emergency and min(left, right, fl, fr) <= emergency)
            )
            if blocked_latched_side:
                latched_vy = 0.0
                lateral_hold_until = 0.0

            if not blocked_latched_side and abs(target_vy) >= 0.05:
                requested_vy = math.copysign(max(abs(target_vy), min_vy), target_vy)
                if latched_vy == 0.0 or now >= lateral_hold_until:
                    latched_vy = round(requested_vy, 2)
                    lateral_hold_until = now + hold_sec
                elif math.copysign(1.0, target_vy) == math.copysign(1.0, latched_vy):
                    # Same direction: retain the gait cycle but allow stronger
                    # avoidance demand to increase the lateral amplitude.
                    latched_vy = math.copysign(max(abs(latched_vy), abs(requested_vy)), latched_vy)
            elif now >= lateral_hold_until:
                latched_vy = 0.0

            vy = 0.0 if blocked_latched_side else round(latched_vy, 2)
            blend = clamp(float(cfg.get("speed_change_ratio", 0.30)), 0.05, 1.0)
            if target_vx <= 0.01:
                vx = 0.0
                smoothed_vx = 0.0
            else:
                if smoothed_vx is None:
                    # On entering auto mode, use the current target immediately.
                    # speed_change_ratio is only for transitions between cruise
                    # and avoidance speeds, not for ramping from standstill.
                    smoothed_vx = target_vx
                else:
                    smoothed_vx += (target_vx - smoothed_vx) * blend
                if abs(target_vx - smoothed_vx) < 0.05:
                    smoothed_vx = target_vx
                vx = round(smoothed_vx, 2)
            changed = abs(vx - last_sent[0]) >= 0.15 or abs(vy - last_sent[1]) >= 0.5
            keepalive = now - last_tx >= 0.18
            if changed or keepalive:
                try:
                    ROBOT.axes(vx, vy, 0.0, mode="auto")
                    last_sent = (vx, vy)
                    last_tx = now
                except Exception:
                    # If robot control fails, zero the internal estimate and keep trying without pretending motion continued.
                    STATE.set_command(0.0, 0.0, 0.0, mode="auto-error")
            time.sleep(1.0 / hz)


def compute_wall_turn_command(
    current_yaw: float, target_yaw: float, cfg: dict[str, Any]
) -> tuple[float, float, bool]:
    """Close the heading loop for a right-angle turn, without translation."""
    error = normalize_angle(float(target_yaw) - float(current_yaw))
    if abs(error) <= float(cfg["wall_heading_tolerance"]):
        return 0.0, round(error, 3), True
    maximum = float(cfg["wall_max_yaw_rate"])
    # Keep the final approach above the chassis command deadzone.
    minimum = MIN_YAW_LEFT if error > 0 else MIN_YAW_RIGHT
    rate = math.copysign(clamp(abs(error) * float(cfg["wall_yaw_kp"]), minimum, maximum), error)
    return round(rate, 2), round(error, 3), False


class WallFollowController(threading.Thread):
    """Three left-wall straights, with IMU-controlled right turns between them."""

    daemon = True

    def _abort(self, message: str) -> None:
        stop_error = ""
        try:
            ROBOT.stop()
        except Exception as exc:
            stop_error = f"; 停车失败: {exc}"
        with STATE.lock:
            STATE.wall.update(
                enabled=False,
                completed=False,
                phase="aborted",
                action="aborted",
                error=message + stop_error,
                complete_frames=0,
            )
            STATE.robot["mode"] = "idle"

    def step(self, now: float) -> None:
        # Serialize the state check and output with mode changes / emergency stop.
        # A cancelled sequence must never send a command from an older snapshot.
        with STATE.lock:
            wall = STATE.wall
            if not wall["enabled"]:
                return
            cfg = wall_segment_config(STATE.config, int(wall["segment"]) if wall["phase"] == "straight" else 1)
            sectors, lidar, imu = STATE.sectors, STATE.lidar, STATE.imu
            target_yaw = wall["target_yaw"]
            if (
                target_yaw is None
                or not imu.get("connected")
                or now - float(imu.get("stamp", 0.0)) > 0.5
            ):
                self._abort("IMU 航向数据中断，已停车")
                return
            if not lidar.get("connected") or now - float(lidar.get("stamp", 0.0)) > 0.5:
                self._abort("雷达数据中断，已停车")
                return
            front = sectors.get("front")
            left = sectors.get("left")
            wall.update(
                front_distance=None if front is None else round(float(front), 3),
                left_distance=None if left is None else round(float(left), 3),
            )
            phase = wall["phase"]
            if phase == "turning":
                if now - float(wall["phase_started"]) > 30.0:
                    self._abort("右转 90° 超时，未达到目标航向，已停车")
                    return
                yaw_rate, yaw_error, reached = compute_wall_turn_command(
                    float(imu["yaw"]), float(target_yaw), cfg
                )
                vx, vy = 0.0, 0.0
                distance_error = None
                stamp = float(imu["stamp"])
                action = "turn_confirming" if reached else "turning_right"
            elif phase == "straight":
                # After rotation, do not finish the next leg using the old corner scan.
                if float(lidar["stamp"]) <= float(wall["phase_started"]):
                    wall["action"] = "waiting_scan"
                    try:
                        ROBOT.axes(0.0, 0.0, 0.0, mode="wall")
                    except Exception as exc:
                        self._abort(f"底盘控制失败: {exc}")
                    return
                if front is None or left is None:
                    self._abort("前方或左侧无有效雷达距离，已停车")
                    return
                vx, vy, yaw_rate, distance_error, yaw_error, reached = compute_wall_command(
                    float(front), float(left), float(imu["yaw"]), float(target_yaw), cfg
                )
                stamp = float(lidar["stamp"])
                action = "confirming" if reached else ("final_align" if vx == 0.0 else "tracking")
            else:
                self._abort(f"左墙流程阶段无效: {phase}")
                return

            frames = int(wall["complete_frames"])
            if not reached:
                frames = 0
            elif stamp > float(wall["confirmation_stamp"]):
                frames += 1
            wall.update(
                distance_error=distance_error,
                yaw_error=yaw_error,
                complete_frames=frames,
                confirmation_stamp=stamp,
                action=action,
                error="",
            )
            if frames >= int(round(float(cfg["wall_complete_frames"]))):
                try:
                    ROBOT.stop()
                except Exception as exc:
                    self._abort(f"阶段完成但停车失败: {exc}")
                    return
                wall.update(complete_frames=0, confirmation_stamp=0.0, phase_started=now)
                if phase == "turning":
                    wall.update(phase="straight", segment=int(wall["segment"]) + 1, action="waiting_scan")
                    STATE.robot["mode"] = "wall"
                elif int(wall["segment"]) < int(wall["total_segments"]):
                    wall.update(
                        phase="turning",
                        target_yaw=round(normalize_angle(float(wall["initial_yaw"]) - 90.0 * int(wall["segment"])), 3),
                        yaw_error=-90.0,
                        action="turning_right",
                    )
                    STATE.robot["mode"] = "wall"
                else:
                    wall.update(enabled=False, completed=True, phase="completed", action="completed")
                    STATE.robot["mode"] = "idle"
                return
            try:
                ROBOT.axes(vx, vy, yaw_rate, mode="wall")
            except Exception as exc:
                self._abort(f"底盘控制失败: {exc}")

    def run(self) -> None:  # pragma: no cover - hardware loop
        while STATE.running:
            self.step(time.time())
            with STATE.lock:
                hz = float(wall_segment_config(STATE.config, int(STATE.wall["segment"]) if STATE.wall["phase"] == "straight" else 1)["wall_control_hz"])
            time.sleep(1.0 / hz)


class LineFollowController(threading.Thread):
    """Run the legacy black-line logic on frames already owned by the web app."""

    daemon = True

    def __init__(self) -> None:
        super().__init__(name="line-follow-controller")
        self.processor = None
        self.last_camera_seq = -1
        self.last_result = None
        self.action = ""
        self.action_speed = 0.0
        self.action_deadline = 0.0
        self.lost_frames = 0
        self.probe_step = 0
        self.probe_cooldown_until = 0.0
        self.surge_frames = 0
        self.enabled_seen = False

    def _set_line(self, **values: Any) -> None:
        with STATE.lock:
            STATE.line.update(values)

    def _send_axes(self, vx: float, yaw_rate: float = 0.0) -> None:
        ROBOT.axes(float(vx), 0.0, float(yaw_rate), mode="line")

    def _reset_runtime(self) -> None:
        self.processor = None
        self.last_result = None
        self.action = ""
        self.action_speed = 0.0
        self.action_deadline = 0.0
        self.lost_frames = 0
        self.probe_step = 0
        self.probe_cooldown_until = 0.0
        self.surge_frames = 0
        self._set_line(
            found=False,
            center_x=None,
            center_y=None,
            line_width=0.0,
            raw_line_width=0.0,
            area=0.0,
            pid=0.0,
            lost_frames=0,
            probe_active=False,
            probe_step=0,
            action="",
            error="",
        )

    def _publish_frame(self, result: Any, config: dict[str, Any]) -> None:
        global LINE_PREVIEW, LINE_MASK_PREVIEW
        ok, preview = cv.imencode(
            ".jpg",
            result.frame,
            [int(cv.IMWRITE_JPEG_QUALITY), CAMERA_JPEG_QUALITY],
        )
        if not ok:
            raise RuntimeError("巡线预览 JPEG 编码失败")
        mask = make_mask_preview(result.binary, int(config["crop_top"]))
        ok_mask, mask_preview = cv.imencode(
            ".jpg",
            mask,
            [int(cv.IMWRITE_JPEG_QUALITY), CAMERA_JPEG_QUALITY],
        )
        if not ok_mask:
            raise RuntimeError("巡线掩码 JPEG 编码失败")
        with LINE_PREVIEW_LOCK:
            LINE_PREVIEW = preview.tobytes()
            LINE_MASK_PREVIEW = mask_preview.tobytes()
        self._set_line(
            found=bool(result.found),
            center_x=None if result.center_x is None else round(result.center_x, 1),
            center_y=None if result.center_y is None else round(result.center_y, 1),
            line_width=round(result.line_width, 1),
            raw_line_width=round(result.raw_line_width, 1),
            area=round(result.area, 1),
            pid=round(result.pid, 2),
            lost_frames=self.lost_frames,
            probe_active=self.action.startswith("probe"),
            probe_step=self.probe_step,
            action=self.action,
            error="",
        )

    def _start_probe(self, config: dict[str, Any], now: float) -> None:
        self.probe_step = 1
        self.action = "probe_turn"
        self.action_speed = float(config["probe_turn_speed"])
        self.action_deadline = now + float(config["probe_step_duration"])
        self._send_axes(0.0, self.action_speed)

    def _finish_probe(self, found: bool, config: dict[str, Any], now: float) -> None:
        self._send_axes(0.0, 0.0)
        self.action = ""
        self.action_speed = 0.0
        self.action_deadline = 0.0
        self.probe_cooldown_until = now + float(config["probe_cooldown"])
        self.lost_frames = 0
        self.probe_step = 0
        if self.processor is not None:
            self.processor.reset()
        self._set_line(probe_active=False, probe_step=0, action="")

    def _start_surge_turn(self, config: dict[str, Any], now: float) -> None:
        self.action = "surge_turn"
        self.action_speed = float(config["surge_turn_speed"])
        self.action_deadline = now + float(config["surge_turn_duration"])
        self._send_axes(0.0, self.action_speed)

    def _advance_action(self, result: Any, config: dict[str, Any], now: float) -> None:
        if self.action == "correction_turn":
            if now < self.action_deadline:
                self._send_axes(0.0, self.action_speed)
                return
            self._send_axes(0.0, 0.0)
            self._send_axes(float(config["turn_move_speed"]), 0.0)
            self.action = "correction_forward"
            self.action_deadline = now + 0.15
            return
        if self.action == "correction_forward":
            if now < self.action_deadline:
                self._send_axes(float(config["turn_move_speed"]), 0.0)
                return
            self._send_axes(0.0, 0.0)
            self.action = ""
            self.action_deadline = 0.0
            return
        if self.action == "probe_turn":
            if now < self.action_deadline:
                self._send_axes(0.0, self.action_speed)
                return
            self._send_axes(0.0, 0.0)
            self.action = "probe_pause"
            self.action_deadline = now + float(config["probe_pause"])
            return
        if self.action == "probe_pause":
            if now < self.action_deadline:
                self._send_axes(0.0, 0.0)
                return
            if result is not None and result.found:
                self._finish_probe(True, config, now)
                return
            if self.probe_step >= int(config["probe_steps"]):
                self._finish_probe(False, config, now)
                return
            self.probe_step += 1
            self.action = "probe_turn"
            self.action_speed = float(config["probe_turn_speed"])
            self.action_deadline = now + float(config["probe_step_duration"])
            self._send_axes(0.0, self.action_speed)
            return
        if self.action == "surge_turn":
            if now < self.action_deadline:
                self._send_axes(0.0, self.action_speed)
                return
            self._send_axes(0.0, 0.0)
            self.action = ""
            self.action_speed = 0.0
            self.action_deadline = 0.0
            self.surge_frames = 0
            if self.processor is not None:
                self.processor.reset()

    def _process_preview(self, frame: Any, config: dict[str, Any]) -> Any:
        if self.processor is None:
            if LineProcessor is None:
                raise RuntimeError(LINE_IMPORT_ERROR or "巡线视觉依赖不可用")
            self.processor = LineProcessor(config)
        elif self.processor.config != config:
            self.processor.update_config(config)

        result = self.processor.process(frame)
        self.last_result = result
        self._publish_frame(result, config)
        return result

    def _step(self, frame: Any, config: dict[str, Any], now: float) -> None:
        result = self._process_preview(frame, config)
        if self.action:
            self._advance_action(result, config, now)
            return

        if not result.found:
            self.lost_frames += 1
            self._send_axes(0.0, 0.0)
            if (
                self.lost_frames >= int(config["lost_confirm_frames"])
                and now >= self.probe_cooldown_until
            ):
                self._start_probe(config, now)
            return

        self.lost_frames = 0
        if result.raw_line_width > float(config["surge_line_width"]):
            self.surge_frames += 1
            if self.surge_frames >= int(config["surge_frames"]):
                self.surge_frames = 0
                self._start_surge_turn(config, now)
                return
        else:
            self.surge_frames = 0

        pid = float(result.pid)
        if abs(pid) < 8.0:
            self._send_axes(float(config["straight_speed"]), 0.0)
            return
        turn_speed = clamp(
            0.8 * abs(pid),
            float(config["turn_speed_min"]),
            float(config["turn_speed_max"]),
        )
        self.action = "correction_turn"
        self.action_speed = math.copysign(turn_speed, pid)
        self.action_deadline = now + min(0.5 + 0.005 * abs(int(pid)), 0.8)
        self._send_axes(0.0, self.action_speed)

    def run(self) -> None:  # pragma: no cover - hardware loop
        while STATE.running:
            with STATE.lock:
                enabled = bool(STATE.line["enabled"])
                config = dict(STATE.line_config)
            if not enabled:
                if self.enabled_seen:
                    self._reset_runtime()
                self.enabled_seen = False
            else:
                self.enabled_seen = True
            if cv is None or line_np is None or LineProcessor is None:
                self._set_line(error=LINE_IMPORT_ERROR or "巡线视觉依赖不可用")
                time.sleep(0.5)
                continue

            try:
                with CAMERA_FRAME_LOCK:
                    jpeg = CAMERA_FRAME
                    sequence = CAMERA_FRAME_SEQ
                now = time.monotonic()
                if jpeg is not None and sequence != self.last_camera_seq:
                    self.last_camera_seq = sequence
                    frame = cv.imdecode(line_np.frombuffer(jpeg, dtype=line_np.uint8), cv.IMREAD_COLOR)
                    if frame is None:
                        raise RuntimeError("巡线无法解码摄像头 JPEG 帧")
                    if enabled:
                        self._step(frame, config, now)
                    else:
                        self._process_preview(frame, config)
                elif enabled and self.action:
                    self._advance_action(self.last_result, config, now)
                elif enabled:
                    self._set_line(error="等待摄像头帧")
                    self._send_axes(0.0, 0.0)
                elif jpeg is None:
                    self._set_line(error="等待摄像头帧")
                time.sleep(1.0 / float(config["control_hz"]))
            except Exception as exc:
                self._set_line(error=str(exc), action="")
                self.action = ""
                self.action_deadline = 0.0
                if enabled:
                    try:
                        ROBOT.stop()
                    except Exception as stop_exc:
                        self._set_line(error=f"{exc}; 停车失败: {stop_exc}")
                time.sleep(1.0)


async def json_body(request: web.Request) -> dict[str, Any]:
    try:
        value = await request.json()
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


async def index(_: web.Request) -> web.Response:
    response = web.FileResponse(STATIC_DIR / "index.html")
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    return response


async def api_state(_: web.Request) -> web.Response:
    return web.json_response(STATE.snapshot())


async def api_line_config(request: web.Request) -> web.Response:
    if request.method == "GET":
        with STATE.lock:
            return web.json_response({"ok": True, "config": dict(STATE.line_config)})
    if save_line_config is None or validate_line_config is None:
        return web.json_response(
            {"ok": False, "message": LINE_IMPORT_ERROR or "巡线视觉依赖不可用"},
            status=503,
        )
    body = await json_body(request)
    try:
        config = save_line_config(LINE_CONFIG_FILE, body)
    except Exception as exc:
        return web.json_response({"ok": False, "message": str(exc)}, status=400)
    with STATE.lock:
        STATE.line_config = dict(config)
    return web.json_response({"ok": True, "config": config})


async def api_line(request: web.Request) -> web.Response:
    body = await json_body(request)
    enabled = bool(body.get("enabled", False))
    if enabled and LineProcessor is None:
        return web.json_response(
            {"ok": False, "message": LINE_IMPORT_ERROR or "巡线视觉依赖不可用"},
            status=503,
        )
    with STATE.lock:
        STATE.robot["auto_enabled"] = False
        STATE.line["enabled"] = False
        STATE.wall.update(enabled=False, completed=False, phase="idle", action="", error="", complete_frames=0)
    try:
        await asyncio.to_thread(ROBOT.stop)
    except Exception as exc:
        return web.json_response({"ok": False, "message": f"启动巡线前停车失败: {exc}"}, status=503)
    with STATE.lock:
        STATE.line["enabled"] = enabled
        STATE.robot["mode"] = "line" if enabled else "idle"
        STATE.line["error"] = ""
    return web.json_response({"ok": True, "enabled": enabled})


async def _mjpeg_stream(request: web.Request, kind: str) -> web.StreamResponse:
    response = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
        },
    )
    await response.prepare(request)
    last_seq = -1
    try:
        while STATE.running:
            if kind == "camera":
                with CAMERA_FRAME_LOCK:
                    seq = CAMERA_FRAME_SEQ
                    jpeg = CAMERA_FRAME
            else:
                with LINE_PREVIEW_LOCK:
                    jpeg = LINE_PREVIEW if kind == "line" else LINE_MASK_PREVIEW
                with CAMERA_FRAME_LOCK:
                    camera_seq = CAMERA_FRAME_SEQ
                seq = camera_seq if jpeg is not None else -1
                if jpeg is None:
                    with CAMERA_FRAME_LOCK:
                        jpeg = CAMERA_FRAME
                    seq = camera_seq
            if jpeg is not None and seq != last_seq:
                header = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + ("Content-Length: %d\r\n\r\n" % len(jpeg)).encode("ascii")
                )
                await response.write(header + jpeg + b"\r\n")
                last_seq = seq
            else:
                await asyncio.sleep(0.02)
    except (ConnectionResetError, asyncio.CancelledError, BrokenPipeError):
        pass
    return response


async def camera_stream(request: web.Request) -> web.StreamResponse:
    return await _mjpeg_stream(request, "camera")


async def line_stream(request: web.Request) -> web.StreamResponse:
    return await _mjpeg_stream(request, "line")


async def line_mask_stream(request: web.Request) -> web.StreamResponse:
    return await _mjpeg_stream(request, "mask")


async def api_auto(request: web.Request) -> web.Response:
    body = await json_body(request)
    enabled = bool(body.get("enabled", False))
    with STATE.lock:
        STATE.line["enabled"] = False
        STATE.robot["auto_enabled"] = False
        STATE.wall.update(enabled=False, completed=False, phase="idle", action="", error="", complete_frames=0)
        STATE.robot["mode"] = "idle"
    if enabled:
        try:
            await asyncio.to_thread(ROBOT.stop)
        except Exception as exc:
            return web.json_response({"ok": False, "message": f"启动自动避障前停车失败: {exc}"}, status=503)
    else:
        try:
            await asyncio.to_thread(ROBOT.stop)
        except Exception:
            pass
    with STATE.lock:
        STATE.robot["auto_enabled"] = enabled
        STATE.robot["mode"] = "auto" if enabled else "idle"
    return web.json_response({"ok": True, "auto_enabled": enabled})


async def api_wall(request: web.Request) -> web.Response:
    body = await json_body(request)
    enabled = bool(body.get("enabled", False))
    with STATE.lock:
        STATE.line["enabled"] = False
        STATE.robot["auto_enabled"] = False
        STATE.wall.update(enabled=False, completed=False, phase="idle", action="", error="", complete_frames=0)
        STATE.robot["mode"] = "idle"
    try:
        await asyncio.to_thread(ROBOT.stop)
    except Exception as exc:
        return web.json_response({"ok": False, "message": f"左墙定距模式启停前停车失败: {exc}"}, status=503)
    if not enabled:
        return web.json_response({"ok": True, "enabled": False})

    now = time.time()
    with STATE.lock:
        imu = dict(STATE.imu)
        lidar = dict(STATE.lidar)
        sectors = dict(STATE.sectors)
    if not imu.get("connected") or now - float(imu.get("stamp", 0.0)) > 0.5:
        return web.json_response({"ok": False, "message": "IMU 航向数据未就绪"}, status=503)
    if not lidar.get("connected") or now - float(lidar.get("stamp", 0.0)) > 0.5:
        return web.json_response({"ok": False, "message": "雷达数据未就绪"}, status=503)
    if sectors.get("front") is None or sectors.get("left") is None:
        return web.json_response({"ok": False, "message": "前方或左侧尚无有效雷达距离"}, status=503)

    target_yaw = float(imu["yaw"])
    with STATE.lock:
        STATE.wall.update(
            enabled=True,
            completed=False,
            phase="straight",
            segment=1,
            total_segments=3,
            initial_yaw=round(target_yaw, 3),
            phase_started=now,
            confirmation_stamp=0.0,
            target_yaw=round(target_yaw, 3),
            yaw_error=0.0,
            left_distance=round(float(sectors["left"]), 3),
            front_distance=round(float(sectors["front"]), 3),
            distance_error=round(float(sectors["left"]) - float(STATE.config["wall_target_left"]), 4),
            complete_frames=0,
            action="starting",
            error="",
        )
        STATE.robot["mode"] = "wall"
    return web.json_response({
        "ok": True, "enabled": True, "target_yaw": round(target_yaw, 3),
        "phase": "straight", "segment": 1, "total_segments": 3,
    })


async def api_imu(request: web.Request) -> web.Response:
    body = await json_body(request)
    enabled = bool(body.get("enabled", False))
    try:
        result = await asyncio.to_thread(ROBOT.post, {"kind": "imu_stable", "enabled": enabled}, 1.2)
        with STATE.lock:
            STATE.robot["imu_stable"] = enabled
        return web.json_response({"ok": True, "enabled": enabled, "result": result})
    except Exception as exc:
        return web.json_response({"ok": False, "message": str(exc)}, status=503)


async def api_manual(request: web.Request) -> web.Response:
    body = await json_body(request)
    try:
        requested = tuple(float(body.get(key, 0.0)) for key in ("vx", "vy", "yaw"))
    except (TypeError, ValueError):
        return web.json_response({"ok": False, "message": "运动指令必须是有效数字"}, status=400)
    if not all(math.isfinite(value) for value in requested):
        return web.json_response({"ok": False, "message": "运动指令必须是有限数字"}, status=400)
    vx = clamp(requested[0], -20.0, 20.0)
    vy = clamp(requested[1], -15.0, 15.0)
    yaw = clamp(requested[2], -60.0, 60.0)
    with STATE.lock:
        conditioned = enforce_axes(vx, vy, yaw, float(STATE.config["lateral_sign"]))
    if conditioned != (vx, vy, yaw):
        return web.json_response({"ok": False, "message": "非零手动速度不能低于实测起动下限", "motion_limits": MOTION_LIMITS}, status=400)
    with STATE.lock:
        STATE.line["enabled"] = False
        STATE.robot["auto_enabled"] = False
        STATE.wall.update(enabled=False, completed=False, phase="idle", action="", error="", complete_frames=0)
    try:
        await asyncio.to_thread(ROBOT.axes, vx, vy, yaw, "manual")
        return web.json_response({"ok": True, "vx": vx, "vy": vy, "yaw": yaw})
    except Exception as exc:
        return web.json_response({"ok": False, "message": str(exc)}, status=503)


async def api_stop(_: web.Request) -> web.Response:
    with STATE.lock:
        STATE.line["enabled"] = False
        STATE.robot["auto_enabled"] = False
        STATE.wall.update(enabled=False, completed=False, phase="idle", action="", error="", complete_frames=0)
    try:
        await asyncio.to_thread(ROBOT.stop)
        return web.json_response({"ok": True})
    except Exception as exc:
        return web.json_response({"ok": False, "message": str(exc)}, status=503)


async def api_arm_state(_: web.Request) -> web.Response:
    try:
        result = await asyncio.to_thread(ROBOT.post, {"kind": "arm_state"}, 1.5)
        arm = result.get("arm") or {}
        return web.json_response({
            "ok": True,
            "upper": arm.get("upper"),
            "lower": arm.get("lower"),
            "claw": arm.get("claw"),
            "rotate": arm.get("rotate"),
            "rotate_raw": arm.get("rotate_raw"),
        })
    except Exception as exc:
        return web.json_response({"ok": False, "message": str(exc)}, status=503)


async def api_arm(request: web.Request) -> web.Response:
    body = await json_body(request)
    command = str(body.get("command", "cartesian") or "cartesian")

    with STATE.lock:
        STATE.line["enabled"] = False
        STATE.robot["auto_enabled"] = False
        STATE.wall.update(enabled=False, completed=False, phase="idle", action="", error="", complete_frames=0)

    # Stop base motion immediately before arm adjustments. A zero-speed UDP
    # frame is serialized with the following arm command by the manual
    # service's dog_lock, so we do not block the UI on a second HTTP stop.
    try:
        await asyncio.to_thread(ROBOT._send_gamepad_udp, 0.0, 0.0, 0.0)
        STATE.set_command(0.0, 0.0, 0.0, mode="idle")
    except Exception as exc:
        return web.json_response({"ok": False, "message": f"底盘停止失败: {exc}"}, status=503)

    try:
        if command == "cartesian":
            x = float(body.get("x", 80.0))
            z = float(body.get("z", 60.0))
            if not (-80.0 <= x <= 155.0):
                return web.json_response({"ok": False, "message": "机械臂 X 必须在 -80~155 之间"}, status=400)
            if not (-95.0 <= z <= 155.0):
                return web.json_response({"ok": False, "message": "机械臂 Z 必须在 -95~155 之间"}, status=400)
            result = await asyncio.to_thread(
                ROBOT.post,
                {"kind": "arm", "command": "cartesian", "x": x, "z": z},
                1.2,
            )
            return web.json_response({"ok": True, "command": command, "x": x, "z": z, "result": result})

        if command == "polar":
            theta = float(body.get("theta", 90.0))
            radius = float(body.get("radius", 100.0))
            if not (70.0 <= theta <= 270.0):
                return web.json_response({"ok": False, "message": "机械臂 θ 必须在 70~270° 之间"}, status=400)
            if not (80.0 <= radius <= 140.0):
                return web.json_response({"ok": False, "message": "机械臂 R 必须在 80~140 之间"}, status=400)
            result = await asyncio.to_thread(
                ROBOT.post,
                {"kind": "arm", "command": "polar", "theta": theta, "radius": radius},
                1.2,
            )
            return web.json_response({"ok": True, "command": command, "theta": theta, "radius": radius, "result": result})

        if command == "claw":
            claw = int(float(body.get("claw", 128)))
            if not (0 <= claw <= 255):
                return web.json_response({"ok": False, "message": "夹爪参数必须在 0~255 之间"}, status=400)
            result = await asyncio.to_thread(
                ROBOT.post,
                {"kind": "arm", "command": "claw", "claw": claw},
                1.2,
            )
            return web.json_response({"ok": True, "command": command, "claw": claw, "result": result})

        if command == "mode":
            mode = int(float(body.get("mode", 0)))
            if mode not in (0, 1):
                return web.json_response({"ok": False, "message": "机械臂模式只能是 0 或 1"}, status=400)
            result = await asyncio.to_thread(
                ROBOT.post,
                {"kind": "arm", "command": "mode", "mode": mode},
                1.2,
            )
            return web.json_response({"ok": True, "command": command, "mode": mode, "result": result})

        if command == "speed":
            speed = int(float(body.get("speed", 128)))
            if not (1 <= speed <= 255):
                return web.json_response({"ok": False, "message": "机械臂速度必须在 1~255 之间"}, status=400)
            result = await asyncio.to_thread(
                ROBOT.post,
                {"kind": "arm", "command": "speed", "speed": speed},
                1.2,
            )
            return web.json_response({"ok": True, "command": command, "speed": speed, "result": result})

        if command == "rotate":
            angle = float(body.get("angle", 0.0))
            if not (-175.0 <= angle <= 175.0):
                return web.json_response({"ok": False, "message": "横向旋转角度必须在 -175~175° 之间"}, status=400)
            result = await asyncio.to_thread(
                ROBOT.post,
                {"kind": "arm", "command": "rotate", "angle": angle},
                1.2,
            )
            return web.json_response({"ok": True, "command": command, "angle": angle, "result": result})

        if command == "joint":
            joint = str(body.get("joint", "") or "")
            limits = {
                "upper": (53, -75.0, 90.0, "上臂"),
                "lower": (52, -85.0, 50.0, "小臂"),
                "claw_joint": (51, -65.0, 65.0, "夹爪"),
            }
            if joint not in limits:
                return web.json_response({"ok": False, "message": "未知机械臂关节"}, status=400)
            motor_id, lo, hi, label = limits[joint]
            angle = float(body.get("angle", 0.0))
            if not (lo <= angle <= hi):
                return web.json_response(
                    {"ok": False, "message": f"{label}角度必须在 {lo:g}~{hi:g}° 之间"},
                    status=400,
                )
            if joint == "claw_joint":
                claw_raw = int(round((angle + 65.0) / 130.0 * 255.0))
                claw_raw = max(0, min(255, claw_raw))
                result = await asyncio.to_thread(
                    ROBOT.post,
                    {"kind": "arm", "command": "claw", "claw": claw_raw},
                    1.2,
                )
            else:
                claw_raw = None
                result = await asyncio.to_thread(
                    ROBOT.post,
                    {"kind": "arm", "command": "joint", "motor_id": motor_id, "angle": angle},
                    1.2,
                )
            return web.json_response({
                "ok": True,
                "command": command,
                "joint": joint,
                "motor_id": motor_id,
                "angle": angle,
                "claw_raw": claw_raw,
                "result": result,
            })

        if command == "servo_speed":
            speed = int(float(body.get("speed", 128)))
            if not (1 <= speed <= 255):
                return web.json_response({"ok": False, "message": "舵机速度必须在 1~255 之间"}, status=400)
            result = await asyncio.to_thread(
                ROBOT.post,
                {"kind": "arm", "command": "servo_speed", "speed": speed},
                1.2,
            )
            return web.json_response({"ok": True, "command": command, "speed": speed, "result": result})

        return web.json_response({"ok": False, "message": "不支持的机械臂命令"}, status=400)
    except (TypeError, ValueError):
        return web.json_response({"ok": False, "message": "机械臂参数必须是有效数字"}, status=400)
    except Exception as exc:
        return web.json_response({"ok": False, "message": str(exc)}, status=503)


async def api_reset_path(_: web.Request) -> web.Response:
    STATE.reset_path()
    return web.json_response({"ok": True})


def validate_config(candidate: dict[str, Any]) -> str | None:
    try:
        cruise = float(candidate["cruise_vx"])
        avoid = float(candidate["avoid_vx"])
        blend = float(candidate["speed_change_ratio"])
        max_vy = float(candidate["max_vy"])
        caution = float(candidate["caution_distance"])
        danger = float(candidate["danger_distance"])
        emergency = float(candidate["emergency_distance"])
        self_min = float(candidate["self_min_range"])
        display = float(candidate["display_max_range"])
    except (KeyError, TypeError, ValueError):
        return "参数必须是有效数字"
    if not MIN_VX <= cruise <= 20.0:
        return f"正常前进速度必须在 {MIN_VX}~20 之间，不能低于实测起动下限"
    if not MIN_VX <= avoid <= cruise:
        return f"避障前进速度必须在 {MIN_VX}~正常前进速度之间"
    if not 0.05 <= blend <= 1.0:
        return "速度变化比例必须在 5%~100% 之间"
    if not MIN_VY_SETTING <= max_vy <= 15.0:
        return f"最大横移速度必须在 {MIN_VY_SETTING}~15 之间，不能低于实测起动下限"
    if not (0.05 <= emergency < danger < caution <= 5.0):
        return "距离必须满足：开始避障距离 > 前进停止距离 > 紧急距离，且范围为 0.05~5m"
    if not 0.03 <= self_min <= 0.50:
        return "自体最小距离过滤必须在 0.03~0.50m 之间"
    if not 0.5 <= display <= 20.0:
        return "雷达显示半径必须在 0.5~20m 之间"
    wall_error = validate_wall_parameters(candidate)
    if wall_error:
        return wall_error
    second_error = validate_wall_parameters(wall_segment_config(candidate, 2))
    if second_error:
        return "第二段：" + second_error
    final_error = validate_wall_parameters(wall_segment_config(candidate, 3))
    if final_error:
        return "第三段：" + final_error
    return None


async def api_config(request: web.Request) -> web.Response:
    if request.method == "GET":
        with STATE.lock:
            return web.json_response(dict(STATE.config))
    body = await json_body(request)
    allowed = set(DEFAULT_CONFIG)
    with STATE.lock:
        candidate = dict(STATE.config)
    for key, value in body.items():
        if key not in allowed:
            continue
        if key in {"mask_ranges", "self_filter_zones"}:
            if isinstance(value, list):
                candidate[key] = value[:16]
        else:
            try:
                candidate[key] = float(value)
            except (TypeError, ValueError):
                return web.json_response({"ok": False, "message": f"参数 {key} 不是有效数字"}, status=400)
    error = validate_config(candidate)
    if error:
        return web.json_response({"ok": False, "message": error}, status=400)
    with STATE.lock:
        STATE.config = candidate
        result = dict(STATE.config)
    try:
        CONFIG_FILE.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    except Exception as exc:
        return web.json_response({"ok": False, "message": f"config save failed: {exc}"}, status=500)
    return web.json_response({"ok": True, "config": result})


async def websocket(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse(heartbeat=15.0)
    await ws.prepare(request)
    try:
        while not ws.closed:
            await ws.send_json(STATE.snapshot())
            await asyncio.sleep(0.10)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return ws


async def on_shutdown(_: web.Application) -> None:
    STATE.running = False
    with STATE.lock:
        was_moving = (
            bool(STATE.line["enabled"])
            or bool(STATE.wall["enabled"])
            or abs(float(STATE.robot["vx"])) > 0.01
            or abs(float(STATE.robot["vy"])) > 0.01
            or abs(float(STATE.robot["yaw_rate"])) > 0.01
        )
    if was_moving:
        try:
            await asyncio.to_thread(ROBOT.stop)
        except Exception:
            pass


def make_app() -> web.Application:
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/api/state", api_state)
    app.router.add_get("/camera.mjpg", camera_stream)
    app.router.add_get("/line.mjpg", line_stream)
    app.router.add_get("/line-mask.mjpg", line_mask_stream)
    app.router.add_route("*", "/api/config", api_config)
    app.router.add_route("*", "/api/line/config", api_line_config)
    app.router.add_post("/api/line", api_line)
    app.router.add_post("/api/auto", api_auto)
    app.router.add_post("/api/wall", api_wall)
    app.router.add_post("/api/imu", api_imu)
    app.router.add_post("/api/manual", api_manual)
    app.router.add_post("/api/stop", api_stop)
    app.router.add_get("/api/arm/state", api_arm_state)
    app.router.add_post("/api/arm", api_arm)
    app.router.add_post("/api/path/reset", api_reset_path)
    app.router.add_get("/ws", websocket)
    app.router.add_static("/static/", STATIC_DIR)
    app.on_shutdown.append(on_shutdown)
    return app


if __name__ == "__main__":
    ImuWorker(name="imu-reader").start()
    LidarWorker(name="lidar-reader").start()
    CameraWorker(name="camera-reader").start()
    AutoController(name="auto-controller").start()
    WallFollowController(name="wall-follow-controller").start()
    LineFollowController().start()
    print(f"Lidar console: http://{HOST}:{PORT}", flush=True)
    print("Autonomous motion starts DISABLED. Enable it explicitly from the web console.", flush=True)
    web.run_app(make_app(), host=HOST, port=PORT, print=None)
