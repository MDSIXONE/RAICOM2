"""Immutable startup floors measured on this XGO on 2026-10-13."""
import math

MIN_VX = 3.0
MIN_VY_LEFT = 8.0
MIN_VY_RIGHT = 2.5
MIN_YAW_LEFT = 16.0
MIN_YAW_RIGHT = 14.0
MIN_VY_SETTING = max(MIN_VY_LEFT, MIN_VY_RIGHT)
MIN_YAW_SETTING = max(MIN_YAW_LEFT, MIN_YAW_RIGHT)

MOTION_LIMITS = {
    "calibrated_on": "2026-10-13",
    "vx": MIN_VX,
    "vy_left": MIN_VY_LEFT,
    "vy_right": MIN_VY_RIGHT,
    "vy_setting": MIN_VY_SETTING,
    "yaw_left": MIN_YAW_LEFT,
    "yaw_right": MIN_YAW_RIGHT,
    "yaw_setting": MIN_YAW_SETTING,
}


def motor_floor(value: float, positive: float, negative: float) -> float:
    if not math.isfinite(value):
        raise ValueError("运动指令必须是有限数字")
    if value == 0:
        return 0.0
    return math.copysign(max(abs(value), positive if value > 0 else negative), value)


def enforce_axes(vx: float, vy: float, yaw: float, lateral_sign: float = 1.0):
    sign = 1.0 if lateral_sign >= 0 else -1.0
    return (
        motor_floor(vx, MIN_VX, MIN_VX),
        motor_floor(vy * sign, MIN_VY_LEFT, MIN_VY_RIGHT) * sign,
        motor_floor(yaw, MIN_YAW_LEFT, MIN_YAW_RIGHT),
    )


def migrate_speed_floor(config: dict, key: str, minimum: float) -> None:
    """Apply the newly calibrated schema to an older persisted speed setting."""
    value = float(config[key])
    if value < minimum:
        print(f"校准升级 {key}: {value} -> {minimum}", flush=True)
        config[key] = minimum
