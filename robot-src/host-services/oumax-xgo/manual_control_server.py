from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


HOST = "0.0.0.0"
PORT = int(os.environ.get("OUMAX_MANUAL_PORT", "8765"))
UDP_PORT = int(os.environ.get("OUMAX_GAMEPAD_UDP_PORT", str(PORT + 1)))
UDP_WATCHDOG_SEC = float(os.environ.get("OUMAX_GAMEPAD_WATCHDOG_SEC", "0.35"))
SERIAL_PORT = os.environ.get("OUMAX_SERIAL_PORT", "/dev/ttyAMA0")
DOG_VERSION = os.environ.get("OUMAX_DOG_VERSION", "auto")

dog = None
dog_lock = threading.RLock()
udp_last_seen = 0.0
udp_last_active = False
udp_latest_payload = None
udp_payload_lock = threading.Lock()
udp_payload_event = threading.Event()


def get_dog():
    global dog
    if dog is None:
        from xgolib import XGO

        dog = XGO(port=SERIAL_PORT, version=xgo_constructor_version(DOG_VERSION))
    return dog


def xgo_constructor_version(version: str) -> str:
    normalized = str(version or "auto").lower()
    if normalized in {"xgomini2sw", "mini2sw", "xgomini3w", "mini3w"}:
        return "xgomini"
    return version


def handle_motion(payload: dict[str, Any]) -> str:
    bot = get_dog()
    axis = payload.get("axis", "stop")
    value = float(payload.get("value", 0))
    runtime = float(payload.get("runtime", 0) or 0)
    mode = payload.get("drive_mode", "auto")

    if axis == "stop":
        stop_bot(bot, mode)
    elif mode == "wheel4" and is_wheel_dog(bot):
        handle_wheel({"enabled": True, "speeds": wheel_speeds(axis, value)})
    elif axis == "x":
        bot.move_x(value, runtime)
    elif axis == "y":
        bot.move_y(value, runtime)
    elif axis == "yaw":
        bot.turn(value, runtime)
    else:
        raise ValueError(f"unknown motion axis: {axis}")
    return f"motion {axis}={value} mode={mode}"


def wheel_speeds(axis: str, value: float) -> list[float]:
    speed = max(-1.2, min(1.2, value / 30.0))
    if axis == "x":
        return [speed, speed, speed, speed]
    if axis == "y":
        return [-speed, speed, speed, -speed]
    if axis == "yaw":
        return [-speed, speed, -speed, speed]
    raise ValueError(f"unknown wheel motion axis: {axis}")


def handle_arm(payload: dict[str, Any]) -> str:
    bot = get_dog()
    command = payload.get("command", "cartesian")

    # The arm is servo group 5. Teaching/demo programs can unload this group,
    # in which case commands are accepted but the joints barely move. Ensure
    # torque is enabled before any manual arm adjustment.
    if command in {"cartesian", "polar", "claw", "joint", "rotate", "speed", "servo_speed", "mode"} and hasattr(bot, "load_motor"):
        bot.load_motor(5)

    if command == "cartesian":
        bot.arm(float(payload.get("x", 80)), float(payload.get("z", 60)))
    elif command == "polar":
        bot.arm_polar(float(payload.get("theta", 90)), float(payload.get("radius", 80)))
    elif command == "claw":
        bot.claw(int(payload.get("claw", 128)))
    elif command == "mode" and hasattr(bot, "arm_mode"):
        bot.arm_mode(int(payload.get("mode", 0)))
    elif command == "speed" and hasattr(bot, "arm_speed"):
        bot.arm_speed(int(payload.get("speed", 128)))
    elif command == "joint":
        motor_id = int(payload.get("motor_id", 53))
        angle = float(payload.get("angle", 0))
        if motor_id not in (51, 52, 53):
            raise ValueError("unsupported arm motor id")
        bot.motor(motor_id, angle)
    elif command == "rotate":
        angle = float(payload.get("angle", 0))
        if angle < -175 or angle > 175:
            raise ValueError("arm rotate angle must be between -175 and 175")
        set_arm_rotate_raw(arm_rotate_angle_to_raw(angle))
    elif command == "servo_speed" and hasattr(bot, "motor_speed"):
        bot.motor_speed(int(payload.get("speed", 128)))
    elif command == "motor" and hasattr(bot, "arm_motor"):
        bot.arm_motor(payload.get("angles", [0, 0, 0]))
    elif command == "mid" and hasattr(bot, "moveToMid"):
        bot.moveToMid()
    else:
        raise ValueError(f"unknown arm command: {command}")
    return f"arm {command}"


def read_arm_feedback() -> dict[str, float]:
    """Read firmware-reported arm joint angles, working around xgolib 1.4.2.

    xgolib's read_motor() requests 15 values but iterates rx_COUNT + 1,
    which can attempt to decode a nonexistent 16th motor and raise IndexError.
    By that point the valid 15-byte response is already present in rx_data.
    """
    bot = get_dog()
    try:
        angles = bot.read_motor()
    except IndexError:
        from xgolib import xgolib_dog as xd

        raw = list(getattr(bot, "rx_data", []))[:15]
        if len(raw) < 15:
            raise RuntimeError(f"incomplete motor feedback: {len(raw)} values")
        angles = []
        for i, value in enumerate(raw):
            limit = xd.XGOparam["MOTOR_LIMIT"][i % 3] if i < 12 else xd.XGOparam["MOTOR_LIMIT"][i - 9]
            angles.append(round(xd.conver2float(value, limit), 2))

    if not isinstance(angles, list) or len(angles) < 15:
        raise RuntimeError(f"invalid motor feedback: {angles!r}")
    return {
        "claw": float(angles[12]),
        "lower": float(angles[13]),
        "upper": float(angles[14]),
    }


def read_arm_rotate_raw() -> int:
    """Read the XGO arm turntable register (ARM_ROTATE, address 0x70)."""
    bot = get_dog()
    bot._XGO_DOG__read(0x70, 1)
    if not bot._XGO_DOG__unpack():
        raise RuntimeError("arm rotate feedback timeout")
    return int(bot.rx_data[0])


def arm_rotate_raw_to_angle(raw: int) -> float:
    raw = max(1, min(255, int(raw)))
    return round(-175.0 + (raw - 1) / 254.0 * 350.0, 2)


def arm_rotate_angle_to_raw(angle: float) -> int:
    angle = max(-175.0, min(175.0, float(angle)))
    return max(1, min(255, int(round(1.0 + (angle + 175.0) / 350.0 * 254.0))))


def set_arm_rotate_raw(value: int) -> None:
    """Write ARM_ROTATE (0x70) used by newer XGO Mini firmware/app."""
    value = max(0, min(255, int(value)))
    bot = get_dog()
    addr = 0x70
    length = 0x09
    mode = 0x00
    checksum = 255 - ((length + mode + addr + value) % 256)
    bot.ser.write(bytes([0x55, 0x00, length, mode, addr, value, checksum, 0x00, 0xAA]))


def arm_angle_to_u8(angle: float, lo: float, hi: float) -> int:
    angle = max(lo, min(hi, float(angle)))
    return max(1, min(255, int(round(1.0 + (angle - lo) / (hi - lo) * 254.0))))


def set_servo_ctrl_u8(servo_reg: int, value: int) -> None:
    """Use the MW-7.1.x SERVO_CTRL protocol (0xC0, subregister, value)."""
    bot = get_dog()
    addr = 0xC0
    data = [int(servo_reg) & 0xFF, max(1, min(255, int(value)))]
    length = 0x08 + len(data)
    mode = 0x00
    checksum = 255 - ((length + mode + addr + sum(data)) % 256)
    bot.ser.write(bytes([0x55, 0x00, length, mode, addr, *data, checksum, 0x00, 0xAA]))


def handle_wheel(payload: dict[str, Any]) -> str:
    bot = get_dog()
    enabled = payload.get("enabled")
    speeds = payload.get("speeds") or [0, 0, 0, 0]

    if not is_wheel_dog(bot):
        raise ValueError("wheel mode is unavailable on this foot-type dog")
    if enabled is not None and hasattr(bot, "enable_wheel_control"):
        bot.enable_wheel_control(1 if enabled else 0)
    send_wheel_control(bot, speeds)
    return f"wheel speeds={speeds}"


def handle_gamepad(payload: dict[str, Any]) -> str:
    bot = get_dog()
    x = clamp(float(payload.get("x", 0)), -1, 1)
    y = clamp(float(payload.get("y", 0)), -1, 1)
    yaw = clamp(float(payload.get("yaw", 0)), -1, 1)
    requested_mode = str(payload.get("drive_mode", "dog") or "dog")
    mode = "wheel4" if requested_mode == "wheel4" and is_wheel_dog(bot) else "dog"

    # Keep the normal 5% gamepad deadzone, but allow the lidar console to
    # command intentionally slow autonomous motion without being mistaken
    # for joystick drift.
    source = str(payload.get("name", "") or "")
    deadzone = 0.005 if source == "lidar-console" else 0.05
    if abs(x) < deadzone and abs(y) < deadzone and abs(yaw) < deadzone:
        stop_bot(bot, requested_mode)
        return f"gamepad stop mode={mode}"

    if mode == "wheel4":
        if hasattr(bot, "enable_wheel_control"):
            bot.enable_wheel_control(1)
        send_wheel_control(bot, gamepad_wheel_speeds(x, y, yaw))
    else:
        send_foot_control(bot, x, y, yaw)
    return f"gamepad x={x:.2f} y={y:.2f} yaw={yaw:.2f} mode={mode}"


def send_foot_control(bot: Any, x: float, y: float, yaw: float) -> None:
    bot.move_x(round(x * 25, 2))
    bot.move_y(round(y * 18, 2))
    bot.turn(round(yaw * 80, 2))


def stop_bot(bot: Any, requested_mode: str = "") -> None:
    bot.stop()
    if requested_mode == "wheel4" and is_wheel_dog(bot):
        send_wheel_control(bot, [0, 0, 0, 0])


def is_wheel_dog(bot: Any) -> bool:
    version = str(getattr(bot, "version", "") or "").lower()
    configured = str(DOG_VERSION or "").lower()
    return version.startswith("w") or "mini3w" in configured or "mini2sw" in configured


def gamepad_wheel_speeds(x: float, y: float, yaw: float) -> list[float]:
    # Mecanum-style mix for MINI3W / mini2SW: front/back, strafe, rotate.
    lf = x - y - yaw
    rf = x + y + yaw
    lr = x + y - yaw
    rr = x - y + yaw
    values = [lf, rf, lr, rr]
    peak = max(1.0, max(abs(v) for v in values))
    return [round(v / peak * 1.15, 3) for v in values]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def send_wheel_control(bot: Any, speeds: list[float]) -> None:
    values = [float(v) for v in list(speeds)[:4]]
    values += [0.0] * (4 - len(values))
    if hasattr(bot, "wheel_control"):
        bot.wheel_control([wheel_byte(v) for v in values])
        return
    if hasattr(bot, "wheel_speed"):
        try:
            bot.wheel_speed(values)
        except TypeError:
            for index, value in enumerate(values, start=1):
                bot.wheel_speed(index, value)
        return
    raise ValueError("wheel control is not available on this dog version")


def wheel_byte(speed: float) -> int:
    value = max(-1.5, min(1.5, speed))
    return max(0, min(255, int(round(128 + value / 1.5 * 127))))


def handle_speak(payload: dict[str, Any]) -> str:
    text = str(payload.get("text") or "").strip()
    if not text:
        raise ValueError("speak text is empty")

    commands = [
        ("espeak-ng", ["espeak-ng", "-v", "zh", "-s", "150", text]),
        ("espeak", ["espeak", "-v", "zh", "-s", "150", text]),
        ("spd-say", ["spd-say", text]),
        ("flite", ["flite", "-t", text]),
    ]
    for binary, command in commands:
        if shutil.which(binary):
            subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return f"speak on dog: {text}"
    raise RuntimeError("no TTS command found on CM5; install espeak-ng or another TTS tool")


def dispatch(payload: dict[str, Any]) -> dict[str, Any]:
    with dog_lock:
        kind = payload.get("kind")
        if kind == "motion":
            message = handle_motion(payload)
        elif kind == "arm":
            message = handle_arm(payload)
        elif kind == "wheel":
            message = handle_wheel(payload)
        elif kind == "gamepad":
            message = handle_gamepad(payload)
        elif kind == "speak":
            message = handle_speak(payload)
        elif kind == "imu_stable":
            enabled = bool(payload.get("enabled", False))
            bot = get_dog()
            bot.imu(1 if enabled else 0)
            return {
                "ok": True,
                "mode": "manual-runtime",
                "message": "imu stable on" if enabled else "imu stable off",
                "enabled": enabled,
            }
        elif kind == "imu_yaw":
            bot = get_dog()
            return {
                "ok": True,
                "mode": "manual-runtime",
                "yaw": float(bot.read_yaw()),
            }
        elif kind == "arm_state":
            rotate_raw = read_arm_rotate_raw()
            return {
                "ok": True,
                "mode": "manual-runtime",
                "arm": {
                    **read_arm_feedback(),
                    "rotate_raw": rotate_raw,
                    "rotate": arm_rotate_raw_to_angle(rotate_raw),
                },
            }
        elif kind == "arm_rotate_raw":
            value = int(payload.get("value", 128))
            set_arm_rotate_raw(value)
            return {
                "ok": True,
                "mode": "manual-runtime",
                "message": "arm rotate raw",
                "value": max(0, min(255, value)),
            }
        elif kind in {"ping", "health"}:
            message = "pong"
        else:
            raise ValueError(f"unknown command kind: {kind}")
    return {"ok": True, "mode": "manual-runtime", "message": message}


def is_active_gamepad_payload(payload: dict[str, Any]) -> bool:
    if payload.get("kind") == "motion":
        return payload.get("axis") != "stop" and abs(float(payload.get("value", 0))) >= 0.01
    if payload.get("kind") != "gamepad":
        return False
    x = abs(float(payload.get("x", 0)))
    y = abs(float(payload.get("y", 0)))
    yaw = abs(float(payload.get("yaw", 0)))
    source = str(payload.get("name", "") or "")
    deadzone = 0.005 if source == "lidar-console" else 0.05
    return x >= deadzone or y >= deadzone or yaw >= deadzone


def mark_udp_seen(payload: dict[str, Any]) -> None:
    global udp_last_seen, udp_last_active
    if payload.get("kind") not in {"gamepad", "motion"}:
        return
    udp_last_seen = time.monotonic()
    udp_last_active = is_active_gamepad_payload(payload)


def stop_from_watchdog() -> None:
    global udp_last_active
    try:
        dispatch({"kind": "gamepad", "x": 0, "y": 0, "yaw": 0, "drive_mode": "dog", "name": "udp-watchdog"})
        print("udp watchdog stop", flush=True)
    except Exception as exc:
        print(f"udp watchdog stop failed: {exc}", flush=True)
    finally:
        udp_last_active = False


def udp_watchdog_loop() -> None:
    while True:
        time.sleep(0.05)
        if udp_last_active and time.monotonic() - udp_last_seen > UDP_WATCHDOG_SEC:
            stop_from_watchdog()


def udp_server_loop() -> None:
    global udp_latest_payload
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, UDP_PORT))
    print(f"gamepad udp server listening on {HOST}:{UDP_PORT}", flush=True)
    while True:
        data, address = sock.recvfrom(4096)
        try:
            payload = json.loads(data.decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("udp payload must be a json object")
            mark_udp_seen(payload)
            with udp_payload_lock:
                udp_latest_payload = payload
                udp_payload_event.set()
            result = {"ok": True, "mode": "manual-runtime", "message": "queued latest gamepad frame"}
        except Exception as exc:
            result = {"ok": False, "mode": "manual-runtime", "message": str(exc)}
        try:
            sock.sendto(json.dumps(result, ensure_ascii=False).encode("utf-8"), address)
        except OSError as exc:
            print(f"udp response failed: {exc}", flush=True)


def udp_dispatch_loop() -> None:
    global udp_latest_payload
    while True:
        udp_payload_event.wait()
        while True:
            with udp_payload_lock:
                payload = udp_latest_payload
                udp_latest_payload = None
                if payload is None:
                    udp_payload_event.clear()
                    break
            try:
                dispatch(payload)
            except Exception as exc:
                print(f"udp dispatch failed: {exc}", flush=True)


class Handler(BaseHTTPRequestHandler):
    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.end_headers()

    def do_GET(self) -> None:
        if self.path != "/health":
            self.send_error(404)
            return
        self.respond(
            {
                "ok": True,
                "dog_version": DOG_VERSION,
                "serial_port": SERIAL_PORT,
                "manual_port": PORT,
                "gamepad_udp_port": UDP_PORT,
            }
        )

    def do_POST(self) -> None:
        if self.path != "/command":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            self.respond(dispatch(payload))
        except Exception as exc:
            self.respond({"ok": False, "message": str(exc)}, status=500)

    def log_message(self, fmt: str, *args: object) -> None:
        print(fmt % args, flush=True)

    def respond(self, data: dict[str, Any], status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


if __name__ == "__main__":
    threading.Thread(target=udp_server_loop, name="gamepad-udp", daemon=True).start()
    threading.Thread(target=udp_dispatch_loop, name="gamepad-dispatch", daemon=True).start()
    threading.Thread(target=udp_watchdog_loop, name="gamepad-watchdog", daemon=True).start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"manual control server listening on {HOST}:{PORT}", flush=True)
    server.serve_forever()
