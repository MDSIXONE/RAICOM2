#!/usr/bin/env bash
set -euo pipefail
BASE="$(cd "$(dirname "$0")" && pwd)"
XGO_PY="${XGO_PY:-/home/pi/RaspberryPi-CM5/xgovenv/bin/python}"
YDLIDAR_PYTHON="${YDLIDAR_PYTHON:-/home/pi/RaspberryPi-CM5/robots/Mini3W_W/demos/YDLidar-SDK/build/python}"
MANUAL_DIR="${MANUAL_DIR:-/home/pi/oumax-xgo}"

health_manual() {
  curl -fsS --max-time 1 -H 'Content-Type: application/json' -d '{"kind":"health"}' http://127.0.0.1:8765/command >/dev/null 2>&1
}

if ! health_manual; then
  if [[ -f "$MANUAL_DIR/manual_control.pid" ]]; then
    oldpid="$(cat "$MANUAL_DIR/manual_control.pid" 2>/dev/null || true)"
    if [[ -n "$oldpid" ]] && kill -0 "$oldpid" 2>/dev/null; then kill "$oldpid" || true; fi
  fi
  nohup "$XGO_PY" "$MANUAL_DIR/manual_control_server.py" >> "$MANUAL_DIR/manual_control.log" 2>&1 &
  echo $! > "$MANUAL_DIR/manual_control.pid"
  for _ in $(seq 1 20); do health_manual && break; sleep 0.2; done
fi

if ! health_manual; then
  echo "ERROR: XGO manual control service on :8765 is not healthy" >&2
  exit 1
fi

# Refuse to leave a stale dashboard process behind on redeploy.
PID_FILE="$BASE/console.pid"
if [[ -f "$PID_FILE" ]]; then
  oldpid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$oldpid" ]] && kill -0 "$oldpid" 2>/dev/null; then
    oldcmd="$(tr '\0' ' ' < "/proc/$oldpid/cmdline" 2>/dev/null || true)"
    if [[ "$oldcmd" == *"$BASE/server.py"* ]]; then
      kill "$oldpid" 2>/dev/null || true
      for _ in $(seq 1 20); do kill -0 "$oldpid" 2>/dev/null || break; sleep 0.1; done
    fi
  fi
fi

export YDLIDAR_PYTHON
exec "$XGO_PY" "$BASE/server.py"
