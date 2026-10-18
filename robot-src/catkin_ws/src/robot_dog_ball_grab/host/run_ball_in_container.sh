#!/usr/bin/env bash
# 球程序容器内运行入口（在 ros-noetic-ball 容器内执行，勿在宿主机执行）。
#
# 设置运行环境（host 系统库兜底 + RPi 检测环境变量）后用容器内 ballenv
# 运行 catkin 包 robot_dog_ball_grab 的球编排脚本。
#
# 用法（容器内或经 docker exec）：
#   run_ball_in_container.sh [--enable-motion] [其他 ball_grab_release.py 参数...]
set -euo pipefail

export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu-host:${LD_LIBRARY_PATH:-}"
export RPI_LGPIO_REVISION="${RPI_LGPIO_REVISION:-00c041a0}"

SCRIPT="/root/catkin_ws/src/robot_dog_ball_grab/scripts/ball_grab_release.py"
exec /opt/ballenv/bin/python "$SCRIPT" "$@"
