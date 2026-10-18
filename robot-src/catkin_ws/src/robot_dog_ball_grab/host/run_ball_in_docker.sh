#!/usr/bin/env bash
# 抓球/放球容器化运行启动器（宿主机执行）：在球程序容器 ros-noetic-ball 内
# 运行 ball_grab_release.py（或经 -- 透传其他球脚本参数），容器内环境由
# setup_ball_container.sh 配置（ballenv + 宿主厂商环境复用）。
#
# 用法：
#   run_ball_in_docker.sh [--release-camera-serial] [-- 参数...]
# 默认执行 ball_grab_release.py（无 --enable-motion 时只检测不动作）。
set -euo pipefail

CONTAINER="${RAICOM_BALL_CONTAINER:-ros-noetic-ball}"
RUNNER="/root/catkin_ws/src/robot_dog_ball_grab/host/run_ball_in_container.sh"

RELEASE_SERVICES=0
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --release-camera-serial)
      RELEASE_SERVICES=1
      shift
      ;;
    --)
      shift
      EXTRA_ARGS=("$@")
      break
      ;;
    *)
      EXTRA_ARGS=("$@")
      break
      ;;
  esac
done

if [[ "$RELEASE_SERVICES" -eq 1 ]]; then
  echo "Stopping oumax-camera + oumax-manual on host (release camera & serial)..."
  sudo systemctl stop oumax-camera.service
  sudo systemctl stop oumax-manual.service
fi

echo "Running ball program inside ${CONTAINER}: ${EXTRA_ARGS[*]:-(default ball_grab_release.py)}"
docker exec "$CONTAINER" "$RUNNER" "${EXTRA_ARGS[@]}"
