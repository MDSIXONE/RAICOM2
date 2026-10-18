#!/usr/bin/env bash
# 球程序容器 ros-noetic-ball 创建与配置（幂等，宿主机执行）。
#
# 背景：导航容器 ros-noetic 基于 Ubuntu Focal（ROS Noetic 要求），无法直接
# 复用宿主机 xgovenv（Python 3.11 + libcamera 0.3 与 Focal 不兼容）；球程序
# 容器改用 Debian Bookworm（与宿主机同基线），复用宿主机厂商环境
# （/home/pi 挂载 + host-dist-packages + host 系统库），容器内自建 ballenv
# （numpy/cv2/onnxruntime 对齐宿主版本），实现球程序全部容器内运行。
#
# 用法（宿主机）：
#   sudo bash setup_ball_container.sh
# 幂等：容器已存在则跳过创建；apt/venv/pip/.pth 每步均可重复执行。
set -euo pipefail

IMAGE="m.daocloud.io/docker.io/library/debian:bookworm-slim"
NAME="${RAICOM_BALL_CONTAINER:-ros-noetic-ball}"

if ! docker inspect "$NAME" >/dev/null 2>&1; then
  docker run -d --name "$NAME" \
    --network host \
    --privileged \
    --restart unless-stopped \
    -v /home/pi:/home/pi \
    -v /home/pi/ros_ws:/root/catkin_ws \
    -v /usr/lib/python3/dist-packages:/usr/lib/python3/host-dist-packages:ro \
    -v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu-host:ro \
    -v /usr/lib/aarch64-linux-gnu/libcamera:/usr/lib/aarch64-linux-gnu/libcamera:ro \
    -v /usr/share/libpisp:/usr/share/libpisp:ro \
    -v /usr/share/libcamera:/usr/share/libcamera:ro \
    -v /dev:/dev \
    "$IMAGE" sleep infinity
  echo "container created: $NAME"
else
  echo "container exists: $NAME"
fi

docker exec "$NAME" bash -lc 'set -e
  apt-get update -qq
  apt-get install -y -qq python3 python3-venv libblas3 liblapack3 libarmadillo11 libpulse0 ffmpeg udev
  [ -x /opt/ballenv/bin/python ] || python3 -m venv /opt/ballenv
  /opt/ballenv/bin/pip install -q -i https://pypi.tuna.tsinghua.edu.cn/simple numpy==1.24.2 opencv-python==4.11.0.86 onnxruntime==1.20.0
  printf "%s\n%s\n%s\n" /usr/lib/python3/host-dist-packages /home/pi/RaspberryPi-CM5/xgovenv/lib/python3.11/site-packages /home/pi/RaspberryPi-CM5/uiutils/src > /opt/ballenv/lib/python3.11/site-packages/extra.pth
  ln -sf /usr/bin/python3 /usr/bin/python
  # 容器内跑 udevd：libcamera 的 udev 枚举器依赖 /run/udev/data 数据库
  #（设备属性/依赖计数），无 udevd 时相机枚举为空
  pgrep -x systemd-udevd >/dev/null || /lib/systemd/systemd-udevd --daemon
  sleep 1
  udevadm trigger
'

# pinctrl 工具：Debian 官方源无此包，从宿主机复制（rpi-lgpio 的 GPIO.setmode 需要）
docker cp /usr/bin/pinctrl "$NAME":/usr/local/bin/pinctrl

echo "ball container ready: $NAME"
