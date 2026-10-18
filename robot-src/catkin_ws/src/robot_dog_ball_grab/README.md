# robot_dog_ball_grab

> 2026-10-18 自上级仓库 `RAICOM` 整包移入本仓库，脚本内容与机器端部署路径不变。本包
> 只作为代码归档，运行时不依赖 ROS 编译（`package.xml`/`CMakeLists.txt` 仅为历史包装）。
> 本仓库当前只移入本包：下文提到的 `robot_dog_navigation/`、`robot_dog_teleop/` 与
> `robot-src/host-services/oumax-xgo/ball_yolo_grab.py` 尚未移入，对应交叉引用暂不指向
> 本仓库现有文件。

本次睿抗国赛抓球/放球任务的代码存放包。四个程序同目录存放（仓库 `scripts/`，
机器端部署 `/home/pi/oumax-xgo/`）：

| 程序 | 说明 |
| --- | --- |
| `ball_yolo_grab.py` | 机器端独立 YOLO 抓球程序：厂商硬件接口（`xgolib`/`uiutils`）+ 项目训练的 ONNX 球模型（YOLOv8 导出） |
| `ball_release.py` | 放球程序（抓球逆操作）：低趴 → 视觉跟踪对齐 → 张爪放球 |
| `rotate.py` | 旋转程序：抓球完成后掉头 180°（turn 脉冲，时长按实机标定） |
| `ball_grab_release.py` | 一键编排：抓球 → 旋转 180° → 放球 |

`scripts/yolo_view.py` 是独立的检测检查工具（画全部 NMS 框、8090 端口 MJPEG 推流、
不驱动狗），不参与抓取流程。

## 运行环境

程序统一在机器端球容器 `ros-noetic-ball` 内运行（导航容器 `ros-noetic` 基于 Ubuntu
Focal，与宿主机 Python 3.11 / libcamera 0.3 不兼容，故球程序用独立 Bookworm 容器，
与宿主机同基线）：

- 容器内自建 `ballenv`（`/opt/ballenv`）：numpy 1.24.2、opencv-python 4.11.0.86、
  onnxruntime 1.20.0（与宿主 xgovenv 版本对齐）；
- `.pth` 兜底复用宿主机 `/usr/lib/python3/dist-packages`（picamera2/libcamera/RPi）
  与 xgovenv site-packages（uiutils/xgolib/xgoscreen，含 editable 的 uiutils src）；
- 容器挂载宿主 `/home/pi`、`/dev`（全设备）、系统库与
  libcamera IPA / libpisp / libcamera tuning 数据；容器内运行 udevd（libcamera
  相机枚举依赖 /run/udev/data）。
- ONNX 模型：默认 `/home/pi/ros_ws/models/best.onnx`。

容器环境由 `host/setup_ball_container.sh` 一键配置（幂等，详见脚本头部注释）。

## 部署与运行

```bash
# 1. 配置球容器（宿主机执行一次，幂等）
bash robot-src/catkin_ws/src/robot_dog_ball_grab/host/setup_ball_container.sh

# 2. 部署脚本：上传到机器 catkin_ws 本包（球容器内 /root/catkin_ws/...）
scp scripts/*.py host/*.sh pi@192.168.137.157:/home/pi/ros_ws/src/robot_dog_ball_grab/{scripts,host}/
scp scripts/*.py pi@192.168.137.157:/home/pi/ros_ws/src/robot_dog_ball_grab/scripts/

# 3. 容器化运行（默认只检测，不动作；--release-camera-serial 先停
#    oumax-camera/oumax-manual 释放相机与串口）
bash robot-src/catkin_ws/src/robot_dog_ball_grab/host/run_ball_in_docker.sh \
  --release-camera-serial -- --enable-motion

# 等价的手动容器内命令
docker exec ros-noetic-ball /root/catkin_ws/src/robot_dog_ball_grab/host/run_ball_in_container.sh \
  --enable-motion
```

主流程（`robot_dog_navigation/scripts/main_flow.py`）导航完成后经 ssh 在宿主机
`docker exec` 进球容器运行本包球编排（见该包 README「主流程」章节）。

运行前注意：先停 `raicom-original-main.service` 与 `oumax-camera.service`
（原厂主服务占 SPI/串口、相机服务占 Picamera2，会导致 xgolib 挂死或相机打不开），
结束后可恢复。

## 抓球程序参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--model` | `/home/pi/ros_ws/models/best.onnx` | ONNX 球检测模型 |
| `--target-radius` | `28.0` | YOLO 框半径达到该像素值后抓取；需按实机继续标定 |
| `--confidence` | `0.60` | 检测置信度阈值 |
| `--enable-motion` | 关 | 显式开启才允许真实运动 |
| `--frames` | `0` | 仅检测 N 帧后退出；0 持续运行 |

## 抓取流程

1. 设定 `slow_trot` 步态 → 低趴（`z=10`、`p=15`，**接近阶段不加 yaw 补偿**）。
2. 检测球：水平偏差 >25px 转向脉冲；框半径小于目标时小步前进。
3. 框半径达标后：重新低趴（对齐脉冲可能让车体恢复站姿）→ 后肢抬高（`31=26°、41=25°`）
   → 机械臂安全序列 `51=-65 → 52=-50 → 53=90 → 52=-45 → 51=40 → 53=0 → 52=0`
   → 车体 yaw 复原。抓球瞬间使用车体 y=-3 补偿机械臂左偏。

## 一键编排：抓球 → 旋转 180° → 放球

```bash
python3 ball_grab_release.py --enable-motion
```

- 阶段 1：`ball_yolo_grab.py`：YOLO 检测 → 对准接近 → 夹球。
- 阶段 2：`rotate.py`：turn 脉冲掉头（默认速度 -15、时长 9s，按实机标定）。
- 阶段 3：`ball_release.py`：低趴 → 视觉跟踪对齐（球模型暂代，后续换字母模型
  `letters.onnx`；60s 超时后仍继续放球）→ 张爪放球（放球瞬间 y=-3 补偿）→ 收臂。

编排参数 `--model`/`--target-radius`/`--confidence`/`--turn-duration`/`--enable-motion`
会透传给对应阶段；日志按 `stage=1/3`…`action=grabrelease-complete` 输出，供持续监控判定。

注意：180° 掉头用 `dog.turn` 转向脉冲实现（`dog.attitude("y", 180)` 大角度命令
实机不生效，已弃用）；掉头时长（默认 9s）需按实机标定。

关节方向标定、安全顺序与低趴姿态详见 `docs/lingo.md`（机械臂关节 / 低趴词条）。

## 相关文件

- 本包四个脚本（机器端部署路径 `/home/pi/oumax-xgo/`）。
- `robot-src/host-services/oumax-xgo/ball_yolo_grab.py`：抓球程序原件副本
  （与 `scripts/ball_yolo_grab.py` 内容一致，需保持同步）。
- `robot-src/catkin_ws/src/robot_dog_teleop/scripts/ball_align_grab.py`：另一条
  基于手控服务 HTTP 的自动对准夹球方案（ROS launch 方式）。
- 模型：`best.onnx`（球模型）、`letters.onnx`（A/B/C/D 字母模型，机器端
  `/home/pi/ros_ws/models/`）。
