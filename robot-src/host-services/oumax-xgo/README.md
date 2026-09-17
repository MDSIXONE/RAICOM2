# OUMAX-XGO 机器端控制服务

本目录保存从机器狗（`192.168.10.15`，用户 `pi`）拉取的控制代码。
原路径：`/home/pi/oumax-xgo/`。

## 文件

| 文件 | 拉取时机器端 md5 | 说明 |
| --- | --- | --- |
| `manual_control_server.py` | `88b1f68651e37b6c00e8b1f110c6a06f` | 手动控制 HTTP 服务（端口 8765，游戏手柄 UDP 8766）；基于 `xgolib`，串口 `/dev/ttyAMA0` |
| `picam_mjpeg_server.py` | `08da7183235c05cfa1c7102888da0998` | 树莓派摄像头 MJPEG 视频流服务 |

机器端未发现相关的 systemd 服务或 `/usr/local/sbin` 控制脚本；`/home/pi/oumax-xgo/`
（含 `manual_control.log`、`manual_control.pid`、`picam_mjpeg.log`、`picam_mjpeg.pid`）
即控制代码全集。日志与 pid 文件为运行产物，未纳入本仓库。

## 拉取方式

```bash
scp pi@192.168.10.15:/home/pi/oumax-xgo/manual_control_server.py .
scp pi@192.168.10.15:/home/pi/oumax-xgo/picam_mjpeg_server.py .
```

## 说明

- `manual_control_server.py` 从环境变量读取配置：`OUMAX_MANUAL_PORT`（默认 8765）、
  `OUMAX_GAMEPAD_UDP_PORT`（默认 8766）、`OUMAX_SERIAL_PORT`（默认 `/dev/ttyAMA0`）、
  `OUMAX_DOG_VERSION`（默认 `auto`）。
- 本仓库中机器端源码的写入边界见根 `README.md`：机器相关代码只允许写入
  `robot-src/catkin_ws/src/` 或 `robot-src/host-services/<device>/`。
