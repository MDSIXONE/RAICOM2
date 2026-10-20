# 当前主程序：ruikon

2026-10-21 起，ruikon 为当前主程序。控制页同时开放在 `http://192.168.31.35:8088/` 和 `http://192.168.31.35:8091/`，点击「开始主任务」运行，点击「停止任务」结束。开机只启动网页控制服务，不自动运动。

旧 `oumax-lidar-console.service` 和 `oumax-xgo.service` 已停止并禁用开机自启，源码保留；停止当前任务不会恢复旧服务。主程序直接独占串口和CSI摄像头，模型与检测库仍使用机器上现有文件。

部署备份：`/home/pi/delivery-backups/promote-ruikon-20261021`。下方为切换前的历史说明，其中自动恢复旧主服务的行为已停用。

---

# DOGRk 备用任务选择页

独立服务：`dogrk-supervisor.service`，端口 **8091**。它不依赖主控制台，主控制台被停掉时页面仍然在线。

打开：

```text
http://192.168.31.35:8091/
```

主任务页面 8088 顶部也有「任务切换」链接。

主页面配送区点击「运行备用 ruikon」，会直接启动 `ruikon.py` 并跳转到独立控制页；独立页面也提供专用的「运行 ruikon.py」按钮。点击「停止备用并恢复主服务」结束程序并恢复主控制台。启动入口固定为 `ruikon.py`，不受其他入口选择影响。

## 页面能力

- 主任务：链接到 `http://192.168.31.35:8088/#delivery`，由原页面启动/停止识别配送。
- 备用方案 DOGRk：选择入口后启动；启动前会停掉 `oumax-lidar-console.service` 和 `oumax-xgo.service`，释放 `/dev/ttyAMA0` 和摄像头 0。
- 停止备用：结束 DOGRk 进程组，执行 `safe_stop.py` 停车，然后按“先串口网关、后控制台”的顺序恢复主服务。
- 自检入口 `selftest.py`：不打开串口/摄像头、不动作，只验证任务切换与恢复链路。

## 入口

| 入口 | 用途 |
| --- | --- |
| `dog1_main.py` | 狗1 仪表识别，向 `10.187.166.14:9999` 发送（需要另一台狗/接收端） |
| `dog2_main.py` | 狗2 安全帽、抓球、放置；当前脚本里异常字母固定为 B/C |
| `ruikon.py` | 巡检字母/物品识别 + 抓放 |
| `chuangyi_main.py` | 单放 D 区的最小测试 |
| `selftest.py` | 不动作自检，5 秒退出 |

狗1/狗2 角色不确定时先跑 `selftest.py`，确认页面切换/恢复；再用 `chuangyi_main.py` 或对应入口短试。备用方案是独立旧程序，直接开环定时移动，不经过当前 LiDAR 避障/前停；启动前必须清空场地并手边可停止。

## 文件与服务

机器端：

```text
/home/pi/dogrk/                    # DOGRk 代码 + 4 个 ONNX 模型
/home/pi/dogrk-supervisor/         # supervisor、safe_stop.py、logs/
/home/pi/Desktop/DOG1 -> /home/pi/dogrk
/etc/systemd/system/dogrk-supervisor.service
```

日志在 `/home/pi/dogrk-supervisor/logs/`，页面底部也会显示最近输出。

## 故障恢复

如果页面显示备用退出但主服务没有恢复：

```bash
sudo systemctl start oumax-xgo.service
sudo systemctl start oumax-lidar-console.service
sudo systemctl status dogrk-supervisor.service
```

如果 DOGRk 进程卡住，先点页面“停止备用并恢复主服务”；仍不行则：

```bash
sudo systemctl stop dogrk-supervisor.service
sudo pkill -f '/home/pi/dogrk/'
sudo systemctl start oumax-xgo.service
sudo systemctl start oumax-lidar-console.service
sudo systemctl start dogrk-supervisor.service
```

## 边界

- 备用程序不是当前 LiDAR 执行器，没有雷达前停、定位看门狗或路线校验。
- 停止通过进程组 + `xgolib.stop()`；若串口被异常占用，恢复主服务可能失败，需要重启对应服务。
- 当前部署只验证了 `selftest.py` 的切换/恢复，未验证任何真实 DOGRk 运动入口。
