# ruikon 主任务

机器端目录为 `/home/pi/dogrk`。网页控制入口由 `../host-services/lidar-console/dogrk_supervisor.py` 提供，监听 8088/8091，由用户点击启动，不在开机时自动运动。旧导航与串口网关服务暂停。

`ruikon.py` 执行巡逻识别和两轮抓放，`lib/Motion.py` 管理 IMU 朝向；`lib/catch_ball.py` 管理找球与抓取，`lib/place_abcd.py` 管理字母搜索与放置。CSI 相机由 `lib/csi_camera.py` 取帧。

抓球使用 `best.onnx`，识别区使用 `model.onnx`，放置使用 `letters.onnx`，模型均需预先位于 `/home/pi/ros_ws/models/`。不将模型或原始压缩包加入源码提交。复用的 `recognize_at_point.py`、`delivery_vision.py` 和 `capture_photos.py` 需部署在 `/home/pi/oumax-lidar-console/`。机器 Python 环境需有厂商 xgolib、xgoedu，以及 Picamera2、OpenCV、NumPy、ONNX Runtime。播报依赖机器上相应音频文件。

当前抓球框半径小于 18px 时前进指令3、持续1秒，近处持续0.15秒，达到24px开始夹取；夹取后恢复找球前朝向。放置搜索横移每次1.5秒，看到其他字母时按 ABCD 排列判断左右；A点释放框半径22px，其他点24px。所有像素参数按320px图像宽度归一化，真实距离和落点需实机标定。

本地不连接硬件的检查：

```powershell
python robot-src/dogrk/test_catch_ball_control.py
python robot-src/dogrk/test_place_control.py
```

测试覆盖动作决策、速度时长和阈值，不能替代实机夹持和落点验证。旧颜色检测及字母检测实现保留在模块中，当前流程使用上述 ONNX 检测入口。
