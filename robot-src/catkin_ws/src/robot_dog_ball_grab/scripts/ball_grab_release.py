#!/usr/bin/env python3
"""抓球 → 旋转 180° → 放球 一键编排程序。

顺序运行三个独立程序（各自取得/释放串口控制权，无跨进程冲突）：
1. 抓球程序 ball_yolo_grab.py（YOLO 检测 → 接近 → 夹球）
2. 旋转程序 rotate.py（掉头 180°，默认）
3. 放球程序 ball_release.py（低趴 → 视觉跟踪对齐 → 张爪放球）

四个脚本同目录存放（仓库在 robot_dog_ball_grab/scripts/，
机器端部署在 /home/pi/oumax-xgo/），默认路径按编排脚本同目录解析。
"""

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--grab-script", default=os.path.join(HERE, "ball_yolo_grab.py"))
    parser.add_argument("--rotate-script", default=os.path.join(HERE, "rotate.py"))
    parser.add_argument("--release-script", default=os.path.join(HERE, "ball_release.py"))
    parser.add_argument("--model", default="/home/pi/ros_ws/models/best.onnx")
    parser.add_argument("--target-radius", type=float, default=28.0)
    parser.add_argument("--confidence", type=float, default=.60)
    parser.add_argument("--turn-speed", type=float, default=-15.0,
                        help="抓球后旋转的转向速度（透传给 rotate.py）")
    parser.add_argument("--turn-duration", type=float, default=9.0,
                        help="旋转脉冲时长秒数；180° 掉头时长按实机标定，当前 9.0s（透传给 rotate.py）")
    parser.add_argument("--enable-motion", action="store_true",
                        help="显式开启才允许真实运动（透传给三个阶段）")
    args = parser.parse_args()

    base = [sys.executable]
    grab_cmd = base + [args.grab_script, "--model", args.model,
                       "--target-radius", str(args.target_radius),
                       "--confidence", str(args.confidence)]
    rotate_cmd = base + [args.rotate_script,
                         "--turn-speed", str(args.turn_speed),
                         "--turn-duration", str(args.turn_duration)]
    release_cmd = base + [args.release_script, "--model", args.model,
                          "--target-radius", str(args.target_radius),
                          "--confidence", str(args.confidence)]
    if args.enable_motion:
        grab_cmd.append("--enable-motion")
        rotate_cmd.append("--enable-motion")
        release_cmd.append("--enable-motion")

    print("stage=1/3 ball-grab", flush=True)
    subprocess.run(grab_cmd, check=True)
    print("stage=2/3 rotate", flush=True)
    subprocess.run(rotate_cmd, check=True)
    print("stage=3/3 ball-release", flush=True)
    subprocess.run(release_cmd, check=True)
    print("action=grabrelease-complete", flush=True)


if __name__ == "__main__":
    main()
