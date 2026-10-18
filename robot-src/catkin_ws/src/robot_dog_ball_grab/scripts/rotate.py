#!/usr/bin/env python3
"""旋转程序：抓球完成后掉头（默认 180°），使放球方向对准目标区域。

供 ball_grab_release.py 编排链调用，也可单独运行。独立进程，自己取得串口控制权，
退出即释放，不与抓球/放球程序冲突。

实现说明：`dog.attitude("y", 180)` 大角度姿态命令实机不生效（日志成功但机器不动），
因此改用实机验证过的转向速度脉冲 `dog.turn(speed)` 持续 `duration` 秒。
抓球对准用 turn(-15) × 0.74s 实测可转（角度未标定），180° 掉头时长需现场标定
（默认 5.0 秒，按实机角度调整）。
"""

import argparse
import time

from uiutils import dog


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--turn-speed", type=float, default=-15.0,
                        help="转向速度（与抓球对准同款 -15 实测可转）")
    parser.add_argument("--turn-duration", type=float, default=9.0,
                        help="转向脉冲时长秒数；180° 掉头时长按实机标定，当前 9.0s")
    parser.add_argument("--settle", type=float, default=3.0,
                        help="转向停止后的等待秒数，让车体稳定")
    parser.add_argument("--enable-motion", action="store_true",
                        help="显式开启才允许真实运动")
    args = parser.parse_args()
    if not args.enable_motion:
        print("motion disabled; holding position", flush=True)
        return
    print(f"action=rotate-start speed={args.turn_speed} duration={args.turn_duration}",
          flush=True)
    dog.turn(args.turn_speed)
    time.sleep(args.turn_duration)
    dog.stop()
    time.sleep(args.settle)
    print("action=rotate-complete", flush=True)


if __name__ == "__main__":
    main()
