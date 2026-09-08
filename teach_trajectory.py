"""Qmini 四轴拖动示教与 MoveJ 轨迹回放入口。

示例：
  python teach_trajectory.py record --port /dev/ttyUSB0 --output trajectories/demo.json
  python teach_trajectory.py replay --port /dev/ttyUSB0 --input trajectories/demo.json

为避免误触导致真实机械臂动作，两个子命令都要求显式传入 --enable-hardware。
"""
import argparse

from config.config import MOTOR_OFFSETS
from motor_driver import ArmController


def build_parser():
    parser = argparse.ArgumentParser(description="Qmini 拖动示教 / MoveJ 回放")
    parser.add_argument("--port", required=True, help="电机串口，例如 /dev/ttyUSB0")
    parser.add_argument("--enable-hardware", action="store_true",
                        help="确认允许访问真实串口（必需）")
    sub = parser.add_subparsers(dest="action", required=True)
    record = sub.add_parser("record", help="重力补偿拖动并记录轨迹")
    record.add_argument("--output", required=True, help="JSON 轨迹文件")
    record.add_argument("--duration", type=float, default=None, help="录制秒数，默认 Ctrl-C 结束")
    record.add_argument("--sample-period", type=float, default=0.02, help="采样周期（秒）")
    replay = sub.add_parser("replay", help="按轨迹逐段调用 MoveJ")
    replay.add_argument("--input", required=True, help="JSON 轨迹文件")
    replay.add_argument("--speed", type=float, default=1.0, help="时间缩放，越大越快")
    replay.add_argument("--start-duration", type=float, default=2.0,
                        help="到达第一个轨迹点的 MoveJ 时长（秒）")
    return parser


def main():
    args = build_parser().parse_args()
    if not args.enable_hardware:
        raise SystemExit("为安全起见，必须显式指定 --enable-hardware")
    arm = ArmController(args.port, offsets=[MOTOR_OFFSETS[i] for i in range(4)])
    if arm.ser.serial is None or not arm.ser.serial.is_open:
        arm.close()
        raise SystemExit("串口打开失败，未执行示教/回放")
    try:
        if args.action == "record":
            samples = arm.record_trajectory(args.output, args.duration, args.sample_period)
            print(f"已保存 {len(samples)} 个轨迹点: {args.output}")
        else:
            arm.replay_trajectory(args.input, speed=args.speed, start_duration=args.start_duration)
            print(f"回放完成: {args.input}")
    finally:
        arm.disable()
        arm.close()


if __name__ == "__main__":
    main()
