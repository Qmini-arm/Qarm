"""在桌面标定姿态下读取四个电机零位偏置并写入配置。"""

from __future__ import annotations

import argparse
import os
import statistics
import tempfile
import time
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from motor_driver import SerialPort


# 桌面标定姿态中，关节 0~3 在 URDF 坐标系下的弧度值。
CALIBRATION_POSE = (
    0.0,
    1.7480178110996762,
    0.1548064706928587,
    0.0,
)
CONFIG_PATH = Path(__file__).resolve().parent / "config" / "config.py"


def read_offsets(serial_port: SerialPort, sample_count: int) -> dict[int, float]:
    """读取各电机位置；驱动减去标定姿态后得到的值即为零位偏置。"""
    from motor_driver import MotorCmd, MotorData

    offsets: dict[int, float] = {}

    for motor_id, pose_offset in enumerate(CALIBRATION_POSE):
        cmd = MotorCmd(id=motor_id, direction=1, offset=pose_offset)
        data = MotorData()
        samples: list[float] = []
        attempts = 0
        max_attempts = max(sample_count * 5, 20)

        while len(samples) < sample_count and attempts < max_attempts:
            attempts += 1
            if serial_port.sendRecv(cmd, data):
                samples.append(data.q)
            time.sleep(0.01)

        if len(samples) < sample_count:
            raise RuntimeError(
                f"电机 {motor_id} 通信失败：仅收到 {len(samples)}/{sample_count} 个有效数据包"
            )

        # 中位数可避免偶发异常帧影响标定结果。
        offsets[motor_id] = statistics.median(samples)
        print(
            f"电机 {motor_id}: 标定姿态 {pose_offset:+.12f} rad, "
            f"读取偏置 {offsets[motor_id]:+.12f} rad"
        )

    return offsets


def write_config(offsets: dict[int, float]) -> None:
    """以原子替换方式写入 MOTOR_OFFSETS，避免留下半写入的配置。"""
    content = "MOTOR_OFFSETS = {\n" + "".join(
        f"    {motor_id}: {offsets[motor_id]:.15g},\n"
        for motor_id in range(len(CALIBRATION_POSE))
    ) + "}\n"

    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    current_mode = CONFIG_PATH.stat().st_mode if CONFIG_PATH.exists() else None

    temp_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=CONFIG_PATH.parent,
            prefix=".config.py.",
            delete=False,
        ) as temp_file:
            temp_file.write(content)
            temp_file.flush()
            os.fsync(temp_file.fileno())
            temp_name = temp_file.name

        if current_mode is not None:
            os.chmod(temp_name, current_mode)
        os.replace(temp_name, CONFIG_PATH)
    finally:
        if temp_name is not None and os.path.exists(temp_name):
            os.unlink(temp_name)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="电机串口")
    parser.add_argument(
        "--samples",
        type=int,
        default=20,
        help="每个电机的有效采样数（默认：20）",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples <= 0:
        raise SystemExit("--samples 必须大于 0")

    try:
        from motor_driver import SerialPort
    except ModuleNotFoundError as error:
        if error.name == "serial":
            raise SystemExit("缺少 pyserial，请先执行：pip install pyserial") from error
        raise

    print("请先将机械臂固定在桌面标定姿态：")
    print("关节弧度 = " + ", ".join(str(value) for value in CALIBRATION_POSE))
    input("确认姿态正确且机械臂不会移动后，按 Enter 开始标定（Ctrl+C 取消）...")

    serial_port = SerialPort(args.port)
    if serial_port.serial is None or not serial_port.serial.is_open:
        raise SystemExit(f"无法打开串口 {args.port}，未修改 {CONFIG_PATH}")

    try:
        offsets = read_offsets(serial_port, args.samples)
        write_config(offsets)
    except RuntimeError as error:
        raise SystemExit(f"标定失败：{error}；未修改 {CONFIG_PATH}") from error
    finally:
        serial_port.serial.close()

    print(f"标定完成，偏置已写入 {CONFIG_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
