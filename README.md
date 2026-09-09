# Qarm

Qmini 四轴机械臂的串口控制、零位标定、拖动示教、连续轨迹回放及 Viser 可视化工具。

## 环境准备

以下命令均在 `Qarm` 目录执行：

```bash
uv venv
uv pip install -r requirements-viz.txt
```

该依赖文件包含串口、数值计算和可视化依赖。实机程序使用电机 ID `0～3`，零位偏置位于 `config/config.py`，模型与限位位于 `description/qmini_arm.urdf`。

## Python 文件一览

| 文件 | 用途 | 运行方式 |
| --- | --- | --- |
| [callibration.py](callibration.py) | 在桌面标定姿态下采样，计算四轴零位偏置并写入配置。文件名中的双 `l` 按仓库原名保留。 | 标定命令见下文。 |
| [read_motors.py](read_motors.py) | 持续显示四轴位置 `P`（rad）和速度 `V`（rad/s），发送的命令为电机停用模式。 | `uv run read_motors.py`；串口在代码中固定为 `/dev/ttyUSB0`，Ctrl-C 退出。 |
| [teach_trajectory.py](teach_trajectory.py) | 重力补偿拖动录制；读取 JSON，先到起点，再连续插值回放。 | `record` / `replay` 命令见下文。 |
| [viser_app.py](viser_app.py) | 浏览器内关节调节、基于 URDF 的正运动学和末端位置逆运动学、实机控制与实时反馈显示。 | 三种模式命令见下文。 |
| [kinematics.py](kinematics.py) | 简化双连杆正逆运动学、腕部姿态计算；独立入口演示实机运动。 | `uv run kinematics.py`；详情见下文。 |
| [motor_driver.py](motor_driver.py) | 串口协议、反馈解析、零位/方向转换，以及 `ArmController` 的限位、MoveJ、示教和回放控制。 | 通常由其他脚本导入；`uv run motor_driver.py` 仅尝试打开 `/dev/ttyUSB0` 并创建单电机对象，不发送运动命令。 |
| [gravity.py](gravity.py) | 重力补偿力矩、力矩缓启动及手腕水平姿态计算。 | 作为库导入；独立入口未完成，见下文。 |
| [joint_trajectory.py](joint_trajectory.py) | 保形三次插值，按时间输出连续位置与速度，整段起止速度为零。 | 库模块，无独立启动入口。 |
| [config/config.py](config/config.py) | 保存四个电机的 `MOTOR_OFFSETS` 零位偏置。 | 配置模块，由标定脚本写入、其他模块读取。 |
| [tests/test_viser_app.py](tests/test_viser_app.py) | 离线检查四轴 URDF、正逆运动学，以及模拟控制器的实机接管逻辑。 | 测试命令见下文，不连接机械臂。 |

## 零位标定

先将机械臂固定在脚本提示的桌面标定姿态，再按 Enter 开始采样。成功后会**覆盖 `config/config.py` 中的偏置配置**。

```bash
uv run callibration.py --port /dev/ttyUSB0 --samples 20
```

`--samples` 是每个电机的有效采样数，默认 20，使用中位数计算偏置。

## 拖动示教与轨迹回放

```bash
# 重力补偿下手动拖动；Ctrl-C 结束录制并保存文件
uv run teach_trajectory.py record --port /dev/ttyUSB0 --enable-hardware --output trajectories/demo.json

# 读取已录制轨迹并驱动机械臂回放
uv run teach_trajectory.py replay --port /dev/ttyUSB0 --enable-hardware --input trajectories/demo.json
```

| 子命令 | 可选参数 | 功能 / 默认值 |
| --- | --- | --- |
| `record` | `--duration 10` | 录制 10 秒；不传则由 Ctrl-C 结束。 |
| `record` | `--sample-period 0.02` | 每轮采样后的等待时间，默认 0.02 秒；实际采样间隔还包含通信耗时。 |
| `replay` | `--speed 0.5` | 半速回放；默认 `1.0`，数值越大越快。 |
| `replay` | `--start-duration 2` | 移动到轨迹起点的时长，默认 2 秒。 |
| `replay` | `--control-period 0.005` | 连续回放目标更新周期，默认 5 ms；实际频率受通信耗时限制。 |

两个子命令均要求 `--enable-hardware`；上述参数放在 `record` 或 `replay` 后。轨迹 JSON 保存 `time`（秒）和四轴 `q`（弧度），可重复回放。

## Viser 浏览器界面

启动后打开 <http://127.0.0.1:8080>。

```bash
# 离线关节滑条调节，观察模型姿态与末端位置
uv run viser_app.py --sim --mode viewer

# 离线拖动末端目标，求解位置逆运动学
uv run viser_app.py --sim --mode ik

# 通过浏览器关节滑条控制实机；改为 --mode ik 可控制末端目标位置
uv run viser_app.py --enable-hardware --device /dev/ttyUSB0 --mode viewer

# 将当前四轴实机反馈实时显示在模型上
uv run viser_app.py --enable-hardware --device /dev/ttyUSB0 --mode replay
```

实机 `viewer` / `ik` 模式连接后，还需在浏览器勾选“启用实机驱动”，程序会先接管当前位置。`replay` 模式显示串口实时反馈，不读取示教 JSON；启动时发送停用模式的轮询命令，不适合与其他控制程序同时占用串口。

常用参数：`--port 8080` 为网页端口，`--device` 为串口；`--move-duration 0.5` 为浏览器目标的 MoveJ 时长，`--replay-period 0.05` 为反馈显示循环的等待时间。更多说明见 [VISER.md](VISER.md)。

## 运动学示例与库模块

```bash
uv run kinematics.py
```

该命令会连接代码中固定的 `/dev/ttyUSB0`，计算示例腕部目标 `(0.2, 0.2, 0.3)` 米对应的关节角，加载重力补偿后执行 5 秒 MoveJ，随后持续打印反馈，Ctrl-C 退出。它会实际驱动机械臂；修改目标和串口需编辑脚本。

`gravity.py` 的独立入口目前会先使能电机，随后因 `q0` 等变量未定义而报错，因此应仅导入其函数使用；拖动示教请使用 `teach_trajectory.py record`。

## 离线测试

```bash
uv pip install pytest
uv run python -m pytest tests/test_viser_app.py -q
```
