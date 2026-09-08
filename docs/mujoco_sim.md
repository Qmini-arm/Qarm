# Qarm 四轴 MuJoCo 动力学环境

该环境直接加载 `description/output_mjcf/robot.xml`，包含固定基座、CAD 质量和惯量、
四个转动关节、简化碰撞体、重力、M8010-6 输出侧扭矩执行器以及位置/速度/扭矩和
末端位姿传感器。控制循环采用参考项目的基本结构：每个 MuJoCo 步更新关节 PD 与
动力学前馈，再调用 `mj_step`；可视化用 passive viewer 与仿真时钟同步。

## 安装与运行

```bash
.venv/bin/python -m pip install -r requirements-sim.txt

# 编译模型并打印自由度、关节限位和执行器限幅
.venv/bin/python -m qarm_sim validate

# 无窗口、可重复的 3 秒闭环测试
.venv/bin/python -m qarm_sim demo --headless --duration 3

# macOS 交互窗口（MuJoCo 要求由 mjpython 启动）
.venv/bin/mjpython -m qarm_sim demo --duration 10 --sine-joint 2

# 对比 MuJoCo 逆动力学与 functions.py 的 PI 经验公式
.venv/bin/python -m qarm_sim compare-gravity --q 0 0.8 0.2 0

# 启动 MuJoCo + Viser FK/IK 工作台
.venv/bin/python -m qarm_sim viser --host 127.0.0.1 --port 8080
# mjviser 是同一个命令的别名
# .venv/bin/python -m qarm_sim mjviser --port 8080
```

浏览器打开 `http://127.0.0.1:8080`。界面包含三组控件：

- **FK / 关节空间**：拖动 `joint_1..joint_4` 角度，实时更新 MuJoCo CAD mesh、末端位置和四元数。
- **IK / 末端位置**：输入目标 XYZ，使用 MuJoCo Jacobian 阻尼最小二乘求位置 IK；初值可选当前 FK 或零位。
- **functions.py / 平面参考**：调用现有 `functions.py` 的二维 `forward_kinematics(r,z)` 和 `inverse_kinematics(r,z)`，用于对比经验平面模型。它只涉及两连杆参考，不会改变 MuJoCo 四轴状态。

Viser 工作台的 IK 只约束末端 XYZ，不约束末端姿态；四个自由度不能承诺任意六维位姿。

`compare-gravity` 中 MuJoCo 姿态设为给定 `q`，速度和加速度设为零，先执行
`mj_forward`，再将 `qacc=0` 后执行 `mj_inverse`；输出的 `qfrc_inverse` 是静态
重力平衡所需的四轴关节力矩。经验公式默认使用
`PI=(4.4, 1.712549, 0.0) N m`，对应 `functions.py` 当前参数。

默认姿态的比较结果为：

| 关节 | MuJoCo (N m) | 经验式 (N m) | 经验式 - MuJoCo (N m) |
|---|---:|---:|---:|
| `joint_1` | -0.0028 | 0.0000 | +0.0028 |
| `joint_2` | -3.6836 | -4.1233 | -0.4397 |
| `joint_3` | +0.9947 | +0.9670 | -0.0277 |
| `joint_4` | -0.0016 | 0.0000 | +0.0016 |

这说明当前经验式在该姿态下主要偏差来自 `joint_2`，约 `0.44 N m`；`joint_3`
相差约 `0.028 N m`。`joint_1` 在旧公式中没有被计算，比较器因此显式填 0，不能
把它误解为已完成的 joint_1 重力建模。不同姿态请重新运行 `--q`，不要把这一组
数值当成全工作空间恒定误差。

## 运行现有 `functions.py` 主循环

`functions.py` 现在只有显式传入 `--sim` 才会使用 MuJoCo；不传时仍然连接
`/dev/ttyUSB0`。主循环没有自动超时，使用 `Ctrl+C` 停止：

```bash
# 无窗口、持续运行；Ctrl+C 停止
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python functions.py --sim

# 指定初始四轴姿态（rad）
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python functions.py --sim \
  --initial-q 0 0.8 0.2 0

# macOS 可视化：viewer 在主线程，控制循环在后台运行
PYTHONDONTWRITEBYTECODE=1 .venv/bin/mjpython functions.py --sim --viewer
```

`--viewer` 只适用于 `--sim`；关闭窗口或按 `Ctrl+C` 都会发送四个电机的
`mode=0` 并关闭仿真线程。该入口保留了 `functions.py` 原有的 `PI_1/PI_2/PI_3`
重力近似，主要用于验证串口兼容调用链；它不是 MuJoCo `qfrc_bias` 精确重力补偿，
因此出现姿态漂移时应先检查控制模型和关节坐标约定。

`demo` 默认在 `[0, 0.8, 0.2, 0] rad` 保持当前位置，并使用 MuJoCo 的
`qfrc_bias` 计算理想重力前馈。可通过 `--target J1 J2 J3 J4`、`--kp`、`--kd`、
`--control-rate` 调整控制。`--kp`/`--kd` 接受一个公共值或四个逐轴值。

## 兼容现有 motor_driver.py

`MotorCmd`、`MotorData` 和 `move()` 不需要改。把真实串口路径换成 `mujoco://` 即可：

```python
from motor_driver import MotorCmd, MotorData, SerialPort, move

ser = SerialPort(
    "mujoco://",
    initial_qpos=[0.0, 0.8, 0.2, 0.0],
    realtime=True,
)
cmd = MotorCmd(id=1, mode=1, q=0.8, kp=0.2, kd=0.03)
data = MotorData()

ser.sendRecv(cmd, data)
move(ser, cmd, data, target=1.0, duration=2.0, kp=0.2, kd=0.03)

# 状态/重力矩/窗口可从仿真对象访问；窗口应在主线程打开。
print(ser.simulation.snapshot())
print(ser.simulation.gravity_compensation())
ser.simulation.launch_viewer(duration_s=10.0)
ser.close()
```

若需要严格可重复、不依赖墙钟时间的测试，使用
`MujocoSerialPort(realtime=False)`，每批命令后显式调用
`ser.simulation.step(n_steps)`。

接口坐标语义与现有驱动一致：

| 字段 | 仿真中的含义 |
|---|---|
| `q`, `dq` | 减速器输出端/URDF 关节的 rad、rad/s |
| `tau` | 输出端理想前馈力矩 N·m |
| `kp`, `kd` | M8010 数据包中的转子侧增益；理想减速器映射到关节侧后乘 `6.33²` |
| `direction`, `offset` | 真实编码器到关节坐标的映射；进入仿真关节坐标后已抵消 |
| `mode=0` | 零电机力矩，仅保留 MJCF 被动阻尼 |
| `mode=1` | M8010 力位混合控制 |

目标位置、速度、增益和力矩会按 MJCF 关节范围、机械臂 3 rad/s 限制、数据包增益范围
及 23.7 N·m 峰值力矩裁剪；`snapshot().command_clipped` 可检查是否发生裁剪。

## 模型边界

23.7 N·m、6.33、30 rad/s 来自 GO-M8010-6 公布参数；关节速度 3 rad/s 来自当前
URDF。MJCF 中每轴 `armature=0.001 kg·m²` 是为小惯量腕部数值稳定设置的保守假设，
不是实测值。以下参数仍需在装配完成的机械臂上辨识后才能用于 sim-to-real：

- 转子惯量和减速器反射惯量；
- 连续力矩、温升和电流/电压包络；
- 库仑摩擦、粘性阻尼、减速器效率和齿隙；
- 串口延迟、丢包、内部控制周期和 `mode=0` 电制动；
- 线缆、桌面固定柔性及外部载荷。

因此当前结果是离线刚体动力学验证，不代表真实机械臂已标定或可安全下发。
