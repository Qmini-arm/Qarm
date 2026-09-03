# Qmini Unitree Arm

`Qmini_unitree_arm` 是面向 Qmini 六轴机械臂后续开发的混合工程。C++14 层完成了
GO-M8010-6 通信、转子/关节坐标换算和台架测试；Python 层完成了基于 URDF 的 FK、
无自碰撞工作空间采样、位置 IK、轨迹规划、M8010 命令生成和浏览器实时可视化。
工程引用相邻的 `unitree_actuator_sdk`，不会修改 SDK 仓库。

当前可执行程序：

- `qmini_motor_state`：读取指定串口、指定 ID 的电机状态；
- `qmini_sine_position`：让 ID 0–5 执行相同的相对正弦位置测试。
- `qmini-motion`：离线 FK/IK、可达空间、轨迹与可视化入口（不打开串口）。

## 目录结构

```text
Qmini_unitree_arm/
├── CMakeLists.txt
├── cmake/
│   └── UnitreeActuatorSDK.cmake  # SDK 路径、架构和共享库导入
├── include/qmini_arm/
│   ├── types.hpp                 # MotorCommand/MotorState/JointState
│   ├── motor_bus.hpp             # SDK 无关的公开通信接口
│   ├── joint_conversion.hpp      # 转子侧与机械关节侧换算
│   ├── safety.hpp                # 通用反馈和运动保护
│   └── sine_trajectory.hpp       # SI 单位的轨迹模块
├── src/                          # 公共库实现，SDK 细节只在这里出现
├── apps/
│   ├── read_motor_state.cpp      # 状态读取工具
│   ├── sine_position_test.cpp    # 六电机正弦位置测试
│   └── cli_utils.hpp             # 两个工具共用的参数解析
├── tests/                        # C++ 和 Python 离线测试
├── docs/                         # 架构与运动规划说明
├── config/m8010_arm.yaml         # 六关节 ID、方向、零位和控制参数
├── python/qmini_arm_motion/      # FK/IK/碰撞/规划/命令/可视化
└── pyproject.toml
```

应用只依赖 `qmini_arm_core` 的公开头文件，不直接使用 `MotorCmd`、`MotorData` 或 `SerialPort`。未来替换通信后端、增加仿真后端或 ROS 2 适配时，不需要改动 IK 和轨迹层。

## 编译与离线测试

```bash
cd /home/wyt06/unitree-arm
cmake -S Qmini_unitree_arm -B Qmini_unitree_arm/build
cmake --build Qmini_unitree_arm/build -j2
ctest --test-dir Qmini_unitree_arm/build --output-on-failure
```

默认 SDK 路径是 `/home/wyt06/unitree-arm/unitree_actuator_sdk`。若 SDK 位于其他位置：

```bash
cmake -S Qmini_unitree_arm -B Qmini_unitree_arm/build \
  -DUNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk
```

## FK、IK、无自碰撞规划与可视化

Python 运动层沿用了 `/home/wyt06/Qmini-arm` 中经过验证的“URDF 模型—阻尼最小二乘
IK—工作空间采样—Viser”思路，并针对当前 M8010 URDF 增加圆柱碰撞体、连续路径碰撞
检查、RRT-Connect 兜底规划和转子侧控制参数输出。

安装在独立虚拟环境中：

```bash
cd /home/wyt06/unitree-arm/Qmini_unitree_arm
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev,viz]'
.venv/bin/pytest -q
```

查看零位 FK：

```bash
.venv/bin/qmini-motion fk --q-deg 0 0 0 0 0 0
```

采样 100000 个关节姿态，以 FK 构建无自碰撞可达区域并保存：

```bash
.venv/bin/qmini-motion workspace \
  --samples 100000 \
  --output build/collision_free_workspace.npz
```

从 URDF 零位规划到 `base_link` 坐标系中的目标点，并导出每个控制周期、每台电机的
控制参数：

```bash
.venv/bin/qmini-motion plan \
  --start-deg 0 0 0 0 0 0 \
  --target 0.10 -0.45 0.20 \
  --output build/m8010_commands.csv
```

启动可视化：

```bash
.venv/bin/qmini-motion viz --host 127.0.0.1 --port 8080
```

浏览器中可拖动目标点、规划并播放轨迹、显示无自碰撞可达空间，并实时查看 ID 0–5
的关节目标、转子位置/相对偏移、目标速度、`kp`、`kd` 和前馈力矩。可视化和 CSV
导出均不会打开 `/dev/ttyUSB0`。

完整的算法边界、控制参数语义和真机接入条件见
[运动规划说明](docs/motion_planning.md)。

## 读取电机状态

读取 `/dev/ttyUSB0` 上 ID 0 的一个状态样本：

```bash
./Qmini_unitree_arm/build/qmini_motor_state \
  --port /dev/ttyUSB0 \
  --id 0
```

程序要求输入 `READ` 才会打开串口。M8010 是请求—应答设备，所谓“读取”并不是被动监听：程序发送 `tau=dq=q=kp=kd=0` 的 FOC 请求再取得反馈。这会释放主动保持，不是急停；电机不得支撑会因失力而坠落的机械臂。

持续读取，并把启动时位置临时定义为关节 0°：

```bash
./Qmini_unitree_arm/build/qmini_motor_state \
  --port /dev/ttyUSB0 \
  --id 0 \
  --samples 0 \
  --rate-hz 10 \
  --relative-to-start
```

`--samples 0` 表示持续运行到 Ctrl+C。临时零位只对本次进程有效。

如果已经通过可靠的机械找零获得转子零位，可以显式换算关节角：

```bash
./Qmini_unitree_arm/build/qmini_motor_state \
  --port /dev/ttyUSB0 \
  --id 0 \
  --samples 100 \
  --rate-hz 10 \
  --direction 1 \
  --rotor-zero-rad -1529.46704 \
  --joint-zero-deg 0
```

换算关系为：

```text
q_joint = joint_zero
        + direction * (q_rotor - rotor_zero) / gear_ratio
```

如果没有传入 `--rotor-zero-rad` 或 `--relative-to-start`，程序会把 `joint_position_deg` 输出为 `nan`，同时保留 `q_rotor_rad` 和 `q_output_raw_deg`。`q_output_raw_deg=q_rotor/6.33` 只是未标定诊断值，不是机械臂绝对关节角。

状态输出字段：

```text
time_s,motor_id,q_rotor_rad,dq_rotor_rad_s,tau_rotor_est_nm,
q_output_raw_deg,joint_position_deg,joint_velocity_rad_s,
joint_tau_ideal_nm,temp_c,merror,mode,exchange_ms
```

其中 `joint_tau_ideal_nm` 仅为 `tau_rotor_est_nm × 6.33` 的理想换算，没有考虑减速器效率、摩擦和结构载荷。

## 六电机正弦位置测试

先做不打开串口的轨迹检查：

```bash
./Qmini_unitree_arm/build/qmini_sine_position --dry-run
```

实机测试命令：

```bash
./Qmini_unitree_arm/build/qmini_sine_position \
  --port /dev/ttyUSB0 \
  --ids 0,1,2,3,4,5 \
  --amplitude-deg 8 \
  --period-s 4 \
  --duration-s 12 \
  --ramp-s 2 \
  --kp-rotor 0.2 \
  --kd-rotor 0.03 \
  --speed-limit-rad-s 0.5 \
  --print-hz 20
```

六台电机分别记录启动转子位置，然后共用同一个相对输出轴目标。RS-485 通信仍按 ID 顺序请求—应答，不是广播。任意一台出现通信错误、`merror`、超温、超速、超力矩估计或超行程时，程序终止轨迹并尝试让全部六台零输出。

这个测试要求六台电机分别固定且空载。当前 `direction`、幅值和轨迹对所有 ID 相同，不能直接用于装配后的机械臂；真实机械臂必须先配置每个关节的 ID、方向、机械零位、软硬限位和不同轨迹。

## 用公共 API 开发

公共代码统一使用 SI 单位：弧度、弧度每秒、牛·米和秒。SDK 的转子侧语义只保留在通信层。例如：

```cpp
#include "qmini_arm/joint_conversion.hpp"
#include "qmini_arm/motor_bus.hpp"

qmini_arm::MotorBus bus("/dev/ttyUSB0");
qmini_arm::MotorState motor = bus.readStateZeroOutput(0);

qmini_arm::JointCalibration calibration;
calibration.motor_id = 0;
calibration.direction = 1;
calibration.gear_ratio = bus.gearRatio();
calibration.rotor_zero_rad = -1529.46704;
calibration.joint_zero_rad = 0.0;
calibration.position_calibrated = true;

qmini_arm::JointState joint =
    qmini_arm::toJointState(motor, calibration);
```

后续 IK、笛卡尔目标和轨迹执行模块只应接触 `JointState`、关节目标和安全约束，不应直接处理 SDK 报文字段。详细分层见 [工程架构](docs/architecture.md)。

## 重要限制

- M8010 的反馈位置是转子侧累计位置，启动值不是 URDF 关节零位；
- 电机转子侧单圈绝对编码不能替代机械臂回零或输出侧绝对编码器；
- Python 运动层只处理自碰撞；尚未处理地面、工装、线缆、负载、重力补偿、外部障碍
  和硬实时调度；
- 可达空间点云是对连续工作空间的有限采样，不是解析边界或安全证明；
- `config/m8010_arm.yaml` 默认未标定，绝对转子位置不会生成；完成六轴方向、机械零位
  和关节限位标定前，规划结果不得发送到真机；
- 当前正弦程序是台架验证工具，不是机械臂控制器；
- 进程、USB 或供电异常时无法保证最后的零输出命令送达，必须提供物理断电和机械限位。
