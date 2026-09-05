# Qmini Unitree Arm

`Qarm` 是面向 Qmini 六轴机械臂后续开发的混合工程。C++14 层完成了
GO-M8010-6 通信、转子/关节坐标换算和台架测试；Python 层完成了基于 URDF 的 FK、
无自碰撞工作空间采样、位置 IK、轨迹规划、M8010 命令生成和带重力的初步关节动力学
可视化。
工程只读引用 `unitree_actuator_sdk`，不会修改 SDK 仓库；CMake 会优先使用仓库内的
本地 SDK 工作副本，否则回退到与 `Qarm/` 相邻的 SDK 目录。

当前可执行程序：

- `qmini_motor_state`：读取指定串口、指定 ID 的电机状态；
- `qmini_sine_position`：让 ID 0–5 执行相同的相对正弦位置测试。
- `qmini_gravity_comp`：带保护的 100% 静态重力前馈实验；
- `qmini_return_to_zero`：执行 MuJoCo 验证过的 CSV 回到桌面支撑标定位；
- `qmini-motion`：离线 FK/IK、可达空间、轨迹与可视化入口（不打开串口）。

## 目录结构

```text
Qarm/
├── CMakeLists.txt
├── cmake/
│   └── UnitreeActuatorSDK.cmake  # SDK 路径、架构和共享库导入
├── include/qmini_arm/
│   ├── types.hpp                 # MotorCommand/MotorState/JointState
│   ├── motor_bus.hpp             # SDK 无关的公开通信接口
│   ├── joint_conversion.hpp      # 转子侧与机械关节侧换算
│   ├── safety.hpp                # 通用反馈和运动保护
│   ├── sine_trajectory.hpp       # SI 单位的轨迹模块
│   └── joint_trajectory.hpp      # 回零 CSV 解析和安全约束
├── src/                          # 公共库实现，SDK 细节只在这里出现
├── apps/
│   ├── read_motor_state.cpp      # 状态读取工具
│   ├── sine_position_test.cpp    # 六电机正弦位置测试
│   ├── gravity_compensation.cpp  # 重力补偿控制器
│   ├── return_to_zero.cpp         # CSV 驱动的回零控制器
│   └── cli_utils.hpp             # 两个工具共用的参数解析
├── tests/                        # C++ 和 Python 离线测试
├── docs/                         # 架构与运动规划说明
├── HANDOFF.md                    # 当前目标、进度、下一阶段和安全边界
├── config/m8010_arm.yaml         # 六关节 ID、方向、零位和控制参数
├── description/                  # xacro/URDF 机械臂模型与可视网格
├── python/qmini_arm_motion/      # FK/IK/碰撞/规划/命令/动力学/可视化
├── python/qarm_sim/              # MuJoCo、遥测镜像和离线回零实验
└── pyproject.toml
```

应用只依赖 `qmini_arm_core` 的公开头文件，不直接使用 `MotorCmd`、`MotorData` 或 `SerialPort`。未来替换通信后端、增加仿真后端或 ROS 2 适配时，不需要改动 IK 和轨迹层。

## 平台支持

| 功能                     | Linux x86_64/aarch64 |          macOS |
| ------------------------ | -------------------: | -------------: |
| Python FK/IK、规划、测试 |                 支持 |           支持 |
| Viser 可视化和初步动力学 |                 支持 |           支持 |
| C++ 电机层编译           |                 支持 | 不支持现有 SDK |
| M8010 串口实机工具       |                 支持 | 不支持现有 SDK |

macOS 的限制来自 Unitree 官方仓库当前只提供 Linux x86_64 和 aarch64 的预编译 `.so`；
与 Python 离线运动层无关。真机控制建议使用 Ubuntu 22.04/24.04 或等价 Linux 环境。

## 从零安装

项目要求 Python 3.10 或更高版本。Python 离线层不需要 Unitree SDK，也不会打开串口。

### Linux：Python 离线层

Ubuntu/Debian 安装基础工具并克隆仓库：

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-pip

git clone https://github.com/Qmini-arm/Qarm.git
cd Qarm
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[dev,viz]'
```

验证安装：

```bash
.venv/bin/ruff check python tests/python
.venv/bin/pytest -q
.venv/bin/qmini-motion fk --q-deg 0 0 0 0 0 0
```

### macOS：Python 离线层

先安装 Xcode Command Line Tools，并通过 [Homebrew](https://brew.sh/) 安装 Git 和 Python：

```bash
xcode-select --install
brew install git python

git clone https://github.com/Qmini-arm/Qarm.git
cd Qarm
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e '.[dev,viz]'
.venv/bin/ruff check python tests/python
.venv/bin/pytest -q
```

如果系统已安装满足版本要求的 Git 和 Python，可以跳过 Homebrew 对应步骤。

### Linux：C++ 电机层和真机工具

完成上述 Linux Python 安装后，额外安装编译工具，并把 Unitree SDK 放在仓库内的本地
工作副本或与本仓库同一父目录：

```bash
sudo apt install -y build-essential cmake

cd ..
git clone https://github.com/unitreerobotics/unitree_actuator_sdk.git
cd Qarm
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

默认先使用仓库内的 `unitree_actuator_sdk/` 工作副本；没有该目录时回退到与 `Qarm/`
相邻的 `unitree_actuator_sdk/`。若 SDK 位于其他位置：

```bash
cmake -S . -B build \
  -DUNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk
```

Linux 下访问 USB 串口通常还需要把当前用户加入 `dialout` 组，执行后注销并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

## FK、IK、无自碰撞规划与可视化

Python 运动层采用“URDF 模型—阻尼最小二乘 IK—工作空间采样—Viser”流程，并针对
当前 M8010 URDF 增加圆柱碰撞体、连续路径碰撞检查、RRT-Connect 兜底规划和转子侧
控制参数输出。

`description/qmini_arm.urdf.xacro` 是模型源；修改它后生成非 ROS 运行层使用的 URDF：

```bash
.venv/bin/xacro description/qmini_arm.urdf.xacro -o description/qmini_arm.urdf
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
  --target 0.668 0.105 -0.163 \
  --output build/m8010_commands.csv
```

启动可视化：

```bash
.venv/bin/qmini-motion viz --host 127.0.0.1 --port 8080
```

浏览器中可拖动目标点、规划并播放轨迹、显示无自碰撞可达空间，并实时查看 ID 0–5
的关节目标、仿真角、跟踪误差、电机关节力矩、重力负载和转子侧命令。动力学使用 xacro
中的质量/质心/惯量/阻尼/力矩与速度限制，重力在 `world` 中为 `-Z`。可视化和 CSV 导出均
不会打开 `/dev/ttyUSB0`。

完整的算法边界、控制参数语义和真机接入条件见
[运动规划说明](docs/motion_planning.md)。

## 读取电机状态

读取 `/dev/ttyUSB0` 上 ID 0 的一个状态样本：

```bash
./build/qmini_motor_state \
  --port /dev/ttyUSB0 \
  --id 0
```

程序要求输入 `READ` 才会打开串口。M8010 是请求—应答设备，所谓“读取”并不是被动监听：程序发送 `BRAKE(mode 0)` 且 `tau=dq=q=kp=kd=0` 后取得反馈。这会改变电机状态，也不是安全机械抱闸；机械臂必须有可靠支撑。

持续读取，并把启动时位置临时定义为关节 0°：

```bash
./build/qmini_motor_state \
  --port /dev/ttyUSB0 \
  --id 0 \
  --samples 0 \
  --rate-hz 10 \
  --relative-to-start
```

`--samples 0` 表示持续运行到 Ctrl+C。临时零位只对本次进程有效。

如果已经通过可靠的机械找零获得转子零位，可以显式换算关节角：

```bash
./build/qmini_motor_state \
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
./build/qmini_sine_position --dry-run
```

实机测试命令：

```bash
./build/qmini_sine_position \
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
qmini_arm::MotorState motor = bus.readStateBrake(0);

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
- Python 规划层只处理自碰撞和关节限位；可视化已有初步重力/刚体动力学，但尚未处理
  地面接触、工装、线缆、末端负载、减速器效率和硬实时调度；
- 可达空间点云是对连续工作空间的有限采样，不是解析边界或安全证明；
- `config/m8010_arm.yaml` 默认未标定，绝对转子位置不会生成；完成六轴方向、机械零位
  和关节限位标定前，规划结果不得发送到真机；
- 当前正弦程序是台架验证工具，不是机械臂控制器；
- 进程、USB 或供电异常时无法保证最后的零输出命令送达，必须提供物理断电和机械限位。

## MuJoCo 与六轴只读镜像

`qarm-sim` 保持 `description/qmini_arm.urdf.xacro` 为模型源，运行时展开并
生成 MuJoCo 场景，保留 visual STL 和原生 cylinder/box collision，同时增加
六个关节力矩执行器、`tool0` site、状态传感器和固定基座场景。执行器的峰值
边界来自宇树官方 GO-M8010-6 参数；转子惯量、连续力矩、摩擦、齿隙和通信延迟
仍明确保留为待辨识参数。

安装并验证：

```bash
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv sync --extra dev
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim validate
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim render
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run pytest -q
```

开发板读取器安装在：

```text
/home/HwHiAiUser/.local/libexec/qarm/m8010_readonly
```

它只允许顺序发送 `BRAKE+全零` 请求，不实现 FOC 运动命令。运行仍要求机械支撑：

```bash
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim inspect-stream \
  --ssh-target HwHiAiUser@192.168.10.102 \
  --samples 5 \
  --acknowledge-supported-arm
```

### 在桌面支撑姿态标零

完整 URDF 零位很难靠人稳定保持，因此采用照片确认过的桌面支撑姿态。该姿态
不是运行姿态；`joint_2` 在手动标零时超过软运行限位，但仍位于专门保留的
`±1.75 rad` 硬限位内。程序直接使用
当前 `base_pair.stl`、`arm_link.stl` 和 `motor.stl` 顶点重新解算；当前结果为：

```text
motor ID / joint:       0       1          2         3     4       5
reference angle deg:   0.0   +100.1540   +8.8698   -1.1489   0.0  -89.9544
```

- 第一根长臂 STL 与底板 STL 定义的桌面相切；
- 远端 `motor.stl` 与同一桌面相切；
- `joint_2/motor ID 1` 从照片操作侧逆时针约 `100.1540°`；该正支使
  motor 4 朝上的水平解只需 `joint_4/motor ID 3≈-1.1489°`，不会使
  joint_4 越过正常硬限位；
- motor 5 从照片中的操作侧看，顺时针转到机械限位，对应 `joint_6` 的
  URDF 下限 `-1.57 rad`。

可随时复算并检查 STL 误差：

```bash
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim solve-calibration-pose
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim viewer --calibration-pose
```

1. 可靠支撑机械臂，手动摆到上述姿态；不要用电机命令把它驱动到这个超软限位姿态。
2. 清空工作区，确保没有第二个进程占用 `/dev/ttyUSB0`。
3. 执行：

```bash
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim capture-zero \
  --ssh-target HwHiAiUser@192.168.10.102 \
  --samples 200 \
  --confirm-table-supported-pose \
  --acknowledge-supported-arm
```

程序连续采集约 2 秒；任何电机无响应、报错，或任一关节位置跨度超过
`0.01 rad` 都会拒绝写入。成功时会先备份 `config/joint_map.json`，再原子写入
参考关节角、六轴零偏、转子零位、采样稳定性、UTC 时间和开发板 boot ID。
标零绑定当前上电
周期，开发板或电机重新上电后必须重新采集。

方向尚未确认时，程序以参考姿态形式保存原始编码器值，并按
`q=q_ref+direction·(encoder-encoder_ref)` 映射；之后修正 `direction` 不需要
重新摆标定姿态。派生的 `zero_offset_rad` 仅用于兼容和诊断。

本次标零后已逐轴转动并确认实机与 MuJoCo 方向一致，`joint_map.json` 已记录
`direction_calibrated=true`。它仍不会自动把 `m8010_arm.yaml` 中的运动规划命令
标记为可下发，避免重力补偿部署意外扩大成轨迹控制授权。

标零后启动实时 MuJoCo 镜像：

```bash
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim mirror \
  --ssh-target HwHiAiUser@192.168.10.102 \
  --acknowledge-supported-arm
```

macOS 会自动使用 `mjpython` 打开窗口。默认不无限写日志；需要实验记录时显式
添加 `--record runs/name.ndjson`。镜像使用 `mj_forward` 显示实测姿态，不进行
动力学积分，因此它首先验证 ID、方向、零位和几何 FK，而不是直接证明动态
sim-to-real 精度。

## 开发板重力补偿

开发板已安装用户态命令：

```text
~/.local/bin/qmini-gravity
```

运行配置位于 `~/.config/qarm/gravity_comp.conf`，并绑定本次标零的开发板
boot ID、六轴转子参考位置、ID 顺序和方向。程序直接使用与 MuJoCo
`qfrc_bias(q, qvel=0)` 同符号的静态保持力矩，并按功率守恒换算到转子侧：

```text
tau_rotor = scale * direction * tau_joint_gravity / 6.33
```

照片中的末端机械手按用户确认作为可忽略轻负载，当前模型不计其质量；以后更换
较重工具或拿取物体前必须补入工具质量和质心。

部署默认不自启，并提供三个互斥模式：

```bash
# 不开串口
qmini-gravity --dry-run

# 只发 BRAKE，计算但绝不发送重力力矩
qmini-gravity --shadow \
  --acknowledge-supported-arm \
  --confirm-same-motor-power-cycle

# 100% 模型补偿；仍受逐轴力矩帽、速度保护和渐入/渐出约束
qmini-gravity --enable-foc \
  --scale 1.0 \
  --duration-s 12 \
  --ramp-s 3 \
  --acknowledge-supported-arm \
  --acknowledge-estop-ready \
  --confirm-same-motor-power-cycle
```

FOC 模式启动前要求五轮完整反馈、当前 boot ID、所有关节在软限位内且离边界至少
`0.05 rad`。当前最大模型 scale 为 100%，逐轴转子力矩限幅为
`[0.03, 2.00, 0.90, 0.08, 0.03, 0.03] N·m`；离线扫描的软限位内最大模型请求约为
`[0.0016, 1.9967, 0.8773, 0.0716, 0.0014, 0.0005] N·m`，因此不会把正常的 100%
模型请求截成较低比例，日志中的 `saturated=1` 仍会显示 slew 或异常限幅。逐轴关节
速度软保护为 `[0.80, 0.80, 0.80, 1.20, 2.00, 2.50] rad/s`，连续三帧才退出；
硬保护为 `[1.50, 1.50, 1.50, 2.40, 4.00, 5.00] rad/s`，单帧立即退出。超速故障
会打印实测值、阈值和保护类型。控制器还包含 100 Hz 循环、阻尼、力矩 slew、
温度/错误码/反馈/50 ms 调度看门狗。正常 12 秒实验为 3 秒渐入、6 秒观察、
3 秒渐出，再确认零力矩 FOC 并切回 BRAKE；故障路径立即尝试 BRAKE。SSH 断开
产生的 `SIGHUP` 也会走停止流程，并忽略日志管道断开产生的 `SIGPIPE`。

通过 SSH 做首次实验时，建议先创建 `~/.local/state/qarm`，并把控制器 stdout/stderr
直接重定向到开发板本地文件，而不是接到 `tee` 等管道；这样远端输出背压不会阻塞
控制线程。实验结束后再读取该日志。

开发板或任一电机掉电后必须重新标零并更新部署配置。boot ID 只能发现开发板重启，
不能自动发现单台电机掉电，因此 `--confirm-same-motor-power-cycle` 是人工安全门。
`SIGKILL`、USB 断开或整板故障时软件无法保证最后一帧 BRAKE 到达，实验期间必须
始终保留机械支撑和物理断电手段。

## 回到桌面支撑标定位

下电前的“回零”目标是桌面支撑标定姿态，而不是数学上的
`q=[0,0,0,0,0,0]`。它包含 `joint_2=100.154°` 和 `joint_6=-90°`，位于正常
运行软限位之外，但位于 URDF 硬限位内；只有机械臂沿轨迹回到桌面后才允许安全下电。
回零分成离线规划/验证和实机执行两步。先在本机用同一份 URDF、碰撞检查器和 MuJoCo
生成轨迹；`plan-home` 的 `--start-deg` 必须填写当前六个关节角：

```bash
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim plan-home \
  --start-deg 10 5 10 5 -5 5 \
  --output build/calibration_home.csv
```

数学 URDF 零位仍可单独规划，但不会用于下电：

```bash
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qarm-sim plan-urdf-zero \
  --start-deg 10 5 10 5 -5 5 \
  --output build/urdf_zero.csv
```

该命令只做离线计算，不访问 SSH/串口。规划器先检查起点、零位和完整路径的 URDF
自碰撞，必要时使用 RRT-Connect 绕开碰撞；随后用五次曲线限制到 `0.25 rad/s`、
`0.50 rad/s²` 和 `10 ms` 控制周期。命令会再用 MuJoCo 闭环实验复现 M8010 的
位置/速度控制、100% 重力前馈、Q8 力矩量化、已部署力矩帽和假设的
`0.001 kg·m²` 反射关节惯量；除预期的桌面接触外，只有无自碰撞、无硬限位越界、无
力矩饱和、速度和跟踪误差均通过，且终点检测到桌面接触时才写出 CSV。这个惯量仍未
由实机辨识，MuJoCo 结果不能替代现场慢速验证。

开发板上的受保护执行器为 `~/.local/bin/qmini-return-home`。它只接受上述 13 列关节
轨迹 CSV，不接受任意电机目标；启动前会重新读取六轴 BRAKE 反馈，确认当前关节角
与轨迹首帧相差不超过 `0.03 rad`，检查标定 boot ID、ID 0--5、速度/温度/反馈和
活动限位（回标定位阶段允许进入已声明的硬限位区），然后按轨迹发送带绝对转子位置、速度、位置增益、阻尼和 100% 重力前馈的
FOC 帧。故障或中断先尝试六轴 BRAKE，正常结束也先 BRAKE 再打印日志。
手动拖动的放宽阈值只作用于重力补偿；回标定位执行器仍使用更严格的逐轴速度保护，
其规划速度上限保持 `0.25 rad/s`。
BRAKE→FOC 的前三帧单独处理 SDK 模式切换速度瞬态：首帧位置目标设为刚读取的实测
姿态并立即启用完整 `kp/kd`，随后用 1 秒平滑对齐 CSV 起点；同时用
`0.01 rad/cycle` 的位置步长保护确认机械臂没有真实快速运动，三帧后恢复严格的自动
回位速度阈值。

实机操作示例（人在机械臂旁、机械支撑和物理断电就绪后）：

```bash
# 开发板本地执行；日志写本地文件，避免 SSH 输出背压阻塞控制线程
mkdir -p ~/.local/state/qarm
~/.local/bin/qmini-return-home \
  --trajectory /path/to/calibration_home.csv \
  --enable-foc \
  --acknowledge-supported-arm \
  --acknowledge-estop-ready \
  --confirm-same-motor-power-cycle \
  --confirm-collision-checked-plan \
  >~/.local/state/qarm/return-home-$(date +%Y%m%dT%H%M%S).log 2>&1
```

`--dry-run` 可在没有串口时验证 CSV 和配置；它不会读取开发板。这个流程是“下电前返回
支撑姿态”，不是跨上电自动寻找机械零点；实机回标定位不能从
桌面支撑标定姿态以外的超限姿态启动；如果现场反馈角度不匹配、刚重启过开发板/电机、
CSV 不是当前 URDF 生成的文件，程序会拒绝发出第一帧 FOC。到达终点后先确认电机
反馈仍在标定姿态，再切 BRAKE，最后由现场人员执行物理下电。
