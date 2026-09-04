# Qmini Unitree Arm Agent Handoff

## 任务主线

当前首要目标是打通下面这条真实机械臂链路：

```text
电脑设置 base_link 坐标系中的末端目标点
  → 读取并标定实机当前六关节姿态
  → IK + 无自碰撞路径规划
  → 有速度/加速度约束的六轴轨迹
  → 转换为 M8010 转子侧命令
  → 通过 /dev/ttyUSB0 串联控制 ID 0..5
  → 实时反馈、跟踪误差和故障处理
```

当前仓库已有初步动力学可视化。MuJoCo 高保真仿真、姿态约束、外部障碍和任务规划可以与
真机闭环准备并行推进，但不能替代六轴标定、回零和硬件安全状态机。

## 接手时先做

1. 运行 `git status --short --branch`，确认并保留协作者已有改动；默认开发分支是 `main`。
2. 阅读 [README.md](README.md)、[docs/architecture.md](docs/architecture.md) 和
   [docs/motion_planning.md](docs/motion_planning.md)。涉及电机协议与安全语义时，再读
   Unitree SDK 原始示例。仓库必须保持自包含，不得引入开发者本机绝对路径作为运行时依赖。
3. 执行本文“回归命令”。Linux 且相邻目录有 Unitree SDK 时，完成标准是 Python 检查、
   Python 测试和 C++ CTest 全部通过；macOS 只运行 Python 部分。
4. 真机动作必须由用户明确要求；离线开发默认不打开串口。

## 已完成

### C++ 电机基础层

- `MotorBus` 封装 `unitree_actuator_sdk`，支持 GO-M8010-6 请求—应答通信。
- 已有转子侧 `MotorCommand`、反馈 `MotorState`、关节侧 `JointState` 和
  `JointCalibration`。
- 已集中实现转子/关节位置、速度、力矩和增益换算。
- 已有基础反馈检查、速度/力矩/温度/相对行程保护。
- `qmini_motor_state` 可读取指定 ID 状态；注意读取本身会发送零输出命令。
- `qmini_sine_position` 可依次控制 ID 0..5 做相对正弦台架测试。
- 现有应用不会修改相邻的 `unitree_actuator_sdk` 仓库。

关键 interface：

- [include/qmini_arm/motor_bus.hpp](include/qmini_arm/motor_bus.hpp)
- [include/qmini_arm/types.hpp](include/qmini_arm/types.hpp)
- [include/qmini_arm/joint_conversion.hpp](include/qmini_arm/joint_conversion.hpp)
- [include/qmini_arm/safety.hpp](include/qmini_arm/safety.hpp)

### Python 离线运动层

- 规划和控制以 `base_link` 为基准，`tool0` 为末端；xacro 额外保留 `world_to_base` 固定变换。
- 从 URDF 读取六轴链、关节轴、质量/质心/惯量/阻尼、effort、硬限位和软限位。
- 已实现 FK 和解析几何雅可比。
- 已实现多起点、自适应阻尼最小二乘位置 IK；当前只约束目标位置，不约束姿态。
- 已实现保守自碰撞检查：box 使用 OBB SAT；cylinder 使用包围 OBB；相邻 link 免检；
  默认碰撞裕量 2 mm，关节边检查分辨率 2°。
- 规划优先逐笛卡尔路点连续求 IK；失败后使用双向 RRT-Connect 在关节空间绕碰撞。
- 已实现五次时间参数化，默认 0.5 rad/s、1.0 rad/s²、50 Hz。
- 已实现无自碰撞工作空间采样和 NPZ 导出。点云是有限采样，不是解析完整边界。
- 已实现 M8010 命令映射和 CSV 导出。
- 已实现低速初步关节动力学：质量矩阵、重力负载、可选科氏项、URDF 阻尼、M8010 PD、
  effort/速度/硬限位和仅仿真的名义重力补偿。

关键 interface：

```python
model = ArmModel(urdf)
collision = CollisionChecker(model)
plan = MotionPlanner(model, collision).plan(start_q, target_position_m)
frames = M8010CommandMapper.from_yaml(model, config).frames(plan.trajectory)
```

主要实现：

- [python/qmini_arm_motion/model.py](python/qmini_arm_motion/model.py)
- [python/qmini_arm_motion/ik.py](python/qmini_arm_motion/ik.py)
- [python/qmini_arm_motion/collision.py](python/qmini_arm_motion/collision.py)
- [python/qmini_arm_motion/planner.py](python/qmini_arm_motion/planner.py)
- [python/qmini_arm_motion/commands.py](python/qmini_arm_motion/commands.py)
- [python/qmini_arm_motion/dynamics.py](python/qmini_arm_motion/dynamics.py)
- [python/qmini_arm_motion/workspace.py](python/qmini_arm_motion/workspace.py)

### 可视化

- Viser 显示 URDF、`tool0`、可拖动目标点、规划轨迹和无自碰撞工作空间。
- 可从当前仿真姿态规划，并选择运动学直接播放或带重力的动力学跟踪。
- 动力学面板显示目标/仿真角、跟踪误差、电机关节力矩和重力负载。
- “六轴 M8010 控制参数”当前全部是计划发送的命令，不是实机反馈。
- 当前界面不会打开 `/dev/ttyUSB0`。
- 未标定时绝对转子位置显示“未标定”；相对启动点转子偏移仍可用于离线检查。

实现见 [python/qmini_arm_motion/visualization.py](python/qmini_arm_motion/visualization.py)。

### 已验证结果

- Python 离线测试：13 个通过。
- C++ CTest：5 个通过。
- Viser 服务已完成启动测试。
- 当前 xacro 的示例目标 `[0.668, 0.105, -0.163] m` 可从 home pose 规划，末端误差约 0.76 mm。
- 20,000 个关节样本中，当前保守碰撞模型保留 18,038 个无自碰撞样本。这个比例不等于
  实际机械臂工作空间占比。

## 尚未完成

当前项目不是可直接运行的真机机械臂控制器，缺少以下闭环：

1. 六轴实机方向、机械零位和有效关节限位尚未标定。
2. 每次上电的回零流程和初始化状态机尚未实现。
3. 尚无一次读取六轴标定关节状态的聚合模块。
4. Python 规划层和 C++ `MotorBus` 之间尚无真机执行 adapter。
5. 可视化没有“实机接管/使能/执行”入口，也没有位置、速度、力矩、温度和错误码反馈。
6. 当前 `kp_rotor=0.2`、`kd_rotor=0.03` 只来自空载台架测试，不能视为装臂后的安全增益。
7. 真机尚未接入重力补偿；初步仿真尚缺负载、地面/外部接触、线缆、减速器详细模型和硬实时调度。
8. 尚未接入 MuJoCo；后续需要从 URDF 生成 MJCF，并补充 M8010 执行器、软限位、传动摩擦、
   转子惯量、控制延迟、接触参数、传感噪声和末端负载。

## 下一阶段实施顺序

### 1. 建立运行配置和有效关节限位

扩展 [config/m8010_arm.yaml](config/m8010_arm.yaml)，为每个关节加入：

```yaml
lower_limit_deg: -80.0
upper_limit_deg: 95.0
home_joint_deg: 0.0
homing_method: limit_or_sensor
homing_direction: -1
homing_speed_rad_s: 0.05
```

增加独立的 `startup_pose_deg`。保持下面三个概念互不混用：

- `joint_zero_rad`：URDF 关节坐标零位；
- `rotor_zero_rad`：与该关节零位对应的转子编码位置；
- `startup_pose`：完成回零后要运动到的安全初始姿态。

为 `ArmModel` 增加 `tighten_limits()` 或 `with_limits()`，有效范围必须是：

```text
URDF 软限位 ∩ 实机标定限位
```

完成标准：IK、工作空间、RRT、轨迹命令和可视化都只读取同一份有效限位；非法交集在启动
时失败；测试覆盖每一轴的收紧、越界拒绝和工作空间变化。

### 2. 实现六轴启动与回零状态机

建议在 C++ 硬件层新增 `ArmController`，由单一线程独占 `MotorBus`：

```text
Disconnected
  → ConnectedReadOnly
  → Homing
  → Calibrated
  → HoldingCurrent
  → MovingToStartupPose
  → Ready
  → Executing / Fault
```

回零结果应生成本次上电的 `SessionCalibration`，不要每次自动覆盖持久 YAML。只有六轴均
完成回零、反馈有效并位于有效限位内，状态才能进入 `Ready`。

回零方法尚未选定。累计转子角不能被当作跨上电绝对零位；使用机械挡块、堵转检测或外部
传感器前，必须先确认真实机构、允许方向、低速/限流参数和超时条件。不要默认通过大力矩
撞机械限位完成回零。

完成标准：使用 fake motor adapter 能覆盖正常回零、单轴超时、通信错误、超温、错误码、
混合标定状态和中途取消；任一失败不会开始轨迹执行。

### 3. 建立 Python 到 C++ 的真机执行 adapter

优先让 C++ 保持控制循环所有权，再通过 pybind11 暴露高层 interface；不要从 Python
每周期分别操作六次串口。建议最小 interface：

```text
read_joint_state()
home()
hold_current()
execute_trajectory(times, q, qd)
cancel()
```

`execute_trajectory()` 内部负责关节到转子换算、ID 0..5 顺序交换、反馈校验、调度超时、
跟踪误差和退出行为。Python 只传完整、已检查的轨迹。

完成标准：先用 fake `MotorBus` 证明每周期恰好产生六条顺序正确的命令；拒绝未标定、
起点不匹配、越界、自碰撞、超速、轨迹时间不单调和通信中断。

### 4. 将可视化接到实机 adapter

界面必须明确分成：

- 电机指令：`q_des`、`dq_des`、转子目标、Kp、Kd、前馈力矩；
- 电机反馈：`q_actual`、`dq_actual`、估计力矩、温度、错误码、通信时间；
- 跟踪误差：目标减实际；
- 状态机：未连接、未标定、已回零、Ready、执行中、Fault。

“连接”“回零”“接管/使能”“执行”应是不同操作。拖动目标只更新规划目标，不直接发送
电机命令。执行前显示整条路径、预计时间和最终关节角，并要求明确确认。

完成标准：仿真 adapter 和真实 adapter 通过同一 interface 驱动界面；关闭实机开关后，
所有拖动和播放均保持纯仿真。

### 5. 低风险真机验证

按以下顺序推进，每一级通过后才进入下一级：

1. 六电机拆离机械臂或安全固定，只读反馈并验证 ID；
2. 单轴确认方向和零位；
3. 单轴小角度闭环命令；
4. 六轴读取与保持当前姿态；
5. 机械臂被可靠支撑、无负载、低速执行短轨迹；
6. 比较实机姿态与可视化反馈；
7. 执行电脑目标点规划。

每次验证都保留物理断电手段和运行日志。

## 处理原则

### 单一事实来源

- `description/qmini_arm.urdf.xacro` 保存几何、惯性、关节轴和理论机械限位；展开后的
  `description/qmini_arm.urdf` 供当前非 ROS 运行层读取。
- YAML 保存装配相关配置、实机安全限位、回零参数和启动姿态。
- `SessionCalibration` 保存本次上电获得的转子零位。
- `ArmModel` 保存三者合成后的有效规划限位。
- 电机换算公式只存在于关节/转子映射模块，不复制到 UI、IK 或应用代码。

### 安全语义

- 内部统一使用 rad、rad/s、N·m、m 和 s；角度仅在 CLI/UI 边缘出现。
- 绝对位置命令要求六轴完成标定；任一轴未标定则整臂禁止执行。
- 实机当前反馈是规划起点；启动时不假定关节为 0°。
- 轨迹执行前重新验证起点、有效限位、自碰撞、速度、时间戳和标定版本。
- `sendZeroOutput()` 表示释放主动保持，不等同于机械臂安全急停；竖直负载下可能坠落。
- 请求—应答串口由一个控制线程独占，UI、IK 和回调通过队列或完整轨迹调用交互。
- 真机动作使用显式使能和确认；测试、导出、可视化默认纯离线。

### 模块 seam

- IK/FK 不依赖 Unitree SDK。
- `MotionPlanner` 接收关节起点和笛卡尔目标，返回完整轨迹。
- 硬件 adapter 接收完整轨迹并返回带时间戳的六轴反馈。
- 可视化只消费规划结果和执行反馈，不拥有串口。
- 复用优先：新增代码前先搜索并复用现有模块、interface、换算函数和测试夹具，保持单一
  事实来源，减少重复实现与冗余逻辑。
- 新功能优先扩展现有深模块；避免在 CLI、GUI 和硬件循环中各写一份限位或换算逻辑。

### 仓库纪律

- 保留用户已有改动，先检查 diff 再编辑。
- 不修改 `unitree_actuator_sdk` 仓库；通过当前 CMake 导入和 adapter 使用它。
- 硬件测试程序必须保留 `--dry-run` 或等价离线模式，并要求明确确认后才打开串口。
- 单元测试使用 fake adapter，不依赖 `/dev/ttyUSB0` 或真实电机。
- 修改运动学、限位、标定或命令换算时，同时增加数值回归测试。

## 已知约束与陷阱

- M8010 SDK 反馈位置是转子侧累计角，不是 URDF 关节绝对角。
- 减速比当前为 6.33；位置、速度、力矩和增益的换算方向不同，复用现有函数。
- 六个电机串联在同一串口上，但协议是逐 ID 请求—应答，不是同时收到六轴反馈。
- [config/m8010_arm.yaml](config/m8010_arm.yaml) 的方向和零位仍是占位值，
  `calibrated: false` 是有意的安全门。
- 当前可视化中的控制参数是命令值，不是反馈值。
- 当前自碰撞检查对圆柱使用保守包围盒，可能拒绝狭窄但真实可行的路径。
- 当前规划目标是位置点；“位置可达”不表示该点的任意末端姿态都可达。
- 当前工作空间需要在实机限位变化后重新采样。

## 回归命令

```bash
# 在 Qarm 仓库根目录执行

python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev,viz]'
.venv/bin/ruff check python tests/python
.venv/bin/pytest -q

cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure

.venv/bin/qmini-motion fk --q-deg 0 0 0 0 0 0
.venv/bin/qmini-motion plan \
  --start-deg 0 0 0 0 0 0 \
  --target 0.668 0.105 -0.163 \
  --output build/m8010_commands.csv
```

可视化启动测试：

```bash
.venv/bin/qmini-motion viz --host 127.0.0.1 --port 8080
```

完成下一里程碑的判断标准：用户在界面设置一个无自碰撞可达目标后，系统从真实标定反馈
建立起点，只在明确使能后执行完整轨迹；界面同步显示六轴指令、反馈和误差；任何标定、
限位、通信或电机错误都阻止启动或使执行进入可解释的 Fault 状态。
