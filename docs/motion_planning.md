# Qmini FK、IK、无自碰撞规划与 M8010 命令说明

## 当前能力

运动层固定 `base_link` 为单位变换，以 `tool0` 为末端。一个完整调用隐藏在
`MotionPlanner.plan(start_q, target_position)` 这个 interface 后面：

```text
当前四关节角 + base_link 中的目标点
        │
        ├─ 笛卡尔直线分段，每个路点以前一解为种子做位置 IK
        │      └─ 每个关节段执行连续离散自碰撞检查
        │
        └─ 直线路径失败时：目标点多起点 IK → RRT-Connect → 捷径平滑
                    │
└─ 五次曲线时间参数化（速度、加速度、周期限制）
                                      │
                                      ▼
                         M8010 四轴逐周期控制参数
```

`MotionPlanner.plan_home(start_q)` 是回到数学 URDF 零位的关节空间接口；它和
`plan_to_configuration(start_q, goal_q)` 共用起点/终点软限位、自碰撞检查、RRT-Connect
兜底和五次曲线时间参数化。`MotionPlanner.plan_calibration_pose(start_q, calibration_q)`
用于桌面支撑标定位；`qarm-sim plan-home` 使用该目标，`qarm-sim plan-urdf-zero`
才使用数学零位。两者都将轨迹导出为关节 CSV，并用 MuJoCo 做离线闭环实验；CSV 不是
电机协议帧，也不会打开串口。实机执行必须经过
`qmini-return-home` 的 C++ 安全门，重新核对当前反馈、标定 boot ID、速度、温度、
通信和物理急停。

这样区分 IK 和路径规划很重要：IK 只回答“这个末端点对应哪个关节姿态”，不能证明从
当前位置到该姿态的中间过程不碰撞。路径规划器负责验证完整关节轨迹。

路径规划仍只约束自碰撞和 URDF 软限位。浏览器可视化另外加入了基于 URDF 质量、质心、
惯量和关节阻尼的初步动力学被控对象；它不将动力学约束反向加入 IK/RRT，也没有地面、
外部障碍接触、末端负载和线缆模型。

## 当前 URDF 版本

`description/qmini_arm.urdf.xacro` 是人工维护的模型源，`description/qmini_arm.urdf`
是供当前非 ROS 运行层读取的展开产物。模型保持
`base_link → joint_1 ... joint_4 → tool0` 的四轴串联链，
并增加 `world → base_link` 的固定坐标变换。xacro 的四关节零位就是机械臂
`home_pose`。

home pose 下，`base_link` 中的 `tool0` 位置为
`[0.742450, -0.004895, -0.006285] m`；建模总质量约为 `2.5306 kg`。配置的重力在
`world` 中为 `[0, 0, -9.80665] m/s²`，经 xacro 的 `world_to_base` 旋转后，在
`base_link` 中约为 `[-9.806647, -0.005266, -0.005767] m/s²`。
`link_6` 是沿用的工具支架名称，不表示仍有第六个运动关节。

因关节坐标姿态已变化，旧模型对应的真机 `direction` 和零位不得直接复用；当前
`config/m8010_arm.yaml` 继续保持 `calibrated: false`，等新机械结构逐轴确认方向和回零后
再填入。

## 模块结构

```text
python/qmini_arm_motion/
├── model.py          URDF 解析、base_link 固定、FK、解析雅可比
├── collision.py      box 精确 OBB SAT、cylinder 保守 OBB、路径碰撞检查
├── ik.py             带关节限位、多起点和自碰撞过滤的位置 DLS IK
├── planner.py        连续路点 IK、RRT-Connect、捷径和平滑时间轨迹
├── workspace.py      无自碰撞关节采样及 FK 可达点云
├── commands.py       关节轨迹到 M8010 转子侧参数的统一映射
├── dynamics.py       URDF 惯性、重力、阻尼、M8010 PD 和时间积分
├── visualization.py  Viser 目标拖动、轨迹/动力学播放、工作空间和命令面板
└── cli.py            qmini-motion 命令行入口
```

现有 C++ `MotorBus` 仍是唯一直接依赖 `unitree_actuator_sdk` 的硬件模块。Python
规划层不碰串口，只产生与 `qmini_arm::MotorCommand` 同语义的数据；未来实机 adapter
应消费这些轨迹帧并调用 `MotorBus::exchange()`，无需让 IK 依赖 SDK。

## FK 和可达空间

FK 逐级应用 URDF 关节原点和关节轴旋转：

```text
T_base_tool(q) = T_joint1(q1) · ... · T_joint4(q4) · T_tool0
```

规划限位取 URDF `<limit>` 与 `<safety_controller>` 的交集。当前四轴软限位绝对值依次为
`[170°, 89.95°, 115°, 114.59°]`。对应硬限位
绝对值为 `[180°, 100.27°, 150.11°, 120°]`，所以每个软限位都严格位于
硬限位内。动力学播放对命令继续使用软限位，并使用 `<limit effort>` 做关节力矩限幅。

四自由度连续关节空间映射到连续三维工作空间，因此不能列出“所有点”。
`workspace` 命令采用可重复的均匀蒙特卡洛采样：每个关节姿态先检查自碰撞，通过后再
保存 FK 的 `tool0` 位置。样本数越大，点云越接近完整可达区域，但点云本身不是精确
边界。某个目标是否可用，最终仍由带误差阈值的 IK 和完整路径碰撞检查决定。

NPZ 输出包含：

- `configurations_rad`：每个无自碰撞样本的四关节角；
- `positions_m`：对应的 `tool0` 位置，坐标系为 `base_link`；
- `requested_samples`：原始采样数。

## IK 和路径

IK 只约束末端位置，4 个关节都会参与求解；四轴机构不能独立控制任意六维末端位姿。求解器使用解析
位置雅可比、自适应阻尼最小二乘、关节居中零空间项和多起点重启。成功条件为：

1. 末端位置误差不大于 1 mm；
2. 四轴处于 URDF 软限位内；
3. 最终关节姿态无自碰撞。

规划器优先在末端直线上生成间隔不大于 25 mm 的路点，每一点都重新做 IK，并以前一
点的关节解作为种子，从而尽量维持同一解支。如果任一路点不可解或关节段碰撞，则改用
双向 RRT-Connect 在四维关节空间绕开自碰撞。后者保证到达目标点，但末端在中间不再
保证走直线。

每段路径使用五次时间曲线，段首段尾速度和加速度为零。默认参数来自
`config/m8010_arm.yaml`：

- 关节速度上限：0.5 rad/s；
- 关节加速度上限：1.0 rad/s²；
- 控制周期：0.02 s（50 Hz）。

## 初步动力学仿真

`ArmDynamics` 从同一份 URDF 读取质量、质心、惯量张量和关节阻尼，由质心线速度
雅可比和角速度雅可比构造关节空间质量矩阵，同时计算重力负载。
`MotorDynamicsSimulator` 将转子侧 M8010 Kp/Kd 按减速比折算到关节侧，加入 URDF
关节力矩/速度/硬限位，并用半隐式时间积分产生仿真关节位置和速度。为避免小惯量手腕与
较大阻尼增益形成数值刚性，电机 Kd 和 URDF 粘性阻尼采用隐式步进。

界面默认启用动力学和“仅仿真”的名义重力补偿。关闭补偿后可直接观察当前 Kp/Kd 在
自重下的偏移。仿真重力补偿不会改写 `M8010CommandMapper` 的 `tau_ff`，也不会写入 CSV
或打开串口。

这是低速初步模型：默认关闭科氏/离心项以保持交互实时性，也尚未包含电机转子惯量、减速器
效率/回程间隙、地面与碰撞接触力、末端负载和线缆力。它可用于检查重力矩量级和控制跟踪趋势，
不是真机力矩安全证明。

## 自碰撞语义

URDF 中的 box 使用 15 分离轴 OBB 检测。M8010 外壳是 cylinder；当前将圆柱替换成
完全包住它的 OBB，因此检查是保守的，可能拒绝非常狭窄但真实可行的间隙，不会因为把
电机外壳缩小而放过碰撞。默认额外加入 2 mm 安全裕量。

直接父子 link 在装配面必然接触，因此不互检；其他 link 对全部启用。每条关节边以每轴
不超过 2° 的间隔离散检查。它不能替代更高分辨率的 CAD 碰撞模型和真机慢速验证。

## M8010 控制参数

`M8010CommandMapper` 对每个控制周期输出 ID 0–3 的：

- 关节目标 `q_joint` 和 `dq_joint`；
- 绝对转子目标 `q_rotor`（仅完成标定后存在）；
- 相对轨迹起点的转子偏移；
- 转子目标速度 `dq_rotor`；
- 转子侧 `kp`、`kd` 和 `tau_ff`。

换算与 C++ `joint_conversion.cpp` 一致：

```text
q_rotor  = rotor_zero + direction · ratio · (q_joint - joint_zero)
dq_rotor = direction · ratio · dq_joint
ratio    = 6.33
```

默认 `kp_rotor=0.2`、`kd_rotor=0.03`，沿用此前 M8010 台架测试配置。真机命令映射仍保持
`tau_ff=0`；界面的名义重力补偿是独立的仿真选项。这些增益只证明过空载电机缓慢测试，
不代表装臂后的最终控制器参数。

`config/m8010_arm.yaml` 中四轴默认 `calibrated: false`。此时界面和 CSV 会显示相对
转子偏移，但绝对 `q_rotor` 留空。原因是 URDF 角度必须经过每关节方向和机械零位映射
才能成为真机绝对命令；用占位零位下发可能造成机械臂突然运动。

## 真机接入前的必要条件

1. 标定每个 `joint_n` 对应的电机 ID、旋转方向、`rotor_zero_rad` 和
   `joint_zero_rad`，并把配置中的 `calibrated` 改为 `true`；
2. 在实机当前反馈上验证 FK 数字孪生姿态与真实姿态一致；
3. 让硬件 adapter 从当前四轴标定反馈构造 `start_q`，禁止假定每次都从 URDF 零位
   开始；
4. adapter 逐控制周期把 `rotor_position_rad`、`rotor_velocity_rad_s`、`kp_rotor`、
   `kd_rotor`、`torque_ff_nm` 填入现有 `MotorCommand`，串联总线按 ID 依次交换；
5. 继续使用温度、速度、力矩、通信超时和物理断电保护；先降速、无负载、逐轴验证，
   再做四轴联动。

当前 CSV 是离线接口和审计记录，不是可以直接执行的真机脚本。默认未标定配置下，程序
会明确告警且不产生绝对转子目标。

## 可视化操作

运行：

```bash
.venv/bin/qmini-motion viz --host 127.0.0.1 --port 8080
```

在浏览器中：

1. 拖动 `target` 三轴手柄，或输入 `base_link` 下的 x/y/z；
2. 点击“规划到目标”，查看求解方式、误差、路点和绿色末端路径；
3. 选择是否启用动力学和仅仿真的名义重力补偿；
4. 点击“播放规划”，观察目标/仿真角、跟踪误差、电机关节力矩和重力负载；
5. “四轴 M8010 控制参数”表同步显示每台电机当下的计划命令；
6. 勾选“显示无自碰撞可达空间”后在后台生成 FK 点云。

关闭浏览器或 Ctrl+C 都不会向串口发送任何命令。
