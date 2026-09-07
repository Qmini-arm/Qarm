# Qarm · Viser 统一控制工作树

Qarm 面向 Qmini 四轴 M8010 机械臂，以 `qarm-viser` 作为唯一控制前端，
将标零、重力补偿、位置 IK、计划校验和执行集中在同一个浏览器页面。
运动学和动力学算法保留在现有 Python 包中；C++ 控制核心独立于供应商 SDK。

当前可运行的是离线控制链路。Viser 使用高层后端接口；C++ `ArmController`
提供状态与控制逻辑，但尚未接入 Viser 的 Unix Socket 服务或串口执行适配器。
`--backend hardware` 会明确拒绝启动，不能据此版本的界面操作真机。

## 快速启动

要求 Python 3.10+。离线运行不需要 Unitree SDK，也不访问电机串口。

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e '.[dev]'
./scripts/run-viser --host 127.0.0.1 --port 8080
```

使用 uv 的环境也可执行 `uv sync --extra dev`。
浏览器打开 `http://127.0.0.1:8080`。Viser 固定为 1.1.0，已是基础依赖。
需要从其他设备访问时，显式传入 `--host 0.0.0.0`。

离线体验流程：

1. 连接后端，在标零面板采集候选并提交；首次连接从未标定状态开始。
2. 查看关节状态、模型与标定标识，再启用重力保持或设置 XYZ 目标。
3. 规划并预览轨迹，执行已验证计划；停止和急停通过同一后端处理。
4. 每次重新连接或更换模型后，以页面报告的标定和控制器状态为准。

默认 `fake` 后端用于验证状态转换和理想轨迹执行，不是动力学或硬件安全证明。
界面不承诺任意六维末端姿态：四轴 IK 仅约束 `base_link` 下的 XYZ 位置。

## 结构与职责

```text
浏览器 / Viser
    │ 高层命令、反馈快照、计划预览
python/qarm_viser       唯一操作界面，FK/IK 和规划展示
    │
python/qarm_control     数据结构、标定、后端状态机、协议
    ├── 离线后端        当前可运行路径
    └── 本机 IPC        待连接 C++ 服务与硬件适配器
controller/            SDK 无关的 C++ ArmController 控制核心
include/ + src/        已有重力模型、保护、关节换算和 MotorBus
```

| 路径                         | 用途                                                 |
| ---------------------------- | ---------------------------------------------------- |
| `python/qmini_arm_motion/` | URDF、FK、位置 IK、碰撞、RRT、五次轨迹、离线命令映射 |
| `python/qarm_sim/`         | MuJoCo 模型、渲染与离线诊断                          |
| `protocol/`                | 高层命令与快照协议                                   |
| `config/`                  | 四轴映射、未标定模板与几何参考                       |
| `description/`             | xacro 模型源、URDF 与网格                            |
| `tests/`                   | C++ 与 Python 离线验证                               |
| `apps/`、`tools/`        | 历史硬件实验与维护代码，非新控制入口                 |
| `scripts/`                 | 统一启动与离线检查                                   |

旧 React/HTTP `platform/` 已从当前源码移除，可从 Git 历史恢复。
`qmini-motion viz` 兼容转发至同一个 Viser 应用，不再维护第二套可视化逻辑。
`qmini-motion` 的 FK、工作空间和规划命令，以及 `qarm-sim` 保留作离线诊断。

## 离线检查

```bash
./scripts/check
```

脚本检查 Python 风格与测试，再构建并测试 C++ 核心，显式关闭 SDK 应用。
也可单独执行：

```bash
cmake -S . -B build-core -DQMINI_ARM_BUILD_APPS=OFF
cmake --build build-core -j2
ctest --test-dir build-core --output-on-failure
.venv/bin/qmini-motion fk --q-deg 0 0 0 0
.venv/bin/qarm-sim validate
```

CMake 默认关闭硬件应用。历史实机实验代码的可选编译条件见
[架构说明](docs/architecture.md)，编译成功不表示已完成新的硬件接入。

## 标定与运动边界

仓库配置保持未标定；本工作树不导入其他工作树的现场编码器零位。
几何参考 `config/calibration_pose.json` 也不是有效的编码器标定。
其中 J2 桌面支撑姿态超出正常软限位，采集完成不能直接等同于允许运动。

`q_rotor`、未经标定的输出轴诊断角和 URDF `q_joint` 是不同量。
未标定反馈的 `q_joint` 必须为空，不能传入 FK、IK 或轨迹执行。
计划需绑定模型和标定标识；更换模型、标定或控制器启动周期后需重新验证。

未来真机路径由 C++ 独占串口与周期控制，负责反馈检查、力矩限幅、渐变和看门狗。
Python/Viser 仅提交高层意图。M8010 BRAKE 不等于机械抱闸；实机标零需要可靠支撑。

## 文档

- [当前架构与实现边界](docs/architecture.md)
- [运动学、规划与仿真算法](docs/motion_planning.md)
- [交接与剩余工作](HANDOFF.md)
- [重构前运行记录](docs/legacy_operations.md)（历史参考，非当前操作指南）
