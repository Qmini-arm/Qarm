# Qarm Viser 重构交接

本工作树将操作入口统一为 `qarm-viser`。原工作树的现场标定、测量和未提交修改不属于
本次迁移；当前仓库配置继续保持未标定。

## 当前结构

- `python/qarm_viser/`：唯一浏览器控制界面，包含标零、重力补偿、XYZ 位置 IK、
  轨迹预览、计划执行和停止操作。
- `python/qarm_control/`：高层后端与领域数据，处理标定身份、计划身份、状态和租约。
- `controller/`：C++ `ArmController` 核心与可选的硬件总线适配器。
- `python/qmini_arm_motion/`：保留已验证的 FK、IK、碰撞、RRT、轨迹和动力学算法。
- `python/qarm_sim/`：保留 MuJoCo 离线验证与诊断命令。
- `protocol/`：高层命令与反馈的数据协议。

旧 React/HTTP `platform/` 已从当前源码删除，历史内容可从 Git 恢复。
旧 README 保存在 [历史运行记录](docs/legacy_operations.md)；其中的硬件命令和
独立平台说明不再代表当前操作流程。`qmini-motion viz` 转发到新应用。
历史硬件维护应用默认不构建、不安装，不是 Viser 之外的生产控制入口。

## 已实现与未接通的边界

当前 Viser 可通过 fake 后端完成离线工作流；fake 的理想执行不等同于动力学测试。
C++ 核心和硬件总线适配器已提供，但尚无连接到 Viser 的 daemon、Unix Socket
服务或 Python 绑定。Viser 的 `hardware` 选项因此明确拒绝启动。
Python fake 与 C++ 核心是两份实现，目前并非 Python 调用 C++ 状态机。

本次重构未开启串口、未下发电机命令，也未进行真机标零、重力补偿或轨迹执行。
离线测试不能代替总线故障注入、看门狗验证、机械支撑和实机调试。

## 模型与标定语义

模型源为 `description/qmini_arm.urdf.xacro`，活动关节为 `joint_1..joint_4`，
默认电机 ID 为 0..3。`link_6` 是工具支架名。位置 IK 只约束 XYZ。
旧六轴标定、旧六轴轨迹以及机构变化前的第四轴零位不能复用。

- `config/joint_map.json` 的零位、方向和综合标定为 false。
- `config/m8010_arm.yaml` 的各轴 `calibrated` 为 false。
- `config/gravity_comp.conf` 保持未确认标定和 `UNCALIBRATED` boot ID。
- `config/calibration_pose.json` 是未现场确认的几何参考，不是编码器标定。
- 桌面参考 `[0, 1.7480178111, 0.1548064707, 0]` rad 的 J2 超出正常软限位。
  标零采集、标定有效和允许正常运动必须分开判断。

未标定 `q_joint` 为空，只能显示原始转子量用于诊断。计划和标定绑定当前模型，
标定还需绑定控制器启动周期；换模型、重连或更换标定后必须按后端状态重新验证。
M8010 BRAKE 不等于机械抱闸。

## 开发验证

```bash
uv sync --extra dev
./scripts/check
./scripts/run-viser --host 127.0.0.1 --port 8080
.venv/bin/qmini-motion fk --q-deg 0 0 0 0
.venv/bin/qarm-sim validate
```

Viser 固定 1.1.0；`trimesh==4.11.5` 保留 arm64 可安装依赖组合。
默认 CMake 构建无需 SDK。硬件构建选项和实现限制见
[架构说明](docs/architecture.md)。

## 接续工作

1. 实现本机 C++ daemon 与 Unix Socket 编解码，明确端到端命令和状态契约。
2. 让 Viser 后端通过该服务调用 C++ 控制器，补齐跨语言契约与断连测试。
3. 在控制器中持久化并审核真机标定，绑定模型、boot ID、ID、方向和采样统计。
4. 先验证硬件反馈、超时和故障处理，再在支撑与低力矩条件下开展重力补偿测试。
5. 接入完整轨迹执行后，验证起点偏差、软限位、碰撞证明和跟踪保护。
