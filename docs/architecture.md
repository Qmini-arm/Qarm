# Viser 统一控制架构

`qarm-viser` 是唯一操作界面。浏览器提交标零、模式切换和计划执行等高层意图；
界面代码不打开串口，不逐周期生成电机命令。

## 当前可运行链路

```text
浏览器
  ↕ Viser 场景、控件与事件
qarm_viser.app
  ├── qmini_arm_motion：FK / 位置 IK / 碰撞 / RRT / 时间轨迹
  └── qarm_control 后端：标定 / 状态 / 计划 / 停止 / 租约
        └── fake：离线理想轨迹执行
```

Viser 和 Python fake 运行在同一个进程。`controller/` 中的 C++ 控制器是独立库，
目前没有 daemon、Socket 服务或 Python 绑定把它接到这条路径。
可选硬件总线适配器不改变这一事实；`qarm-viser --backend hardware` 明确拒绝启动。
Python 与 C++ 的状态逻辑需要端到端契约验证后才能视为同一套硬件运行链路。

## 模块边界

| 模块 | 责任 | 不负责 |
| --- | --- | --- |
| `qarm_viser` | 场景、操作面板、规划预览、显示后端结果 | 电机周期控制 |
| `qarm_control` | 领域记录、离线后端、命令和快照契约 | Unitree SDK |
| `qmini_arm_motion` | 模型、算法、离线命令映射和初步动力学 | 串口或网络服务 |
| `qarm_sim` | MuJoCo 及离线诊断 | 新生产控制入口 |
| `controller/` | C++ 状态机、控制计算、计划与反馈检查 | 浏览器 UI |
| `QminiArm::Core` | 重力、轨迹、保护、关节换算 | SDK |
| `QminiArm::Hardware` | MotorBus 与 Unitree SDK 交换 | IK 与 UI |

旧 `platform/` 的 React、HTTP 服务和部署脚本已从当前源码移除，可从 Git 历史恢复。
旧可视化入口 `qmini-motion viz` 转发到 `qarm-viser`，不保留第二套控制页面。
`qmini-motion fk/workspace/plan` 的 CSV 是离线计算产物，不是实机执行授权。

## 状态与身份

控制状态区分断开、只读、标零采集、标定有效、就绪、重力保持、执行、故障和急停。
标零采集只产生候选；提交后还需满足正常软限位才可以进入 READY。
桌面参考的 J2 超出正常软限位，不能把该参考自动作为普通运动目标。

标定记录绑定模型、控制器启动周期、电机 ID、方向、参考姿态和样本统计。
计划绑定模型与当前标定，并校验时间序列、关节限位、速度及起点一致性。
只有通过校验的计划 ID 才能执行；身份变更应使旧计划失效。

三个角度语义必须区分：`q_rotor` 是 SDK 累计转子角，`q_output_raw=q_rotor/r`
是未标定输出轴诊断角，`q_joint` 是应用方向和零位后的 URDF 关节角。
未标定 `q_joint` 为空；只有有效的 `q_joint` 可输入 FK、IK 与规划器。

## C++ 构建

默认只构建 SDK 无关核心与离线测试：

```bash
cmake -S . -B build-core
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure
```

Linux 上可单独编译硬件库和控制适配器，不启动设备：

```bash
cmake -S . -B build-hardware -DQARM_BUILD_HARDWARE=ON \
  -DUNITREE_ACTUATOR_SDK_ROOT=/absolute/path/to/unitree_actuator_sdk
cmake --build build-hardware --parallel 2
```

SDK 保持只读，路径解析位于 `cmake/UnitreeActuatorSDK.cmake`。
`QARM_BUILD_MAINTENANCE_APPS=ON` 仅用于显式构建历史台架工具，不安装成生产入口。
旧 `QMINI_ARM_BUILD_APPS` 作为维护构建的兼容选项保留，默认关闭。

## 尚需接通的硬件路径

```text
Viser → Python IPC client → 本机 Unix Socket → C++ daemon
                                               ↓
                                  ArmController + 总线适配器
                                               ↓
                                          MotorBus / SDK
```

此路径中的 IPC 客户端与 daemon 尚未实现。协议帧格式不能替代服务端的权限、
租约、重放、标定持久化和模型一致性检查；Socket 存在也不代表具备硬件执行能力。

硬件控制周期应由单一 C++ 进程拥有，并统一处理重力渐变、力矩上限与 slew、
速度/温度/反馈有效性、跟踪误差、总线超时和租约失效。
Viser 回调不得阻塞电机周期，断开浏览器也不能移交串口所有权。
M8010 BRAKE 会改变控制状态，且不是机械安全抱闸；现场标零必须有外部支撑。
