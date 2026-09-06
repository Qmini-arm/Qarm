# Qarm 控制平台架构

这份文档定义网页平台、仿真和真实 M8010 控制器之间的目标接口边界。
下文 ArmBackend、WebSocket 和 `/api/v1/*` 是后续设计，当前实现使用
`platform/server/qarm_control_server.py` 的 `/api/*` REST 和轮询式四轴状态。
当前仿真是控制状态模拟，MuJoCo 动力学验证通过 `qarm-sim` 独立运行；
硬件 MOVEJ、使能和重力模式尚未接入网页执行器，后端明确拒绝，前端显示真实错误。
当前接口和启动方式见 [平台说明](../platform/README.md)。

## 运行时边界

```text
浏览器 (React)
   │ REST: 配置、规划、任务      WebSocket: 状态/事件
   ▼
qarm-server (Python)
   │ ArmBackend 接口
   ├── MujocoBackend       # 默认，离线仿真
   └── HardwareBackend     # 通过 C++ adapter，单线程占有串口
          │
          ▼
   ArmController → TrajectoryExecutor → MotorBus → Unitree SDK
```

浏览器永远不能直接访问串口。`qarm-server` 负责鉴权、请求校验、规划调用和状态广播；
每周期的电机交换只能由 `ArmController` 所属线程完成。MuJoCo 与真实后端实现同一
`ArmBackend` 接口，因此 UI 切换仿真/实机时不需要改变控制逻辑。

## C++ 状态机

```text
Disconnected → ConnectedReadOnly → Homing → Calibrated → Ready
                                      │                    │
                                      └──── Fault ◄────────┤
Ready → GravityHold → Executing → Ready
  └──────────────→ EStop/Fault ──(人工复位)──→ ConnectedReadOnly
```

状态转换必须是单向、可审计的事件。`Ready` 的前提是四轴反馈有效、会话标定 ID 匹配、
当前姿态在有效限位内。`GravityHold` 使用重力前馈和速度阻尼；它不设置位置目标。
`Executing` 执行已经过限位、自碰撞、速度、加速度和起点匹配检查的完整轨迹。
任何通信、温度、错误码、看门狗或急停故障都进入 `Fault`，并在退出路径尝试发送
BRAKE；BRAKE 不是机械安全抱闸，外部支撑仍是必要条件。

建议新增的 C++ 公共接口：

```cpp
struct ArmSnapshot { State state; uint64_t sequence; Timestamp stamp;
  JointArray<JointState> joints; JointArray<MotorCommand> command;
  std::string calibration_id; Fault fault; };

class ArmController {
 public:
  ArmSnapshot snapshot() const;
  Result home(const HomeRequest&);
  Result holdGravity(double scale);
  Result execute(const JointTrajectory&, const ExecuteOptions&);
  void cancel();
  void estop();
};
```

`MotorBus` 仍然只负责协议和请求—应答；状态机、轨迹执行、关节/转子换算和安全策略不应
塞进 `MotorBus`。硬件 adapter 提供 fake 实现用于单元测试，测试中不需要 SDK 或串口。

## Python/HTTP 合约

Python 侧建议拆成 `python/qarm_server/{app.py,schemas.py,backends.py,events.py}`：

| 方法 | 路径 | 作用 | 必需前置状态 |
| --- | --- | --- | --- |
| GET | `/api/v1/state` | 当前状态快照 | 已连接或仿真 |
| GET | `/api/v1/config` | URDF、限位、工具和标定版本 | 任意 |
| POST | `/api/v1/connect` | 选择仿真或控制器 | Disconnected |
| POST | `/api/v1/home/plan` | 规划到桌面支撑标定位 | 有当前关节姿态 |
| POST | `/api/v1/home/execute` | 执行已审计回零轨迹 | Ready，且需确认 |
| POST | `/api/v1/gravity` | 开/关重力补偿及比例 | Ready/GravityHold |
| POST | `/api/v1/motion/plan` | FK/IK、碰撞和时间规划 | 已连接 |
| POST | `/api/v1/motion/execute` | 执行规划句柄 | Ready，且需确认 |
| POST | `/api/v1/stop` | 取消轨迹并 BRAKE | 任意已连接 |
| WS | `/api/v1/stream` | 10–100 Hz 快照和事件 | 已连接 |

所有写操作携带 `request_id`，服务端保存最近结果，重复请求不会重复下发动作。执行请求
引用不可变的 `plan_id`，并再次检查 `calibration_id`、起点容差、限位、碰撞和时间戳。
WebSocket 消息分为 `snapshot`、`command`、`fault`、`transition`、`audit` 五类，必须
带单调 `sequence` 和控制器时间戳，前端发现跳号时显示数据不连续。

快照至少包含：状态机状态、连接/使能/急停、四轴 `q/dq/tau/temp/error/mode`、
当前目标 `q_des/dq_des/tau_ff/kp/kd`、跟踪误差、重力比例、计划 ID、标定 ID、
最后一次故障和总线延迟。UI 应分别显示目标、反馈和误差，不能把计划值伪装成实测值。

## 平台功能分区

- 控制：关节 MOVEJ、末端位置规划、复制当前姿态、回到桌面支撑标定位；规划结果先在 Viser/MuJoCo 预览，再显式执行。
- 状态：四轴反馈、温度/错误码/通信新鲜度、跟踪误差、状态机和事件日志。
- 配置：工具坐标系（位姿、重量、重心）、有效软限位、控制器地址和模型/标定版本。配置写入临时版本，校验通过后原子替换。
- 在线编程：版本化 JSON AST，节点包括 `start/end/movej/movel/wait/set/if/loop/popup`；保存、校验、单步和运行都由后端执行器完成，不能让浏览器直接拼接串口命令。
- 可视化：Viser 展示 URDF/MuJoCo 当前姿态、目标、规划路径和桌面支撑平面；实机模式叠加实际反馈，断线时冻结并标记时间戳。

## 优先级

1. 先冻结 REST/WS schema，实现 MujocoBackend + FakeHardwareBackend 和状态机测试。
2. 再实现 C++ `ArmController`、四轴聚合反馈和轨迹执行 adapter。
3. 将 Viser 和 React UI 改为消费真实 schema，移除随机 mock 状态。
4. 最后加入在线编程持久化、权限、审计导出和控制器升级页面。
