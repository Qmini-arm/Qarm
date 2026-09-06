# Qarm 四轴工程交接

## 当前模型与接口

- 工作分支为 `4-axis`；模型源为 `description/qmini_arm.urdf.xacro`，展开产物为
  `description/qmini_arm.urdf`。
- 活动关节严格为 `joint_1..joint_4`，电机映射默认 ID 0..3。保留的 `link_6`
  是工具安装支架名称，不是第六轴；第四轴原点旋转已改为 `rpy=-1.68822 0 -pi`。
- C++ 使用 `kJointCount=4` 和 `JointArray<T>`；Python 从活动 URDF 链和映射读取维度；
  网页从后端模型元数据构建关节控件。
- 关节 CSV 固定 9 列：时间、四轴位置、四轴速度。旧六轴轨迹和旧控制配置会明确拒绝。
- 位置 IK 约束 XYZ，不能把四轴位置可达等同于任意六维末端姿态可达。

## 已实现链路

1. C++ `MotorBus` 管理 Unitree SDK 串口交换；关节换算、安全、轨迹、重力模型
   保持独立。重力模型已删除两个旧腕部质量，并同步新的第四轴安装变换。
2. `qmini-motion` 提供 FK、位置 IK、无自碰撞工作空间、RRT-Connect、
   五次轨迹、转子命令映射和 Viser 仿真。
3. `qarm-sim` 提供四轴 MuJoCo 模型、渲染、遥测镜像、手动标零采集和离线回位验证。
4. `qmini_gravity_comp` 保留 shadow/FOC、逐轴力矩帽、Q8 slew、速度/温度/
   反馈/循环看门狗；`qmini_return_to_zero` 只执行完整受检关节轨迹。
5. 网页控制台及 HTTP 服务使用四轴模型和限位，后端失败会显示为失败，不能在前端
   伪造本地运动成功。网页的仿真状态模拟与 MuJoCo 动力学仿真是不同运行层。

## 标定状态

机构变化后旧六轴编码器捕获失效，包括第四轴坐标变化，不能截断旧文件继续控制。

- `config/joint_map.json`：四轴映射占位；零位、方向及综合标定均为 false。
- `config/m8010_arm.yaml`：四轴转子命令映射仍全部 `calibrated: false`；
  未标定时绝对转子命令为空。
- `config/gravity_comp.conf`：schema 3、`calibration_confirmed=false`，
  boot ID 为 `UNCALIBRATED`，转子参考值为占位零。离线 dry-run 可验证格式；
  硬件模式在打开串口前拒绝。
- `config/calibration_pose.json`：四轴几何参考为
  `[0, 1.7480178111, 0.1548064707, 0.0]` rad；
  现场确认标记 `validated=false`。前两处 STL 与桌面相切，第四轴保持 0° 机械标零，
  工具安装座离桌面约 17 mm。该文件不是编码器标定结果。
- `joint_2` 桌面参考超出运行软限位，但仍在硬限位内。数学 URDF 零位、
  手动桌面参考和当前上电周期的编码器零位必须区分。
- 重新捕获四轴编码器参考并确认方向、ID、几何参考后，才可更新部署配置和标定标记；
  开发板或任一电机掉电后必须重新采集。

## 验证与运行

```bash
.venv/bin/ruff check python tests/python
.venv/bin/pytest -q
.venv/bin/pytest -q platform/server/test_qarm_control_server.py

.venv/bin/qmini-motion fk --q-deg 0 0 0 0
.venv/bin/qmini-motion plan --start-deg 0 0 0 0 \
  --target 0.73 0.02 -0.04 --output build/m8010_commands.csv
.venv/bin/qarm-sim validate
.venv/bin/qarm-sim solve-calibration-pose
.venv/bin/qarm-sim plan-home --start-deg 10 5 10 5 \
  --output build/calibration_home.csv
```

网页启动与构建见 [platform/README.md](platform/README.md)，硬件 C++ 构建见
[README.md](README.md)。MuJoCo 回位测试使用未辨识的反射惯量、摩擦和延迟假设，
通过离线测试不表示已在四轴真机运行。当前重构不自动部署到开发板。

## 后续工作与边界

- 在可靠支撑下确认新机构 ID、方向、手动参考和有效限位，再完成当前上电周期标定。
- 重新构建并部署四轴控制器、只读采集器、配置和新 9 列轨迹；不能混用旧二进制、
  schema 2 配置或六轴 CSV。
- 进一步实现统一 C++ ArmController 和完整规划轨迹执行 adapter；网页任意 MOVEJ、
  实机使能与重力动作仍以服务端 capability 为准。
- 串口由单个控制进程拥有；Python/网页不逐周期拼接电机命令。
- 实机状态是 `q_joint`，SDK 原始转子角和未标定输出轴角必须明确标记，不能直接用于 IK。
- BRAKE 是释放保持，不是机械安全抱闸。硬件操作必须保留机械支撑、物理断电和显式确认。
- 不修改用户现有未跟踪的标定备份、临时 PDF 文件和 `hand_all.stl`，也不修改供应商 SDK。
