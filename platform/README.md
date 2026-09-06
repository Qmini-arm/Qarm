# Qarm Control Platform

Qarm 四自由度 GO-M8010-6 机械臂控制台。轴数、关节名称、软限位来自当前
`description/qmini_arm.urdf`，电机 ID 和标定状态来自 `config/joint_map.json`。
当前链为 `joint_1` 到 `joint_4`，电机 ID 为 `0` 到 `3`。

## 本地运行

```bash
# 仓库根目录，终端 1：默认纯仿真，不打开串口
./platform/run_server.sh

# 终端 2
cd platform
npm ci
npm run dev
```

浏览器打开 <http://127.0.0.1:5173>。Vite 将 `/api` 转发到
`http://127.0.0.1:8090`；使用不同后端可设置 `VITE_API_URL`。
构建后也可直接由 `run_server.sh` 在 <http://127.0.0.1:8090> 提供网页。

## Viser 3D

控制页的关节角度区可以切换到 Viser 模型视图。先在另一个终端启动 Qarm Viser：

```bash
# 仓库根目录
UV_CACHE_DIR=/private/tmp/qarm_uv_cache uv run qmini-motion viz --host 127.0.0.1 --port 8080
```

默认 iframe 地址为 `http://127.0.0.1:8080`。如果 Viser 在另一台机器或端口：

```bash
VITE_VISER_URL=http://192.168.10.102:8080 npm run dev
```

仿真由控制服务提供。后端离线或拒绝动作时，网页保留失败状态，不会自动切换到浏览器内模拟成功。
目标角度与反馈独立保存；周期性反馈不会覆盖正在编辑的目标。Viser 是单独的模型查看器，
平台不会把 MOVEJ 目标推送给它，也不会把独立查看器的姿态当作硬件反馈。

## 四轴接口与标定

- `GET /api/status` 返回 `dof`、`joint_names`、四个 `joints`、`angle_space` 和 `capabilities`。
- `POST /api/movej` 只接受当前关节顺序的四个有限弧度值，并检查 URDF 软限位。
- 未重新标定的硬件反馈以 `uncalibrated_motor_output` 标识，不能视作 URDF 关节角。
- 六轴映射、六电机遥测包和六维轨迹会被拒绝。旧标定不能通过截断数组继续使用。
- `config/calibration_pose.json` 的 `validated` 当前为 `false`，回标定位被禁用。
  有效回标定位还要求准确的 `joint_names`、四维参考姿态；硬件模式额外要求映射的全部标定标志有效。
- 轨迹必须使用当前 Python 规划器的列名：`time_s`，四列 `joint_N_position_rad`，
  四列 `joint_N_velocity_rad_s`，并以已验证参考姿态结束。

在线编程支持四轴 MOVEJ 与 WAIT 节点的编辑、导入、校验及导出，不执行流程。
JSON 必须包含 `version: 1`、准确的 `joint_names` 和 `nodes`；六维旧流程导入被拒绝。
状态页可以导出当前四轴反馈 CSV，配置页显示实际模型限位。

离线验证：

```bash
uv run pytest platform/server/test_qarm_control_server.py
npm --prefix platform run build
```

## 在开发板上运行

平台服务可以和网页静态文件一起部署到开发板；浏览器只需访问开发板地址，电脑上不需要
启动 Node/Vite：

```bash
# 在开发电脑执行（默认 HwHiAiUser@192.168.10.102）
./platform/deploy_board.sh --no-start

# SSH 到板端启动。QARM_HARDWARE=1 启用四轴反馈能力；浏览器连接后开始 BRAKE 轮询。
# 未安装真实执行器时，
# 使能、重力补偿和 MOVEJ 会明确返回 501，不会伪报成功。
ssh HwHiAiUser@192.168.10.102 \
  'cd ~/qarm-platform && QARM_HARDWARE=1 QARM_PLATFORM_PORT=8090 \
   ./platform/run_server.sh'
```

然后打开 <http://192.168.10.102:8090/>。`deploy_board.sh` 会在本地构建 React bundle，
复制 `platform/dist`、控制服务、URDF、关节映射与参考姿态；板端只需要 Python 3.10+。
部署脚本不会复制旧的 `build/calibration_home.csv`。可用环境变量覆盖目标：
`QARM_BOARD_HOST`、`QARM_BOARD_USER`、`QARM_BOARD_ROOT`。如需在板端运行 Viser，另行启动
`qmini-motion viz --host 0.0.0.0 --port 8080`，并在构建时设置
`VITE_VISER_URL=http://192.168.10.102:8080`。
